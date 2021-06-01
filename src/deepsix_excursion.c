/*
 * libdivecomputer
 *
 * Copyright (C) 2020 Ryan Gardner
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "deepsix_excursion.h"
#include "context-private.h"
#include "device-private.h"
#include "platform.h"
#include "checksum.h"
#include "array.h"

#define MAXPACKET 255

#define NSTEPS    1000
#define STEP(i,n) (NSTEPS * (i) / (n))

#define FP_SIZE   6
#define FP_OFFSET 12

#define CMD_GROUP_LOGS     0xC0 // get the logs

#define CMD_GROUP_INFO                   0xA0 // info command group
#define COMMAND_INFO_LAST_DIVE_LOG_INDEX 0x04 // get the index of the last dive
#define COMMAND_INFO_SERIAL_NUMBER       0x03 // get the serial number

#define CMD_GROUP_SETTINGS  0xB0 // settings
#define CMD_SETTING_DATE    0x01 // date setting
#define CMD_SETTING_TIME    0x03 // time setting

// sub commands for the log
#define LOG_INFO    0x02
#define LOG_PROFILE 0x03 // the sub command for the dive profile info

typedef struct deepsix_device_t {
	dc_device_t base;
	dc_iostream_t *iostream;
	unsigned char fingerprint[FP_SIZE];
} deepsix_device_t;

static dc_status_t deepsix_excursion_device_set_fingerprint (dc_device_t *abstract, const unsigned char *data, unsigned int size);
static dc_status_t deepsix_excursion_device_foreach (dc_device_t *abstract, dc_dive_callback_t callback, void *userdata);
static dc_status_t deepsix_excursion_device_timesync(dc_device_t *abstract, const dc_datetime_t *datetime);
static dc_status_t deepsix_device_close (dc_device_t *abstract);

static const dc_device_vtable_t deepsix_device_vtable = {
		sizeof(deepsix_device_t),
		DC_FAMILY_DEEPSIX,
		deepsix_excursion_device_set_fingerprint, /* set_fingerprint */
		NULL, /* read */
		NULL, /* write */
		NULL, /* dump */
		deepsix_excursion_device_foreach, /* foreach */
		deepsix_excursion_device_timesync, /* timesync */
		deepsix_device_close, /* close */
};

static dc_status_t
deepsix_excursion_send (deepsix_device_t *device, unsigned char cmd, unsigned char subcmd, const unsigned char data[], unsigned int size)
{
	dc_status_t status = DC_STATUS_SUCCESS;
	dc_device_t *abstract = (dc_device_t *) device;
	unsigned char packet[4 + MAXPACKET + 1];

	if (device_is_cancelled (abstract))
		return DC_STATUS_CANCELLED;

	if (size > MAXPACKET)
		return DC_STATUS_INVALIDARGS;

	// Setup the data packet
	packet[0] = cmd;
	packet[1] = subcmd;
	packet[2] = 0x01;
	packet[3] = size;
	if (size) {
		memcpy(packet + 4, data, size);
	}
	packet[size + 4] = checksum_add_uint8 (packet, size + 4, 0) ^ 0xFF;

	// Send the data packet.
	status = dc_iostream_write (device->iostream, packet, 4 + size + 1, NULL);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (abstract->context, "Failed to send the command.");
		return status;
	}

	return status;
}

static dc_status_t
deepsix_excursion_recv (deepsix_device_t *device, unsigned char cmd, unsigned char subcmd, unsigned char data[], unsigned int size, unsigned int *actual)
{
	dc_status_t status = DC_STATUS_SUCCESS;
	dc_device_t *abstract = (dc_device_t *) device;
	unsigned char packet[4 + MAXPACKET + 1];
	size_t transferred = 0;

	// Read the packet header.
	status = dc_iostream_read (device->iostream, packet, 4, &transferred);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (abstract->context, "Failed to receive the packet header.");
		return status;
	}

	if (transferred < 4) {
		ERROR (abstract->context, "Packet header too short ("DC_PRINTF_SIZE").", transferred);
		return DC_STATUS_PROTOCOL;
	}

	if (packet[0] != cmd /*|| packet[1] != subcmd*/ || packet[2] != 0x01) {
		ERROR (device->base.context, "Unexpected packet header.");
		return DC_STATUS_PROTOCOL;
	}

	unsigned int len = packet[3];
	if (len > MAXPACKET) {
		ERROR (abstract->context, "Packet header length too large (%u).", len);
		return DC_STATUS_PROTOCOL;
	}

	// Read the packet payload and checksum.
	status = dc_iostream_read (device->iostream, packet + 4, len + 1, &transferred);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (abstract->context, "Failed to receive the packet data.");
		return status;
	}

	if (transferred < len + 1) {
		ERROR (abstract->context, "Packet data too short ("DC_PRINTF_SIZE").", transferred);
		return DC_STATUS_PROTOCOL;
	}

	// Verify the checksum.
	unsigned char csum = checksum_add_uint8 (packet, len + 4, 0) ^ 0xFF;
	if (packet[len + 4] != csum) {
		ERROR (abstract->context, "Unexpected packet checksum (%02x)", csum);
		return DC_STATUS_PROTOCOL;
	}

	if (len > size) {
		ERROR (abstract->context, "Unexpected packet length (%u).", len);
		return DC_STATUS_PROTOCOL;
	}

	memcpy(data, packet + 4, len);

	if (actual)
		*actual = len;

	return status;
}

static dc_status_t
deepsix_excursion_transfer (deepsix_device_t *device, unsigned char cmd, unsigned char subcmd, const unsigned char command[], unsigned int csize, unsigned char answer[], unsigned int asize, unsigned int *actual)
{
	dc_status_t status = DC_STATUS_SUCCESS;

	status = deepsix_excursion_send (device, cmd, subcmd, command, csize);
	if (status != DC_STATUS_SUCCESS)
		return status;

	status = deepsix_excursion_recv (device, cmd + 1, subcmd, answer, asize, actual);
	if (status != DC_STATUS_SUCCESS)
		return status;

	return status;
}

static dc_status_t
deepsix_excursion_device_foreach (dc_device_t *abstract, dc_dive_callback_t callback, void *userdata)
{
	dc_status_t status = DC_STATUS_SUCCESS;
	deepsix_device_t *device = (deepsix_device_t *) abstract;

	// Enable progress notifications.
	dc_event_progress_t progress = EVENT_PROGRESS_INITIALIZER;
	device_event_emit (abstract, DC_EVENT_PROGRESS, &progress);

	unsigned char rsp_serial[12] = {0};
	status = deepsix_excursion_transfer (device, CMD_GROUP_INFO, COMMAND_INFO_SERIAL_NUMBER, NULL, 0, rsp_serial, sizeof(rsp_serial), NULL);
	if (status != DC_STATUS_SUCCESS)
		return status;

	// Emit a device info event.
	dc_event_devinfo_t devinfo;
	devinfo.model = 0;
	devinfo.firmware = 0;
	devinfo.serial = array_convert_str2num (rsp_serial + 3, sizeof(rsp_serial) - 3);
	device_event_emit (abstract, DC_EVENT_DEVINFO, &devinfo);

	const unsigned char cmd_index[2] = {0};
	unsigned char rsp_index[2] = {0};
	status = deepsix_excursion_transfer (device, CMD_GROUP_INFO, COMMAND_INFO_LAST_DIVE_LOG_INDEX, cmd_index, sizeof(cmd_index), rsp_index, sizeof(rsp_index), NULL);
	if (status != DC_STATUS_SUCCESS)
		return status;

	// Calculate the number of dives.
	unsigned int ndives = array_uint16_le (rsp_index);

	// Update and emit a progress event.
	progress.maximum = ndives * NSTEPS;
	device_event_emit (abstract, DC_EVENT_PROGRESS, &progress);

	dc_buffer_t *buffer = dc_buffer_new(0);
	if (buffer == NULL) {
		return DC_STATUS_NOMEMORY;
	}

	for (unsigned int i = 0; i < ndives; ++i) {
		unsigned int number = ndives - i;

		const unsigned char cmd_header[] = {
				(number     ) & 0xFF,
				(number >> 8) & 0xFF};
		unsigned char rsp_header[EXCURSION_HDR_SIZE] = {0};
		status = deepsix_excursion_transfer (device, CMD_GROUP_LOGS, LOG_INFO, cmd_header, sizeof(cmd_header), rsp_header, sizeof(rsp_header), NULL);
		if (status != DC_STATUS_SUCCESS) {
			dc_buffer_free (buffer);
			return status;
		}

		if (memcmp(rsp_header + FP_OFFSET, device->fingerprint, sizeof(device->fingerprint)) == 0)
			break;

		unsigned int length = array_uint32_le (rsp_header + 8);

		// Update and emit a progress event.
		progress.current = i * NSTEPS + STEP(sizeof(rsp_header), sizeof(rsp_header) + length);
		device_event_emit (abstract, DC_EVENT_PROGRESS, &progress);

		dc_buffer_clear(buffer);
		dc_buffer_reserve(buffer, sizeof(rsp_header) + length);

		if (!dc_buffer_append(buffer, rsp_header, sizeof(rsp_header))) {
			ERROR (abstract->context, "Insufficient buffer space available.");
			dc_buffer_free(buffer);
			return DC_STATUS_NOMEMORY;
		}

		unsigned offset = 0;
		while (offset < length) {
			unsigned int len = 0;
			const unsigned char cmd_profile[] = {
					(number     ) & 0xFF,
					(number >> 8) & 0xFF,
					(offset      ) & 0xFF,
					(offset >>  8) & 0xFF,
					(offset >> 16) & 0xFF,
					(offset >> 24) & 0xFF};
			unsigned char rsp_profile[MAXPACKET] = {0};
			status = deepsix_excursion_transfer (device, CMD_GROUP_LOGS, LOG_PROFILE, cmd_profile, sizeof(cmd_profile), rsp_profile, sizeof(rsp_profile), &len);
			if (status != DC_STATUS_SUCCESS) {
				dc_buffer_free (buffer);
				return status;
			}

			unsigned int n = len;
			if (offset + n > length) {
				n = length - offset;
			}

			// Update and emit a progress event.
			progress.current = i * NSTEPS + STEP(sizeof(rsp_header) + offset + n, sizeof(rsp_header) + length);
			device_event_emit (abstract, DC_EVENT_PROGRESS, &progress);

			if (!dc_buffer_append(buffer, rsp_profile, n)) {
				ERROR (abstract->context, "Insufficient buffer space available.");
				dc_buffer_free(buffer);
				return DC_STATUS_NOMEMORY;
			}

			offset += n;
		}

		unsigned char *data = dc_buffer_get_data(buffer);
		unsigned int   size = dc_buffer_get_size(buffer);

		char divedata[25];
		sprintf(divedata, "divenumber=%d", i);
		HEXDUMP(device->base.context, DC_LOGLEVEL_DEBUG, divedata, (const unsigned char *) data, size);

		if (callback && !callback (data, size, data + FP_OFFSET, sizeof(device->fingerprint), userdata)) {
			dc_buffer_free (buffer);
			return DC_STATUS_SUCCESS;
		}
	}

	dc_buffer_free(buffer);

	return DC_STATUS_SUCCESS;
}

static dc_status_t
deepsix_excursion_device_timesync (dc_device_t *abstract, const dc_datetime_t *datetime)
{
	dc_status_t status = DC_STATUS_SUCCESS;
	deepsix_device_t *device = (deepsix_device_t *) abstract;

	if (datetime == NULL || datetime->year < 2000) {
		ERROR (abstract->context, "Invalid date/time value specified.");
		return DC_STATUS_INVALIDARGS;
	}

	const unsigned char cmd_date[] = {
			datetime->year - 2000,
			datetime->month,
			datetime->day};

	const unsigned char cmd_time[] = {
			datetime->hour,
			datetime->minute,
			datetime->second};

	status = deepsix_excursion_send (device, CMD_GROUP_SETTINGS, CMD_SETTING_DATE, cmd_date, sizeof(cmd_date));
	if (status != DC_STATUS_SUCCESS) {
		return status;
	}

	status = deepsix_excursion_send (device, CMD_GROUP_SETTINGS, CMD_SETTING_TIME, cmd_time, sizeof(cmd_time));
	if (status != DC_STATUS_SUCCESS) {
		return status;
	}

	return status;
}

dc_status_t
deepsix_excursion_device_open (dc_device_t **out, dc_context_t *context, dc_iostream_t *iostream)
{
	dc_status_t status = DC_STATUS_SUCCESS;
	deepsix_device_t *device = NULL;

	if (out == NULL)
		return DC_STATUS_INVALIDARGS;

	// Allocate memory.
	device = (deepsix_device_t *) dc_device_allocate (context, &deepsix_device_vtable);
	if (device == NULL) {
		ERROR (context, "Failed to allocate memory.");
		return DC_STATUS_NOMEMORY;
	}

	// Set the default values.
	device->iostream = iostream;
	memset(device->fingerprint, 0, sizeof(device->fingerprint));

	// Set the serial communication protocol (115200 8N1).
	status = dc_iostream_configure (device->iostream, 115200, 8, DC_PARITY_NONE, DC_STOPBITS_ONE, DC_FLOWCONTROL_NONE);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (context, "Failed to set the terminal attributes.");
		//goto error_free;
	}

	// Set the timeout for receiving data (1000ms).
	status = dc_iostream_set_timeout (device->iostream, 1000);
	if (status != DC_STATUS_SUCCESS) {
		ERROR (context, "Failed to set the timeout.");
		//goto error_free;
	}

	// Make sure everything is in a sane state.
	dc_iostream_sleep (device->iostream, 300);
	dc_iostream_purge (device->iostream, DC_DIRECTION_ALL);

	*out = (dc_device_t *) device;

	return DC_STATUS_SUCCESS;
}

static dc_status_t
deepsix_excursion_device_set_fingerprint (dc_device_t *abstract, const unsigned char *data, unsigned int size)
{
	deepsix_device_t *device = (deepsix_device_t *)abstract;

	HEXDUMP(device->base.context, DC_LOGLEVEL_DEBUG, "set_fingerprint", data, size);

	if (size && size != sizeof (device->fingerprint))
		return DC_STATUS_INVALIDARGS;

	if (size)
		memcpy (device->fingerprint, data, sizeof (device->fingerprint));
	else
		memset (device->fingerprint, 0, sizeof (device->fingerprint));

	return DC_STATUS_SUCCESS;
}


static dc_status_t
deepsix_device_close (dc_device_t *abstract)
{
	deepsix_device_t *device = (deepsix_device_t *) abstract;

	return DC_STATUS_SUCCESS;
}
