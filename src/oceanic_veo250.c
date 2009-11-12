/* 
 * libdivecomputer
 * 
 * Copyright (C) 2008 Jef Driesen
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

#include <string.h> // memcpy
#include <stdlib.h> // malloc, free
#include <assert.h> // assert

#include "device-private.h"
#include "oceanic_common.h"
#include "oceanic_veo250.h"
#include "serial.h"
#include "utils.h"
#include "ringbuffer.h"
#include "checksum.h"

#define MAXRETRIES 2

#define EXITCODE(rc) \
( \
	rc == -1 ? DEVICE_STATUS_IO : DEVICE_STATUS_TIMEOUT \
)

#define ACK 0x5A
#define NAK 0xA5

typedef struct oceanic_veo250_device_t {
	oceanic_common_device_t base;
	struct serial *port;
	unsigned int last;
	unsigned char version[OCEANIC_VEO250_PACKET_SIZE];
} oceanic_veo250_device_t;

static device_status_t oceanic_veo250_device_set_fingerprint (device_t *abstract, const unsigned char data[], unsigned int size);
static device_status_t oceanic_veo250_device_version (device_t *abstract, unsigned char data[], unsigned int size);
static device_status_t oceanic_veo250_device_read (device_t *abstract, unsigned int address, unsigned char data[], unsigned int size);
static device_status_t oceanic_veo250_device_dump (device_t *abstract, dc_buffer_t *buffer);
static device_status_t oceanic_veo250_device_foreach (device_t *abstract, dive_callback_t callback, void *userdata);
static device_status_t oceanic_veo250_device_close (device_t *abstract);

static const device_backend_t oceanic_veo250_device_backend = {
	DEVICE_TYPE_OCEANIC_VEO250,
	oceanic_veo250_device_set_fingerprint, /* set_fingerprint */
	oceanic_veo250_device_version, /* version */
	oceanic_veo250_device_read, /* read */
	NULL, /* write */
	oceanic_veo250_device_dump, /* dump */
	oceanic_veo250_device_foreach, /* foreach */
	oceanic_veo250_device_close /* close */
};

static const oceanic_common_layout_t oceanic_veo250_layout = {
	0x0000, /* cf_devinfo */
	0x0040, /* cf_pointers */
	0x0400, /* rb_logbook_begin */
	0x0600, /* rb_logbook_end */
	0x0600, /* rb_profile_begin */
	0x8000, /* rb_profile_end */
	1 /* mode */
};


static int
device_is_oceanic_veo250 (device_t *abstract)
{
	if (abstract == NULL)
		return 0;

    return abstract->backend == &oceanic_veo250_device_backend;
}


static device_status_t
oceanic_veo250_send (oceanic_veo250_device_t *device, const unsigned char command[], unsigned int csize)
{
	// Discard garbage bytes.
	serial_flush (device->port, SERIAL_QUEUE_INPUT);

	// Send the command to the dive computer and 
	// wait until all data has been transmitted.
	serial_write (device->port, command, csize);
	serial_drain (device->port);

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_transfer (oceanic_veo250_device_t *device, const unsigned char command[], unsigned int csize, unsigned char answer[], unsigned int asize)
{
	assert (asize == OCEANIC_VEO250_PACKET_SIZE + 2);

	// Send the command to the device. If the device responds with an
	// ACK byte, the command was received successfully and the answer
	// (if any) follows after the ACK byte. If the device responds with
	// a NAK byte, we try to resend the command a number of times before
	// returning an error.

	unsigned int nretries = 0;
	unsigned char response = NAK;
	while (response == NAK) {
		// Send the command to the dive computer.
		device_status_t rc = oceanic_veo250_send (device, command, csize);
		if (rc != DEVICE_STATUS_SUCCESS) {
			WARNING ("Failed to send the command.");
			return rc;
		}

		// Receive the response (ACK/NAK) of the dive computer.
		int n = serial_read (device->port, &response, 1);
		if (n != 1) {
			WARNING ("Failed to receive the answer.");
			return EXITCODE (n);
		}

#ifndef NDEBUG
		if (response != ACK)
			message ("Received unexpected response (%02x).\n", response);
#endif

		// Abort if the maximum number of retries is reached.
		if (nretries++ >= MAXRETRIES)
			break;
	}

	// Verify the response of the dive computer.
	if (response != ACK) {
		WARNING ("Unexpected answer start byte(s).");
		return DEVICE_STATUS_PROTOCOL;
	}

	// Receive the answer of the dive computer.
	int n = serial_read (device->port, answer, OCEANIC_VEO250_PACKET_SIZE + 2);
	if (n != OCEANIC_VEO250_PACKET_SIZE + 2) {
		WARNING ("Failed to receive the answer.");
		return EXITCODE (n);
	}

	// Verify the checksum of the answer.
	unsigned char crc = answer[OCEANIC_VEO250_PACKET_SIZE];
	unsigned char ccrc = checksum_add_uint8 (answer, OCEANIC_VEO250_PACKET_SIZE, 0x00);
	if (crc != ccrc) {
		WARNING ("Unexpected answer CRC.");
		return DEVICE_STATUS_PROTOCOL;
	}

	// Verify the last byte of the answer.
	if (answer[OCEANIC_VEO250_PACKET_SIZE + 1] != NAK) {
		WARNING ("Unexpected answer byte.");
		return DEVICE_STATUS_PROTOCOL;
	}

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_init (oceanic_veo250_device_t *device)
{
	// Send the command to the dive computer.
	unsigned char command[2] = {0x55, 0x00};
	device_status_t rc = oceanic_veo250_send (device, command, sizeof (command));
	if (rc != DEVICE_STATUS_SUCCESS) {
		WARNING ("Failed to send the command.");
		return rc;
	}

	// Receive the answer of the dive computer.
	unsigned char answer[14] = {0};
	int n = serial_read (device->port, answer, sizeof (answer));
	if (n != sizeof (answer)) {
		WARNING ("Failed to receive the answer.");
		return EXITCODE (n);
	}

	// Verify the answer.
	const unsigned char response[14] = {
		0x50, 0x50, 0x53, 0x2D, 0x2D, 0x4F, 0x4B,
		0x5F, 0x56, 0x32, 0x2E, 0x30, 0x30, 0x00};
	if (memcmp (answer, response, sizeof (response)) != 0) {
		WARNING ("Unexpected answer byte(s).");
		return DEVICE_STATUS_PROTOCOL;
	}

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_quit (oceanic_veo250_device_t *device)
{
	// Send the command to the dive computer.
	unsigned char command[2] = {0x98, 0x00};
	device_status_t rc = oceanic_veo250_send (device, command, sizeof (command));
	if (rc != DEVICE_STATUS_SUCCESS) {
		WARNING ("Failed to send the command.");
		return rc;
	}

	return DEVICE_STATUS_SUCCESS;
}


device_status_t
oceanic_veo250_device_open (device_t **out, const char* name)
{
	if (out == NULL)
		return DEVICE_STATUS_ERROR;

	// Allocate memory.
	oceanic_veo250_device_t *device = (oceanic_veo250_device_t *) malloc (sizeof (oceanic_veo250_device_t));
	if (device == NULL) {
		WARNING ("Failed to allocate memory.");
		return DEVICE_STATUS_MEMORY;
	}

	// Initialize the base class.
	oceanic_common_device_init (&device->base, &oceanic_veo250_device_backend);

	// Set the default values.
	device->port = NULL;
	device->last = 0;
	memset (device->version, 0, sizeof (device->version));

	// Open the device.
	int rc = serial_open (&device->port, name);
	if (rc == -1) {
		WARNING ("Failed to open the serial port.");
		free (device);
		return DEVICE_STATUS_IO;
	}

	// Set the serial communication protocol (9600 8N1).
	rc = serial_configure (device->port, 9600, 8, SERIAL_PARITY_NONE, 1, SERIAL_FLOWCONTROL_NONE);
	if (rc == -1) {
		WARNING ("Failed to set the terminal attributes.");
		serial_close (device->port);
		free (device);
		return DEVICE_STATUS_IO;
	}

	// Set the timeout for receiving data (3000 ms).
	if (serial_set_timeout (device->port, 3000) == -1) {
		WARNING ("Failed to set the timeout.");
		serial_close (device->port);
		free (device);
		return DEVICE_STATUS_IO;
	}

	// Set the DTR and RTS lines.
	if (serial_set_dtr (device->port, 1) == -1 ||
		serial_set_rts (device->port, 1) == -1) {
		WARNING ("Failed to set the DTR/RTS line.");
		serial_close (device->port);
		free (device);
		return DEVICE_STATUS_IO;
	}

	// Give the interface 100 ms to settle and draw power up.
	serial_sleep (100);

	// Make sure everything is in a sane state.
	serial_flush (device->port, SERIAL_QUEUE_BOTH);

	// Initialize the data cable (PPS mode).
	oceanic_veo250_init (device);

	// Switch the device from surface mode into download mode. Before sending
	// this command, the device needs to be in PC mode (manually activated by
	// the user), or already in download mode.
	oceanic_veo250_device_version ((device_t *) device, device->version, sizeof (device->version));

	*out = (device_t*) device;

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_device_set_fingerprint (device_t *abstract, const unsigned char data[], unsigned int size)
{
	oceanic_common_device_t *device = (oceanic_common_device_t*) abstract;

	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	return oceanic_common_device_set_fingerprint (device, data, size);
}


static device_status_t
oceanic_veo250_device_close (device_t *abstract)
{
	oceanic_veo250_device_t *device = (oceanic_veo250_device_t*) abstract;

	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	// Switch the device back to surface mode.
	oceanic_veo250_quit (device);

	// Close the device.
	if (serial_close (device->port) == -1) {
		free (device);
		return DEVICE_STATUS_IO;
	}

	// Free memory.	
	free (device);

	return DEVICE_STATUS_SUCCESS;
}


device_status_t
oceanic_veo250_device_keepalive (device_t *abstract)
{
	oceanic_veo250_device_t *device = (oceanic_veo250_device_t*) abstract;

	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	// Send the command to the dive computer.
	unsigned char command[4] = {0x91,
		(device->last     ) & 0xFF, // low
		(device->last >> 8) & 0xFF, // high
		0x00};
	device_status_t rc = oceanic_veo250_send (device, command, sizeof (command));
	if (rc != DEVICE_STATUS_SUCCESS) {
		WARNING ("Failed to send the command.");
		return rc;
	}

	// Receive the answer of the dive computer.
	unsigned char answer[3] = {0};
	int n = serial_read (device->port, answer, sizeof (answer));
	if (n != sizeof (answer)) {
		WARNING ("Failed to receive the answer.");
		return EXITCODE (n);
	}

	// Verify the answer.
	if (answer[0] != ACK || answer[1] != NAK || answer[2] != NAK) {
		WARNING ("Unexpected answer byte(s).");
		return DEVICE_STATUS_PROTOCOL;
	}

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_device_version (device_t *abstract, unsigned char data[], unsigned int size)
{
	oceanic_veo250_device_t *device = (oceanic_veo250_device_t*) abstract;

	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	if (size < OCEANIC_VEO250_PACKET_SIZE)
		return DEVICE_STATUS_MEMORY;

	unsigned char answer[OCEANIC_VEO250_PACKET_SIZE + 2] = {0};
	unsigned char command[2] = {0x90, 0x00};
	device_status_t rc = oceanic_veo250_transfer (device, command, sizeof (command), answer, sizeof (answer));
	if (rc != DEVICE_STATUS_SUCCESS)
		return rc;

	memcpy (data, answer, OCEANIC_VEO250_PACKET_SIZE);

#ifndef NDEBUG
	answer[OCEANIC_VEO250_PACKET_SIZE] = 0;
	message ("VEO250ReadVersion()=\"%s\"\n", answer);
#endif

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_device_read (device_t *abstract, unsigned int address, unsigned char data[], unsigned int size)
{
	oceanic_veo250_device_t *device = (oceanic_veo250_device_t*) abstract;

	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	assert (address % OCEANIC_VEO250_PACKET_SIZE == 0);
	assert (size    % OCEANIC_VEO250_PACKET_SIZE == 0);

	// The data transmission is split in packages
	// of maximum $OCEANIC_VEO250_PACKET_SIZE bytes.

	unsigned int nbytes = 0;
	while (nbytes < size) {
		// Read the package.
		unsigned int number = address / OCEANIC_VEO250_PACKET_SIZE;
		unsigned char answer[OCEANIC_VEO250_PACKET_SIZE + 2] = {0};
		unsigned char command[6] = {0x20, 
				(number     ) & 0xFF, // low
				(number >> 8) & 0xFF, // high
				(number     ) & 0xFF, // low
				(number >> 8) & 0xFF, // high
				0};
		device_status_t rc = oceanic_veo250_transfer (device, command, sizeof (command), answer, sizeof (answer));
		if (rc != DEVICE_STATUS_SUCCESS)
			return rc;

		memcpy (data, answer, OCEANIC_VEO250_PACKET_SIZE);

		device->last = number;

#ifndef NDEBUG
		message ("VEO250Read(0x%04x,%d)=\"", address, OCEANIC_VEO250_PACKET_SIZE);
		for (unsigned int i = 0; i < OCEANIC_VEO250_PACKET_SIZE; ++i) {
			message("%02x", data[i]);
		}
		message("\"\n");
#endif

		nbytes += OCEANIC_VEO250_PACKET_SIZE;
		address += OCEANIC_VEO250_PACKET_SIZE;
		data += OCEANIC_VEO250_PACKET_SIZE;
	}

	return DEVICE_STATUS_SUCCESS;
}


static device_status_t
oceanic_veo250_device_dump (device_t *abstract, dc_buffer_t *buffer)
{
	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	// Erase the current contents of the buffer and
	// allocate the required amount of memory.
	if (!dc_buffer_clear (buffer) || !dc_buffer_resize (buffer, OCEANIC_VEO250_MEMORY_SIZE)) {
		WARNING ("Insufficient buffer space available.");
		return DEVICE_STATUS_MEMORY;
	}

	return device_dump_read (abstract, dc_buffer_get_data (buffer),
		dc_buffer_get_size (buffer), OCEANIC_VEO250_PACKET_SIZE);
}


static device_status_t
oceanic_veo250_device_foreach (device_t *abstract, dive_callback_t callback, void *userdata)
{
	oceanic_common_device_t *device = (oceanic_common_device_t*) abstract;

	if (! device_is_oceanic_veo250 (abstract))
		return DEVICE_STATUS_TYPE_MISMATCH;

	return oceanic_common_device_foreach (device, &oceanic_veo250_layout, callback, userdata);
}
