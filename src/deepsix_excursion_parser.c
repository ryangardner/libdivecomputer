/*
 * Deep6 Excursion parsing
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

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "deepsix_excursion.h"
#include "context-private.h"
#include "parser-private.h"
#include "array.h"

#define C_ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

#define MAXFIELDS 128

struct msg_desc;

typedef struct deepsix_excursion_parser_t {
	dc_parser_t base;

	dc_sample_callback_t callback;
	void *userdata;

	int sample_interval;

	char divetype;

	// surface pressure
	unsigned int surface_atm;
	char firmware_version[6];

} deepsix_excursion_parser_t;

static dc_status_t deepsix_excursion_parser_set_data (dc_parser_t *abstract, const unsigned char *data, unsigned int size);
static dc_status_t deepsix_excursion_parser_get_datetime (dc_parser_t *abstract, dc_datetime_t *datetime);
static dc_status_t deepsix_excursion_parser_get_field (dc_parser_t *abstract, dc_field_type_t type, unsigned int flags, void *value);
static dc_status_t deepsix_excursion_parser_samples_foreach (dc_parser_t *abstract, dc_sample_callback_t callback, void *userdata);

static const dc_parser_vtable_t deepsix_parser_vtable = {
		sizeof(deepsix_excursion_parser_t),
		DC_FAMILY_DEEPSIX,
		deepsix_excursion_parser_set_data, /* set_data */
		deepsix_excursion_parser_get_datetime, /* datetime */
		deepsix_excursion_parser_get_field, /* fields */
		deepsix_excursion_parser_samples_foreach, /* samples_foreach */
		NULL /* destroy */
};

dc_status_t
deepsix_excursion_parser_create (dc_parser_t **out, dc_context_t *context)
{
	deepsix_excursion_parser_t *parser = NULL;

	if (out == NULL)
		return DC_STATUS_INVALIDARGS;

	// Allocate memory.
	parser = (deepsix_excursion_parser_t *) dc_parser_allocate (context, &deepsix_parser_vtable);
	if (parser == NULL) {
		ERROR (context, "Failed to allocate memory.");
		return DC_STATUS_NOMEMORY;
	}

	*out = (dc_parser_t *) parser;

	return DC_STATUS_SUCCESS;
}

static double
pressure_to_depth(unsigned int mbar, unsigned int surface_pressure)
{
	// Specific weight of seawater (millibar to cm)
	const double specific_weight = 1.024 * 0.980665;

	// Absolute pressure, subtract surface pressure
	if (mbar < surface_pressure)
		return 0.0;
	mbar -= surface_pressure;
	return mbar / specific_weight / 100.0;
}

static dc_status_t
deepsix_excursion_parser_set_data (dc_parser_t *abstract, const unsigned char *data, unsigned int size)
{
	deepsix_excursion_parser_t *deepsix = (deepsix_excursion_parser_t *) abstract;
	const unsigned char *hdr = data;
	dc_gasmix_t gasmix = {0, };

	if (size < EXCURSION_HDR_SIZE)
		return DC_STATUS_IO;

	deepsix->callback = NULL;
	deepsix->userdata = NULL;

	// dive type - scuba = 0.
	// The use of an unsigned 32-bit integer certainly leaves them room to add a few more
	// modes, but we can safely cast this down to a char and not feel bad since there are only
	// four modes we care about
	deepsix->divetype = (char) array_uint32_le(&hdr[4]);

	int profile_data_len = array_uint32_le(&data[8]);
	memcpy(&(deepsix->firmware_version), &data[48], 6);

	// surface pressure
	deepsix->surface_atm = array_uint32_le(&hdr[56]);
	deepsix->sample_interval = array_uint32_le(&hdr[24]);

	return DC_STATUS_SUCCESS;
}

static dc_status_t
deepsix_excursion_parser_get_datetime (dc_parser_t *abstract, dc_datetime_t *datetime)
{
	deepsix_excursion_parser_t *deepsix = (deepsix_excursion_parser_t *) abstract;
	const unsigned char *data = deepsix->base.data;
	int len = deepsix->base.size;

	if (len < 256)
		return DC_STATUS_IO;
	datetime->year = data[12] + 2000;
	datetime->month = data[13];
	datetime->day = data[14];
	datetime->hour = data[15];
	datetime->minute = data[16];
	datetime->second = data[17];
	datetime->timezone = DC_TIMEZONE_NONE;

	return DC_STATUS_SUCCESS;
}

static dc_status_t
deepsix_excursion_parser_get_field (dc_parser_t *abstract, dc_field_type_t type, unsigned int flags, void *value)
{
	deepsix_excursion_parser_t *deepsix = (deepsix_excursion_parser_t *) abstract;

	// this is on the subsurface fork but not the mainline one
//    dc_field_string_t *string = (dc_field_string_t *) value;

	char string_buf[EXCURSION_SERIAL_NUMBER_LEN + 1];

	if (!value)
		return DC_STATUS_INVALIDARGS;

	switch (type) {
		case DC_FIELD_DIVETIME:
			*((unsigned int *) value) = array_uint32_le(deepsix->base.data + 20);
			break;
		case DC_FIELD_MAXDEPTH:
			*((double *) value) = pressure_to_depth(array_uint32_le(deepsix->base.data + 28), deepsix->surface_atm);
			break;
		case DC_FIELD_TEMPERATURE_MINIMUM:
			*((double *) value) = ((double)array_uint32_le(deepsix->base.data + 32))/10;
			break;
		case DC_FIELD_ATMOSPHERIC:
			*((double *) value) = (double) deepsix->surface_atm / 1000;
			break;
		case DC_FIELD_DIVEMODE:
			switch(array_uint32_le(deepsix->base.data + 4)) {
				case 0:
					*((dc_divemode_t *) value) = DC_DIVEMODE_OC;
					break;
				case 1:
					*((dc_divemode_t *) value) = DC_DIVEMODE_GAUGE;
					break;
				case 2:
					*((dc_divemode_t *) value) = DC_DIVEMODE_FREEDIVE;
					break;
				default:
					return DC_STATUS_UNSUPPORTED;
			}
			break;
		/* DC_FIELD_STRING is only on the subsurface branch */
//        case DC_FIELD_STRING:
//            switch (flags) {
//                case 0: /* serial */
//                    string->desc = "Serial";
//                    snprintf(string_buf, EXCURSION_SERIAL_NUMBER_LEN + 1, "%s", deepsix->base.data + EXCURSION_HDR_SIZE);
//                    break;
//                case 1: /* Firmware version */
//                    string->desc = "Firmware";
//                    snprintf(string_buf, 7, "%s", deepsix->firmware_version);
//                    break;
//                default:
//                    return DC_STATUS_UNSUPPORTED;
//            }
//            string->value = strdup(string_buf);
//            break;
		default:
			return DC_STATUS_UNSUPPORTED;
	}
	return DC_STATUS_SUCCESS;
}

static dc_status_t
deepsix_excursion_parser_samples_foreach (dc_parser_t *abstract, dc_sample_callback_t callback, void *userdata)
{
	deepsix_excursion_parser_t *deepsix = (deepsix_excursion_parser_t *) abstract;
	const unsigned char *data = deepsix->base.data;
	int len = deepsix->base.size, i = 0;

	deepsix->callback = callback;
	deepsix->userdata = userdata;

	// Skip the header information
	if (len < EXCURSION_HDR_SIZE+EXCURSION_SERIAL_NUMBER_LEN)
		return DC_STATUS_IO;
	data += EXCURSION_HDR_SIZE+EXCURSION_SERIAL_NUMBER_LEN;
	len -= EXCURSION_HDR_SIZE+EXCURSION_SERIAL_NUMBER_LEN;

	int nonempty_sample_count = 0;

	// the older firmware is parsed differently
	int firmware4c = memcmp(deepsix->firmware_version, "D01-4C", 6);
	while (i < len) {
		dc_sample_value_t sample = {0};
		char point_type = data[0];
		int near_end_of_data = (len - i <= 8);

		if (firmware4c == 0) {
			if (point_type != 1) {
				if (point_type != 2) {
					if (near_end_of_data) {
						break;
					} else {
						i++;
						data++;
						continue;;
					}
				}
				unsigned int pressure = array_uint16_le(data + 2);
				unsigned int something_else = array_uint16_le(data + 4);

				sample.time = (nonempty_sample_count) * deepsix->sample_interval;
				if (callback) callback(DC_SAMPLE_TIME, sample, userdata);

				sample.depth = pressure_to_depth(pressure, deepsix->surface_atm);
				if (callback) callback(DC_SAMPLE_DEPTH, sample, userdata);
				nonempty_sample_count++;

				if (something_else > 1300) {
					if (near_end_of_data) {
						break;
					}
					if (data[8] > 0 && data[8] < 3) {
						i += 8;
						data += 8;
						continue;
					}
				} else {
					if (something_else >= 10) {
						sample.temperature = something_else / 10.0;
						if (callback) callback(DC_SAMPLE_TEMPERATURE, sample, userdata);
					}
					if (near_end_of_data) {
						break;
					} else {
						i += 6;
						data += 6;
						if (data[0] <= 0 || data[0] >= 3) {
							i += 1;
							data += 1;
						}
					}
					continue;
				}
			} else {
				if (near_end_of_data) {
					break;
				}
				if (data[8] > 0 && data[8] < 3) {
					data += 8;
					i += 8;
					continue;
				}
			}
			data+=1;
			i+=1;
		}
		else {
			if (point_type == 2) {
				sample.time = (nonempty_sample_count) * deepsix->sample_interval;
				if (callback) callback(DC_SAMPLE_TIME, sample, userdata);

				unsigned int pressure = array_uint16_le(data + 2);
				unsigned int temp = array_uint16_le(data + 4);

				sample.depth = pressure_to_depth(pressure, deepsix->surface_atm);
				if (callback) callback(DC_SAMPLE_DEPTH, sample, userdata);

				sample.temperature = temp / 10.0;
				if (callback) callback(DC_SAMPLE_TEMPERATURE, sample, userdata);
				nonempty_sample_count++;
				if (data[6] <= 0 || data[6] >= 5) {
					data += 1;
					i++;
				} else {
					i += 6;
					data += 6;
				}

				continue;
			}
			// not sure what this point type indicates, but the phone app skips 8 bytes for it
			if (point_type == 1) {
				if (data[8] <= 0 || data[8] >= 5) {
					data += 1;
					i++;
				} else {
					i += 8;
					data += 8;
				}
				continue;
			}
//            if (point_type == 3) {
//                i += 6;
//                data += 4;
//
//                if (data[6] <= 0 || data[6] >= 5) {
//                    data += 1;
//                    i++;
//                } else {
//                    i += 6;
//                    data += 6;
//                }
//                continue;
//            }
			if (point_type != 3) {
				if (point_type != 4) {
					if (near_end_of_data) {
						break;
					} else {
						i++;
						data++;
						continue;
					}
				}
				if (point_type == 4) {
					if (near_end_of_data) {
						break;
					} else {
						i += 8;
						data += 8;
						if (data[i] > 0 && data[i] < 5) {
							continue;;
						}
					}
				}
			}
			else {
				if (near_end_of_data) {
					break;
				}
				else {
					i += 6;
					data += 6;
					if (data[6] > 0 && data[6] < 5) {
						continue;
					}
				}
			}
			data++;
			i++;
		}

	}

	return DC_STATUS_SUCCESS;
}
