/*
MIT License

Copyright (c) John Blaiklock 2020 Boat Data Hub

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "settings.h"
#include "flash.h"

static const settings_data_t settings_data_init =
{
	WRITE_MARKER,
	50U,
	false,
	20U,
	false,
	20U,
	false,
	10U,
	false,
	25U,
	false,
	5U,
	2U,
	true,
	false,
	10U
};

static settings_data_t settings_data;

void settings_save(void)
{
	flash_store_data(&settings_data, sizeof(settings_data_t));
}

void settings_load(void)
{
	(void)memcpy(&settings_data, (void *)(FLASH_START_ADDRESS), sizeof(settings_data_t));

	if (settings_data.write_marker != WRITE_MARKER)
	{
		settings_init();
	}
}

void settings_init(void)
{
	(void)memcpy(&settings_data, &settings_data_init, sizeof(settings_data_t));
	settings_save();
}

uint8_t settings_get_backlight(void)
{
	return settings_data.backlight;
}

void settings_set_backlight(uint8_t backlight)
{
	settings_data.backlight = backlight;
}

bool settings_get_depth_enabled(void)
{
	return settings_data.depth_enabled;
}

void settings_set_depth_enabled(bool depth_enabled)
{
	settings_data.depth_enabled = depth_enabled;
}

bool settings_get_distance_enabled(void)
{
	return settings_data.distance_enabled;
}

void settings_set_distance_enabled(bool distance_enabled)
{
	settings_data.distance_enabled = distance_enabled;
}

bool settings_get_wind_enabled(void)
{
	return settings_data.wind_enabled;
}

void settings_set_wind_enabled(bool wind_enabled)
{
	settings_data.wind_enabled = wind_enabled;
}

bool settings_get_heading_enabled(void)
{
	return settings_data.heading_enabled;
}

void settings_set_heading_enabled(bool heading_enabled)
{
	settings_data.heading_enabled = heading_enabled;
}

bool settings_get_sog_enabled(void)
{
	return settings_data.sog_enabled;
}

void settings_set_sog_enabled(bool sog_enabled)
{
	settings_data.sog_enabled = sog_enabled;
}

void settings_set_pressure_change_enabled(bool pressure_change_enabled)
{
	settings_data.pressure_change_enabled = pressure_change_enabled;
}

bool settings_get_pressure_change_enabled(void)
{
	return settings_data.pressure_change_enabled;
}

void settings_set_max_distance(uint8_t max_distance)
{
	settings_data.max_distance = max_distance;
}

uint8_t settings_get_max_distance(void)
{
	return settings_data.max_distance;
}

void settings_set_min_depth(uint16_t min_depth)
{
	settings_data.min_depth = min_depth;
}

uint16_t settings_get_min_depth(void)
{
	return settings_data.min_depth;
}

void settings_set_max_sog(uint8_t max_sog)
{
	settings_data.max_sog = max_sog;
}

uint8_t settings_get_max_sog(void)
{
	return settings_data.max_sog;
}

void settings_set_max_wind(uint8_t max_wind)
{
	settings_data.max_wind = max_wind;
}

uint8_t settings_get_max_wind(void)
{
	return settings_data.max_wind;
}

void settings_set_max_heading(uint8_t max_heading)
{
	settings_data.max_heading = max_heading;
}

uint8_t settings_get_max_heading(void)
{
	return settings_data.max_heading;
}

void settings_set_rearm_time(uint8_t rearm_time)
{
	settings_data.rearm_time = rearm_time;
}

uint8_t settings_get_rearm_time(void)
{
	return settings_data.rearm_time;
}

void settings_set_gps_from_seatalk(bool gps_from_seatalk)
{
	settings_data.gps_from_seatalk = gps_from_seatalk;
}

bool settings_get_gps_from_seatalk(void)
{
	return settings_data.gps_from_seatalk;
}

void settings_set_max_pressure_change(uint8_t max_pressure_change)
{
	settings_data.max_pressure_change = max_pressure_change;
}

uint8_t settings_get_max_pressure_change(void)
{
	return settings_data.max_pressure_change;
}

bool settings_get_any_alarm_enabled(void)
{
	return settings_data.depth_enabled || settings_data.distance_enabled || settings_data.heading_enabled || settings_data.pressure_change_enabled || settings_data.sog_enabled || settings_data.wind_enabled;
}
