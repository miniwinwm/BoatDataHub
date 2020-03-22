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

#ifndef SETTINGS_H
#define SETTINGS_H

#define WRITE_MARKER				0xFEEDD095UL	// they are hungry!

typedef struct
{
	uint32_t write_marker;

	uint8_t max_distance;
	bool distance_enabled;
	uint16_t min_depth;

	bool depth_enabled;
	uint8_t max_wind;
	bool wind_enabled;
	uint8_t max_sog;

	bool sog_enabled;
	uint8_t max_heading;
	bool heading_enabled;
	uint8_t rearm_time;

	uint8_t backlight;
	uint8_t gps_from_seatalk;
	bool pressure_change_enabled;
	uint8_t max_pressure_change;

} settings_data_t;

void settings_load(void);
void settings_save(void);
void settings_init(void);

uint8_t settings_get_backlight(void);
void settings_set_backlight(uint8_t backlight);
bool settings_get_depth_enabled(void);
void settings_set_depth_enabled(bool depth_enabled);
bool settings_get_distance_enabled(void);
void settings_set_distance_enabled(bool distance_enabled);
bool settings_get_wind_enabled(void);
void settings_set_wind_enabled(bool wind_enabled);
bool settings_get_heading_enabled(void);
void settings_set_heading_enabled(bool heading_enabled);
bool settings_get_sog_enabled(void);
void settings_set_sog_enabled(bool sog_enabled);
void settings_set_pressure_change_enabled(bool pressure_change_enabled);
bool settings_get_pressure_change_enabled(void);
void settings_set_max_distance(uint8_t max_distance);
uint8_t settings_get_max_distance(void);
void settings_set_min_depth(uint16_t min_depth);
uint16_t settings_get_min_depth(void);
void settings_set_max_sog(uint8_t max_sog);
uint8_t settings_get_max_sog(void);
void settings_set_max_wind(uint8_t max_wind);
uint8_t settings_get_max_wind(void);
void settings_set_max_heading(uint8_t max_heading);
uint8_t settings_get_max_heading(void);
void settings_set_rearm_time(uint8_t rearm_time);
uint8_t settings_get_rearm_time(void);
void settings_set_gps_from_seatalk(bool gps_from_seatalk);
bool settings_get_gps_from_seatalk(void);
void settings_set_max_pressure_change(uint8_t max_pressure_change);
uint8_t settings_get_max_pressure_change(void);
bool settings_get_any_alarm_enabled(void);
#endif
