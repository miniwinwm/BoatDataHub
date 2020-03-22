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
#include <stdlib.h>
#include "lcd.h"
#include "backlight.h"
#include "buttons.h"
#include "watcher.h"
#include "timer.h"
#include "buzzer.h"
#include "seatalk.h"
#include "mathematics.h"
#include "settings.h"
#include "mouse.h"
#include "main.h"
#include "printf.h"

#define MAIN_MENU_COUNT 		(sizeof(main_menu_items) / sizeof(const char *))
#define ALL_DATA_AVAILABLE		0xffU

typedef enum
{
	WAITING_FOR_DATA,
	STANDBY,
	WATCHING,
	ALARM,
	SETUP
} app_state_t;

typedef enum
{
	SETUP_STATE_MAIN,
	SETUP_STATE_DEPTH,
	SETUP_STATE_DEPTH_VALUE,
	SETUP_STATE_SOG,
	SETUP_STATE_SOG_VALUE,
	SETUP_STATE_WIND_SPEED,
	SETUP_STATE_WIND_SPEED_VALUE,
	SETUP_STATE_HEADING,
	SETUP_STATE_HEADING_VALUE,
	SETUP_STATE_POSITION,
	SETUP_STATE_POSITION_VALUE,
	SETUP_STATE_PRESSURE_CHANGE,
	SETUP_STATE_PRESSURE_CHANGE_VALUE,
	SETUP_STATE_ALARM_TEST
} setup_state_t;

typedef enum
{
	ALARM_NONE,
	ALARM_NO_DATA,
	ALARM_DEPTH,
	ALARM_SOG,
	ALARM_WIND,
	ALARM_DISTANCE,
	ALARM_HEADING,
	ALARM_PRESSURE_CHANGE
} alarm_t;

typedef enum
{
	MAIN_MENU_DEPTH = 0,
	MAIN_MENU_SOG,
	MAIN_MENU_WINDSPEED,
	MAIN_MENU_HEADING,
	MAIN_MENU_POSITION,
	MAIN_MENU_PRESSURE,
	MAIN_MENU_BACKLIGHT,
	MAIN_MENU_REARM_TIME,
	MAIN_MENU_GPS_SOURCE,
	MAIN_MENU_ALARM_TEST,

} main_menu_items_t;

static const char main_menu_1[] = "DEPTH";
static const char main_menu_2[] = "SOG";
static const char main_menu_3[] = "WIND SPEED";
static const char main_menu_4[] = "HEADING " LCD_TURN_CHARACTER;
static const char main_menu_5[] = "POSITION " LCD_MOVE_CHARACTER;
static const char main_menu_6[] = "PRESSURE";
static const char main_menu_7[] = "BACKLIGHT";
static const char main_menu_8[] = "REARM TIME";
static const char main_menu_9[] = "GPS SOURCE";
static const char main_menu_10[] = "ALARM TEST";
static const char * const main_menu_items[] = {main_menu_1, main_menu_2, main_menu_3, main_menu_4, main_menu_5,
													main_menu_6, main_menu_7, main_menu_8, main_menu_9, main_menu_10};

static uint8_t current_main_menu_position;
static int16_t heading_difference;
static uint8_t wind_alarm_count;
static uint8_t depth_alarm_count;
static uint8_t sog_alarm_count;
static uint8_t distance_alarm_count;
static uint8_t heading_alarm_count;
static uint32_t earliest_next_alarm_time;
static int16_t heading_ref;
static float latitude_ref;
static float longitude_ref;
static float pressure_ref;
static alarm_t alarm_type;
static float distance_m;
static float pressure_change;
static app_state_t app_state = WAITING_FOR_DATA;
static setup_state_t setup_state;
static char text_line[17];

static void show_standby_display(void);
static void show_waiting_for_data_display(void);
static void do_waiting_for_data_state(void);
static void enter_setup_state(void);
static void show_setup_display(void);
static int get_heading_error(int16_t initial, int16_t final);
static void do_standby_state(void);
static void do_alarm_state(void);
static void show_watching_display(void);
static void do_watching_state(void);
static void show_alarm_display(void);
static uint8_t check_data_availability(void);
static void do_setup_state(void);

static int get_heading_error(int16_t initial, int16_t final)
{
	int16_t clock_wise = final - initial;
	int16_t counter_clock_wise = 360 - final + initial;

	return abs((abs((int)clock_wise) <= abs((int)counter_clock_wise)) ? (int)clock_wise : (int)-counter_clock_wise);
}

static void show_waiting_for_data_display(void)
{
	lcd_clear();
	lcd_puts(0U, 0U, "NO DATA    SETUP");
}

static void do_waiting_for_data_state(void)
{
	if (buttons_get_value() == TOP_BUTTON)
	{
		enter_setup_state();
	}
	else if (check_data_availability() == ALL_DATA_AVAILABLE)
	{
		app_state = STANDBY;
		show_standby_display();
	}
}

static void enter_setup_state(void)
{
	app_state = SETUP;
	setup_state = SETUP_STATE_MAIN;
	current_main_menu_position = 0U;
	show_setup_display();
}

static void show_standby_display(void)
{
	lcd_clear();
	lcd_puts(0U, 0U, "STANDBY    SETUP");

	if (settings_get_any_alarm_enabled())
	{
		lcd_puts(1U, 11U, "START");
	}
}

void watcher_init(void)
{
	show_waiting_for_data_display();
}

void watcher_process(void)
{
	switch (app_state)
	{
		case WAITING_FOR_DATA:
			do_waiting_for_data_state();
			break;

		case STANDBY:
			do_standby_state();
			break;

		case WATCHING:
			do_watching_state();
			break;

		case ALARM:
			do_alarm_state();
			break;

		case SETUP:
			do_setup_state();
			break;
	}
}

static void do_watching_state(void)
{
	uint8_t buttons = buttons_get_value();
	static uint32_t last_display_update_time;
	uint32_t time_ms = timer_get_time_s();

	if (buttons == TOP_BUTTON)
	{
		app_state = STANDBY;
		show_standby_display();
	}
	else
	{
		if (time_ms - last_display_update_time >= 2UL)
		{
			show_watching_display();
			last_display_update_time = time_ms;
		}

		if (time_ms > earliest_next_alarm_time)
		{
			if (settings_get_distance_enabled())
			{
				// check distance from set point
				distance_m = distance_between_points(get_latitude_dual_source_data(), get_longitude_dual_source_data(), latitude_ref, longitude_ref);
				if (distance_m > settings_get_max_distance() && distance_m < 1000.0f)
				{
					distance_alarm_count++;
					if (distance_alarm_count > 9U)
					{
						alarm_type = ALARM_DISTANCE;
					}
				}
				else
				{
					distance_alarm_count = 0U;
				}
			}

			if (settings_get_depth_enabled())
			{
				// check depth
				if (seatalk_depth_data_retrieve() < (((float)settings_get_min_depth()) / 10.0f))
				{
					depth_alarm_count++;
					if (depth_alarm_count > 4U)
					{
						alarm_type = ALARM_DEPTH;
					}
				}
				else
				{
					depth_alarm_count = 0U;
				}
			}

			if (settings_get_sog_enabled())
			{
				// check speed over ground
				if (get_sog_dual_source_data() > ((float)settings_get_max_sog()) / 10.0f)
				{
					sog_alarm_count++;
					if (sog_alarm_count > 4U)
					{
						alarm_type = ALARM_SOG;
					}
				}
				else
				{
					sog_alarm_count = 0U;
				}
			}

			if (settings_get_wind_enabled())
			{
				if ((uint8_t)seatalk_apparent_wind_speed_retrieve() > settings_get_max_wind())
				{
					wind_alarm_count++;
					if (wind_alarm_count > 2U)
					{
						alarm_type = ALARM_WIND;
					}
				}
				else
				{
					wind_alarm_count = 0U;
				}
			}

			if (settings_get_heading_enabled())
			{
				// check heading difference from set point
				heading_difference = get_heading_error(seatalk_heading_magnetic_retrieve(), heading_ref);
				if (heading_difference > settings_get_max_heading() && heading_difference <= 180)
				{
					heading_alarm_count++;
					if (heading_alarm_count > 4U)
					{
						alarm_type = ALARM_HEADING;
					}
				}
				else
				{
					heading_alarm_count = 0U;
				}
			}

			if (settings_get_pressure_change_enabled())
			{
				pressure_change = pressure_ref - get_pressure_data();
				if (pressure_change > settings_get_max_pressure_change() || pressure_change < -settings_get_max_pressure_change())
				{
					alarm_type = ALARM_PRESSURE_CHANGE;
				}
			}

			// no data alarm situation cannot override an anchor alarm
			if (alarm_type == ALARM_NONE && check_data_availability() != ALL_DATA_AVAILABLE)
			{
				alarm_type = ALARM_NO_DATA;
			}

			if (alarm_type != ALARM_NONE)
			{
				app_state = ALARM;
				show_alarm_display();
				buzzer_on();
			}
		}
	}
}

static uint8_t check_data_availability(void)
{
	uint8_t data_available = ALL_DATA_AVAILABLE;

	if (settings_get_distance_enabled())
	{
		// check lat and long is being received
		if (get_latitiude_data_age_s() > MAX_DATA_AGE_S || get_longitiude_data_age_s() > MAX_DATA_AGE_S)
		{
			data_available &= ~0x01U;
		}
	}

	if (settings_get_depth_enabled())
	{
		// check depth is being received
		if (get_depth_data_age_s() > MAX_DATA_AGE_S)
		{
			data_available &= ~0x02U;
		}
	}

	if (settings_get_sog_enabled())
	{
		// check speed over ground is being received
		if (get_sog_data_age_s() > MAX_DATA_AGE_S)
		{
			data_available &= ~0x04U;
		}
	}

	if (settings_get_wind_enabled())
	{
		// check wind speed is being received
		if (get_aws_data_age_s() > MAX_DATA_AGE_S)
		{
			data_available &= ~0x08U;
		}
	}

	if (settings_get_heading_enabled())
	{
		// check heading is being received
		if (get_heading_data_age_s() > MAX_DATA_AGE_S)
		{
			data_available &= ~0x10U;
		}
	}

	if (settings_get_pressure_change_enabled())
	{
		// check pressure is being received
		if (get_pressure_data_age_s() > MAX_PRESSURE_DATA_AGE_S)
		{
			data_available &= ~0x20U;
		}
	}

	return data_available;
}

static void show_alarm_display(void)
{
	lcd_clear();

	switch (alarm_type)
	{
		case ALARM_NO_DATA:
			lcd_puts(0U, 0U, "DATA ALARM    OK");
			text_line[0] = '\0';
			break;

		case ALARM_DEPTH:
			lcd_puts(0U, 0U, "DEPTH ALARM   OK");
			(void)snprintf(text_line, sizeof(text_line), "DEPTH %3.1f m",  seatalk_depth_data_retrieve());
			break;

		case ALARM_SOG:
			lcd_puts(0U, 0U, "SOG ALARM     OK");
			(void)snprintf(text_line, sizeof(text_line), "SOG %2.1f Kts",  get_sog_dual_source_data());
			break;

		case ALARM_WIND:
			lcd_puts(0U, 0U, "WIND ALARM    OK");
			(void)snprintf(text_line, sizeof(text_line), "WINDSPEED %u Kts",  (unsigned int)seatalk_apparent_wind_speed_retrieve());
			break;

		case ALARM_DISTANCE:
			lcd_puts(0U, 0U, "DIST ALARM    OK");
			(void)snprintf(text_line, sizeof(text_line), "DISTANCE %u m",  (unsigned int)distance_m);
			break;

		case ALARM_HEADING:
			lcd_puts(0U, 0U, "HEADING ALARM OK");
			(void)snprintf(text_line, sizeof(text_line), "HEADING " LCD_TURN_CHARACTER " %u" LCD_DEGREE_CHARACTER,  get_heading_error(seatalk_heading_magnetic_retrieve(), heading_ref));
			break;

		case ALARM_PRESSURE_CHANGE:
			lcd_puts(0U, 0U, "PRESSURE ALRM OK");
			(void)snprintf(text_line, sizeof(text_line), "PRESSURE %u mb", (uint32_t)get_pressure_data);
			break;

		default:
			break;
	}

	lcd_puts(1U, 0U, text_line);
}

static void do_alarm_state(void)
{
	uint8_t buttons = buttons_get_value();
	uint32_t time_s = timer_get_time_s();
	uint8_t backlight = settings_get_backlight();

	if (alarm_type != ALARM_NONE)
	{
		if (time_s % 2U == 0U)
		{
			backlight_set(backlight);
		}
		else
		{
			if (backlight < BACKLIGHT_MAX)
			{
				backlight_set(backlight + 1U);
			}
			else
			{
				backlight_set(BACKLIGHT_MAX - 1U);
			}
		}
	}

	if (buttons == TOP_BUTTON)
	{
		if (alarm_type == ALARM_PRESSURE_CHANGE)
		{
			pressure_ref = get_pressure_data();
		}
		alarm_type = ALARM_NONE;
		backlight_set(backlight);
		earliest_next_alarm_time = time_s + ((uint32_t)settings_get_rearm_time()) * 60UL + 5UL;

		if (check_data_availability() == ALL_DATA_AVAILABLE)
		{
			app_state = WATCHING;
			show_watching_display();
		}
		else
		{
			app_state = WAITING_FOR_DATA;
			show_waiting_for_data_display();
		}
		buzzer_off();
	}
	else if (alarm_type == ALARM_NO_DATA)
	{
		if (check_data_availability() == ALL_DATA_AVAILABLE)
		{
			// this alarm self cancels if alarm situation clears
			alarm_type = ALARM_NONE;
			buzzer_off();
			app_state = WATCHING;
			show_watching_display();
		}
		else
		{
			if (time_s % 2UL == 0UL)
			{
				buzzer_on();
			}
			else
			{
				buzzer_off();
			}
		}
	}
}

static void show_watching_display(void)
{
	static uint8_t next_data = 0UL;

	lcd_clear();

	if (timer_get_time_s() > earliest_next_alarm_time)
	{
		lcd_puts(0U, 0U, "WATCHING    STOP");
	}
	else
	{
		lcd_puts(0U, 0U, "RE-ARMING   STOP");
	}

	switch (next_data)
	{
	case 0U:
		if (!settings_get_depth_enabled())
		{
			(void)strcpy(text_line, "DEPTH OFF");
		}
		else
		{
			(void)snprintf(text_line, sizeof(text_line), "DEPTH %3.1f m",  seatalk_depth_data_retrieve());
		}
		break;

	case 1U:
		if (!settings_get_sog_enabled())
		{
			(void)strcpy(text_line, "SOG OFF");
		}
		else
		{
			(void)snprintf(text_line, sizeof(text_line), "SOG %2.1f Kts",  get_sog_dual_source_data());
		}
		break;

	case 2UL:
		if (!settings_get_wind_enabled())
		{
			(void)strcpy(text_line, "WINDSPEED OFF");
		}
		else
		{
			(void)snprintf(text_line, sizeof(text_line), "WINDSPEED %u Kts",  (unsigned int)seatalk_apparent_wind_speed_retrieve());
		}
		break;

	case 3UL:
		if (!settings_get_heading_enabled())
		{
			(void)strcpy(text_line, "HEADING " LCD_TURN_CHARACTER " OFF");
		}
		else
		{
			(void)snprintf(text_line, sizeof(text_line), "HEADING " LCD_TURN_CHARACTER " %u " LCD_DEGREE_CHARACTER,  get_heading_error(seatalk_heading_magnetic_retrieve(), heading_ref));
		}
		break;

	case 4U:
		if (!settings_get_distance_enabled())
		{
			(void)strcpy(text_line, "POSITION " LCD_MOVE_CHARACTER " OFF");
		}
		else
		{
			if (((unsigned int)distance_m) < 1000UL)
			{
				(void)snprintf(text_line, sizeof(text_line), "POSITION " LCD_MOVE_CHARACTER " %u m",  (unsigned int)distance_m);
			}
			else
			{
				(void)strcpy(text_line, "POSITION " LCD_MOVE_CHARACTER " > 1km");
			}
		}
		break;

	case 5U:
		if (!settings_get_pressure_change_enabled())
		{
			(void)strcpy(text_line, "PRESSURE OFF");
		}
		else
		{
			if (pressure_change > -0.99f && pressure_change < 0.99f)
			{
				(void)snprintf(text_line, sizeof(text_line), "PRESS - %u mb", (uint32_t)get_pressure_data());
			}
			else if (pressure_change < 0.5f)
			{
				(void)snprintf(text_line, sizeof(text_line), "PRESS " LCD_DOWN_CHARACTER " %u mb", (uint32_t)get_pressure_data());
			}
			else
			{
				(void)snprintf(text_line, sizeof(text_line), "PRESS " LCD_UP_CHARACTER " %u mb", (uint32_t)get_pressure_data());
			}
		}
		break;
	}

	next_data++;
	if (next_data == 6U)
	{
		next_data = 0U;
	}

	lcd_puts(1U, 0U, text_line);
}

static void do_standby_state(void)
{
	uint8_t buttons = buttons_get_value();

	if (buttons > 0U)
	{
		if (buttons == TOP_BUTTON)
		{
			enter_setup_state();
		}
		else if (buttons == BOTTOM_BUTTON && settings_get_any_alarm_enabled())
		{
			earliest_next_alarm_time = 0UL;
			app_state = WATCHING;
			show_watching_display();
			wind_alarm_count = 0U;
			depth_alarm_count = 0U;
			sog_alarm_count = 0U;
			distance_alarm_count = 0U;
			heading_alarm_count = 0U;

			latitude_ref = get_latitude_dual_source_data();
			longitude_ref = get_longitude_dual_source_data();
			heading_ref = seatalk_heading_magnetic_retrieve();
			pressure_ref = get_pressure_data();
		}
	}
	else
	{
		if (check_data_availability() != ALL_DATA_AVAILABLE)
		{
			app_state = WAITING_FOR_DATA;
			show_waiting_for_data_display();
		}
	}
}

static void show_setup_display(void)
{
	lcd_clear();

	switch (setup_state)
	{
		case SETUP_STATE_MAIN:
			lcd_puts(0U, 0U, main_menu_items[current_main_menu_position]);
			if (current_main_menu_position == MAIN_MENU_BACKLIGHT || current_main_menu_position == MAIN_MENU_REARM_TIME)
			{
				lcd_puts(1U, 12U, "INCR");
			}
			else if (current_main_menu_position == MAIN_MENU_GPS_SOURCE)
			{
				lcd_puts(1U, 10U, "TOGGLE");
			}
			else
			{
				lcd_puts(1U, 10U, "SELECT");
			}

			if (current_main_menu_position == MAIN_MENU_COUNT - 1U)
			{
				lcd_puts(0U, 12U, "EXIT");
			}
			else
			{
				lcd_puts(0U, 12U, "NEXT");
			}

			switch (current_main_menu_position)
			{
				case MAIN_MENU_DEPTH:
					if (settings_get_depth_enabled())
					{
						(void)strcpy(text_line, "ON");
					}
					else
					{
						(void)strcpy(text_line, "OFF");
					}
					break;

				case MAIN_MENU_SOG:
					if (settings_get_sog_enabled())
					{
						(void)strcpy(text_line, "ON");
					}
					else
					{
						(void)strcpy(text_line, "OFF");
					}
					break;

				case MAIN_MENU_WINDSPEED:
					if (settings_get_wind_enabled())
					{
						(void)strcpy(text_line, "ON");
					}
					else
					{
						(void)strcpy(text_line, "OFF");
					}
					break;

				case MAIN_MENU_HEADING:
					if (settings_get_heading_enabled())
					{
						(void)strcpy(text_line, "ON");
					}
					else
					{
						(void)strcpy(text_line, "OFF");
					}
					break;

				case MAIN_MENU_POSITION:
					if (settings_get_distance_enabled())
					{
						(void)strcpy(text_line, "ON");
					}
					else
					{
						(void)strcpy(text_line, "OFF");
					}
					break;

				case MAIN_MENU_PRESSURE:
					if (settings_get_pressure_change_enabled())
					{
						(void)strcpy(text_line, "ON");
					}
					else
					{
						(void)strcpy(text_line, "OFF");
					}
					break;

				case MAIN_MENU_BACKLIGHT:
					(void)snprintf(text_line, sizeof(text_line), "%hhu", settings_get_backlight());
					break;

				case MAIN_MENU_REARM_TIME:
					(void)snprintf(text_line, sizeof(text_line), "%um", settings_get_rearm_time());
					break;

				case MAIN_MENU_GPS_SOURCE:
					if (settings_get_gps_from_seatalk())
					{
						(void)snprintf(text_line, sizeof(text_line), "SEATALK");
					}
					else
					{
						(void)snprintf(text_line, sizeof(text_line), "MOUSE");
					}
					break;

				default:
					text_line[0] = '\0';
					break;
			}
			lcd_puts(1U, 0U, text_line);
			break;

		case SETUP_STATE_DEPTH:
			if (settings_get_depth_enabled())
			{
				lcd_puts(0U, 0U, "ON");
			}
			else
			{
				lcd_puts(0U, 0U, "OFF");
			}
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 10U, "TOGGLE");
			break;

		case SETUP_STATE_DEPTH_VALUE:
			(void)snprintf(text_line, sizeof(text_line), "%u.%u m", settings_get_min_depth() / 10U, settings_get_min_depth() % 10U);
			lcd_puts(0U, 0U, text_line);
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 12U, "INCR");
			break;

		case SETUP_STATE_SOG:
			if (settings_get_sog_enabled())
			{
				lcd_puts(0U, 0U, "ON");
			}
			else
			{
				lcd_puts(0U, 0U, "OFF");
			}
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 10U, "TOGGLE");
			break;

		case SETUP_STATE_SOG_VALUE:
			(void)snprintf(text_line, sizeof(text_line), "%u.%u Kts", settings_get_max_sog() / 10U, settings_get_max_sog() % 10U);
			lcd_puts(0U, 0U, text_line);
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 12U, "INCR");
			break;

		case SETUP_STATE_WIND_SPEED:
			if (settings_get_wind_enabled())
			{
				lcd_puts(0U, 0U, "ON");
			}
			else
			{
				lcd_puts(0U, 0U, "OFF");
			}
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 10U, "TOGGLE");
			break;

		case SETUP_STATE_WIND_SPEED_VALUE:
			(void)snprintf(text_line, sizeof(text_line), "%u Kts", settings_get_max_wind());
			lcd_puts(0U, 0U, text_line);
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 12U, "INCR");
			break;

		case SETUP_STATE_HEADING:
			if (settings_get_heading_enabled())
			{
				lcd_puts(0U, 0U, "ON");
			}
			else
			{
				lcd_puts(0, 0, "OFF");
			}
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 10U, "TOGGLE");
			break;

		case SETUP_STATE_HEADING_VALUE:
			(void)snprintf(text_line, sizeof(text_line), "%u" LCD_DEGREE_CHARACTER, settings_get_max_heading());
			lcd_puts(0U, 0U, text_line);
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 12U, "INCR");
			break;

		case SETUP_STATE_POSITION:
			if (settings_get_distance_enabled())
			{
				lcd_puts(0U, 0U, "ON");
			}
			else
			{
				lcd_puts(0U, 0U, "OFF");
			}
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 10U, "TOGGLE");
			break;

		case SETUP_STATE_POSITION_VALUE:
			(void)snprintf(text_line, sizeof(text_line), "%u m", settings_get_max_distance());
			lcd_puts(0U, 0U, text_line);
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 12U, "INCR");
			break;

		case SETUP_STATE_PRESSURE_CHANGE:
			if (settings_get_pressure_change_enabled())
			{
				lcd_puts(0U, 0U, "ON");
			}
			else
			{
				lcd_puts(0U, 0U, "OFF");
			}
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 10U, "TOGGLE");
			break;

		case SETUP_STATE_PRESSURE_CHANGE_VALUE:
			(void)snprintf(text_line, sizeof(text_line), "%u mb", settings_get_max_pressure_change());
			lcd_puts(0U, 0U, text_line);
			lcd_puts(0U, 12U, "NEXT");
			lcd_puts(1U, 12U, "INCR");
			break;

		case SETUP_STATE_ALARM_TEST:
			lcd_puts(0U, 0U, "ALARM TEST  EXIT");
			lcd_puts(1U, 0U, SOFTWARE_VERSION_NUMBER);
			break;
	}
}

static void do_setup_state(void)
{
	uint8_t buttons = buttons_get_value();

	if (buttons > 0U)
	{
		switch (setup_state)
		{
		case SETUP_STATE_MAIN:
			switch (buttons)
			{
			case TOP_BUTTON:
				current_main_menu_position++;
				if (current_main_menu_position == MAIN_MENU_COUNT)
				{
					buzzer_off();
					backlight_set(settings_get_backlight());
					app_state = WAITING_FOR_DATA;
					show_waiting_for_data_display();
					settings_save();
					return;
				}
				break;

			case BOTTOM_BUTTON:
				switch (current_main_menu_position)
				{
					case MAIN_MENU_DEPTH:
						setup_state = SETUP_STATE_DEPTH;
						break;

					case MAIN_MENU_SOG:
						setup_state = SETUP_STATE_SOG;
						break;

					case MAIN_MENU_WINDSPEED:
						setup_state = SETUP_STATE_WIND_SPEED;
						break;

					case MAIN_MENU_HEADING:
						setup_state = SETUP_STATE_HEADING;
						break;

					case MAIN_MENU_POSITION:
						setup_state = SETUP_STATE_POSITION;
						break;

					case MAIN_MENU_PRESSURE:
						setup_state = SETUP_STATE_PRESSURE_CHANGE;
						break;

					case MAIN_MENU_BACKLIGHT:
						settings_set_backlight(settings_get_backlight() + 1U);
						if (settings_get_backlight() > BACKLIGHT_MAX)
						{
							settings_set_backlight(0U);
						}
						backlight_set(settings_get_backlight());
						break;

					case MAIN_MENU_REARM_TIME:
						settings_set_rearm_time(settings_get_rearm_time() + 1U);
						if (settings_get_rearm_time() > REARM_TIME_MAX)
						{
							settings_set_rearm_time(0U);
						}
						break;

					case MAIN_MENU_GPS_SOURCE:
						settings_set_gps_from_seatalk(!settings_get_gps_from_seatalk());
						if (settings_get_gps_from_seatalk())
						{
							mouse_off();
						}
						else
						{
							mouse_on();
						}
						clear_all_data_reception_times();
						break;

					case MAIN_MENU_ALARM_TEST:
						setup_state = SETUP_STATE_ALARM_TEST;
						buzzer_on();
						backlight_set(BACKLIGHT_MAX);
						break;
				}
				break;

			case BOTTOM_BUTTON_LONG:
				if (current_main_menu_position == MAIN_MENU_REARM_TIME)
				{
					settings_set_rearm_time(settings_get_rearm_time() + 10U);
					if (settings_get_rearm_time() > REARM_TIME_MAX)
					{
						settings_set_rearm_time(0U);
					}
				}
				break;
			}
			break;

		case SETUP_STATE_DEPTH:
			if (buttons == TOP_BUTTON)
			{
				if (settings_get_depth_enabled())
				{
					setup_state = SETUP_STATE_DEPTH_VALUE;
				}
				else
				{
					setup_state = SETUP_STATE_MAIN;
					current_main_menu_position++;
				}
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_depth_enabled(!settings_get_depth_enabled());
			}
			break;

		case SETUP_STATE_DEPTH_VALUE:
			if (buttons == TOP_BUTTON)
			{
				setup_state = SETUP_STATE_MAIN;
				current_main_menu_position++;
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_min_depth(settings_get_min_depth() + 1U);
				if (settings_get_min_depth() > 150U)
				{
					settings_set_min_depth(0U);
				}
			}
			else if (buttons == BOTTOM_BUTTON_LONG)
			{
				settings_set_min_depth(settings_get_min_depth() + 10U);
				if (settings_get_min_depth() > 150U)
				{
					settings_set_min_depth(0U);
				}
			}
			break;

		case SETUP_STATE_SOG:
			if (buttons == TOP_BUTTON)
			{
				if (settings_get_sog_enabled())
				{
					setup_state = SETUP_STATE_SOG_VALUE;
				}
				else
				{
					setup_state = SETUP_STATE_MAIN;
					current_main_menu_position++;
				}
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_sog_enabled(!settings_get_sog_enabled());
			}
			break;

		case SETUP_STATE_SOG_VALUE:
			if (buttons == TOP_BUTTON)
			{
				setup_state = SETUP_STATE_MAIN;
				current_main_menu_position++;
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_max_sog(settings_get_max_sog() + 1U);
				if (settings_get_max_sog() > 60U)
				{
					settings_set_max_sog(0U);
				}
			}
			else if (buttons == BOTTOM_BUTTON_LONG)
			{
				settings_set_max_sog(settings_get_max_sog() + 10U);
				if (settings_get_max_sog() > 60U)
				{
					settings_set_max_sog(0U);
				}
			}
			break;

		case SETUP_STATE_WIND_SPEED:
			if (buttons == TOP_BUTTON)
			{
				if (settings_get_wind_enabled())
				{
					setup_state = SETUP_STATE_WIND_SPEED_VALUE;
				}
				else
				{
					setup_state = SETUP_STATE_MAIN;
					current_main_menu_position++;
				}
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_wind_enabled(!settings_get_wind_enabled());
			}
			break;

		case SETUP_STATE_WIND_SPEED_VALUE:
			if (buttons == TOP_BUTTON)
			{
				setup_state = SETUP_STATE_MAIN;
				current_main_menu_position++;
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_max_wind(settings_get_max_wind() + 1U);
				if (settings_get_max_wind() > 50U)
				{
					settings_set_max_wind(0U);
				}
			}
			else if (buttons == BOTTOM_BUTTON_LONG)
			{
				settings_set_max_wind(settings_get_max_wind() + 10U);
				if (settings_get_max_wind() > 50U)
				{
					settings_set_max_wind(0U);
				}
			}
			break;

		case SETUP_STATE_HEADING:
			if (buttons == TOP_BUTTON)
			{
				if (settings_get_heading_enabled())
				{
					setup_state = SETUP_STATE_HEADING_VALUE;
				}
				else
				{
					setup_state = SETUP_STATE_MAIN;
					current_main_menu_position++;
				}
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_heading_enabled(!settings_get_heading_enabled());
			}
			break;

		case SETUP_STATE_HEADING_VALUE:
			if (buttons==TOP_BUTTON)
			{
				setup_state = SETUP_STATE_MAIN;
				current_main_menu_position++;
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_max_heading(settings_get_max_heading() + 1U);
				if (settings_get_max_heading() > 179U)
				{
					settings_set_max_heading(0U);
				}
			}
			else if (buttons == BOTTOM_BUTTON_LONG)
			{
				settings_set_max_heading(settings_get_max_heading() + 10U);
				if (settings_get_max_heading() > 179U)
				{
					settings_set_max_heading(0U);
				}
			}
			break;

		case SETUP_STATE_POSITION:
			if (buttons == TOP_BUTTON)
			{
				if (settings_get_distance_enabled())
				{
					setup_state = SETUP_STATE_POSITION_VALUE;
				}
				else
				{
					setup_state = SETUP_STATE_MAIN;
					current_main_menu_position++;
				}
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_distance_enabled(!settings_get_distance_enabled());
			}
			break;

		case SETUP_STATE_POSITION_VALUE:
			if (buttons == TOP_BUTTON)
			{
				setup_state = SETUP_STATE_MAIN;
				current_main_menu_position++;
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_max_distance(settings_get_max_distance() + 1U);
				if (settings_get_max_distance() > 120U)
				{
					settings_set_max_distance(10U);
				}
			}
			else if (buttons == BOTTOM_BUTTON_LONG)
			{
				settings_set_max_distance(settings_get_max_distance() + 10U);
				if (settings_get_max_distance() > 120U)
				{
					settings_set_max_distance(10U);
				}
			}
			break;

		case SETUP_STATE_PRESSURE_CHANGE:
			if (buttons == TOP_BUTTON)
			{
				if (settings_get_pressure_change_enabled())
				{
					setup_state = SETUP_STATE_PRESSURE_CHANGE_VALUE;
				}
				else
				{
					setup_state = SETUP_STATE_MAIN;
					current_main_menu_position++;
				}
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_pressure_change_enabled(!settings_get_pressure_change_enabled());
			}
			break;

		case SETUP_STATE_PRESSURE_CHANGE_VALUE:
			if (buttons == TOP_BUTTON)
			{
				setup_state = SETUP_STATE_MAIN;
				current_main_menu_position++;
			}
			else if (buttons == BOTTOM_BUTTON)
			{
				settings_set_max_pressure_change(settings_get_max_pressure_change() + 1U);
				if (settings_get_max_pressure_change() > 25U)
				{
					settings_set_max_pressure_change(1U);
				}
			}
			else if (buttons == BOTTOM_BUTTON_LONG)
			{
				settings_set_max_pressure_change(settings_get_max_pressure_change() + 10U);
				if (settings_get_max_pressure_change() > 25U)
				{
					settings_set_max_pressure_change(1U);
				}
			}
			break;

		case SETUP_STATE_ALARM_TEST:
			if (buttons == TOP_BUTTON)
			{
				show_waiting_for_data_display();
				buzzer_off();
				app_state = WAITING_FOR_DATA;
				backlight_set(settings_get_backlight());
				settings_save();
				return;
			}
			break;
		}

		show_setup_display();
	}
}
