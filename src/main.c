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
#include <math.h>
#include "stm32f10x.h"
#include "timer.h"
#include "nmea.h"
#include "seatalk.h"
#include "pressure_sensor.h"
#include "delay.h"
#include "autopilot_remote.h"
#include "buzzer.h"
#include "backlight.h"
#include "buttons.h"
#include "lcd.h"
#include "watcher.h"
#include "settings.h"
#include "mouse.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "serial.h"
#include "spi.h"
#include "wmm.h"
#include "nrf24l01.h"
#include "flash.h"

#define PORT_BLUETOOTH							0
#define PORT_AIS								1
#define PORT_GPS								2
#define MAIN_TASK_SW_TIMER_COUNT				3
#define SW_TIMER_10_S							0
#define SW_TIMER_25_MS							1
#define SW_TIMER_1_S							2
#define MAIN_TASK_STACK_SIZE					220U
#define LCD_TASK_STACK_SIZE						120U
#define PRESSURE_SENSOR_TASK_STACK_SIZE			120U
#define AUTOPILOT_REMOTE_TASK_STACK_SIZE		120U
#define BAUD_RATE_BLUETOOTH_DATA				57600UL
#define BAUD_RATE_BLUETOOTH_AT					38400UL
#define BAUD_RATE_AIS 							38400UL
#define BAUD_RATE_GPS 							4800UL
#define TWENTY_FIVE_MS_PERIOD_MS				25UL
#define ONE_SECOND_PERIOD_MS					1000UL
#define TEN_SECOND_PERIOD_MS					10000UL
#define ONE_MINUTE_PERIOD_MS					60000UL
#define DEPTH_MAX_DATA_AGE_MS					4000UL
#define HEADING_MAGNETIC_MAX_DATA_AGE_MS		4000UL
#define HEADING_TRUE_MAX_DATA_AGE_MS			4000UL
#define BOAT_SPEED_MAX_DATA_AGE_MS				4000UL
#define GMT_MAX_DATA_AGE_MS						12000UL
#define DATE_MAX_DATA_AGE_MS					12000UL
#define COG_MAX_DATA_AGE_MS						4000UL
#define SOG_MAX_DATA_AGE_MS						4000UL
#define LATITUDE_MAX_DATA_AGE_MS				4000UL
#define LONGITUDE_MAX_DATA_AGE_MS				4000UL
#define TEMPERATURE_MAX_DATA_AGE_MS				4000UL
#define TRIP_MAX_DATA_AGE_MS					8000UL
#define LOG_MAX_DATA_AGE_MS						8000UL
#define TRUE_WIND_ANGLE_MAX_DATA_AGE_MS			4000UL
#define TRUE_WIND_SPEED_MAX_DATA_AGE_MS			4000UL
#define WIND_DIRECTION_MAGNETIC_MAX_DATA_AGE_MS	4000UL
#define WIND_DIRECTION_TRUE_MAX_DATA_AGE_MS		4000UL
#define APPARENT_WIND_ANGLE_MAX_DATA_AGE_MS		4000UL
#define APPARENT_WIND_SPEED_MAX_DATA_AGE_MS		4000UL
#define GPS_QUALITY_INDICATOR_MAX_DATA_AGE_MS	4000UL
#define SATELLITES_IN_USE_MAX_DATA_AGE_MS		4000UL
#define HDOP_MAX_DATA_AGE_MS					4000UL
#define ALTITUDE_MAX_DATA_AGE_MS				4000UL
#define GEOIDAL_SEPARATION_MAX_DATA_AGE_MS		4000UL
#define PRESSURE_MAX_DATA_AGE_MS				30000UL
#define ARRIVAL_CIRCLE_ENTERED_DATA_MAG_AGE_MS	4000UL
#define PERPENDICULAR_PASSED_MAX_DATA_AGE_MS	4000UL
#define WAYPOINT_ID_MAX_DATA_AGE_MS				4000UL
#define CROSS_TRACK_ERROR_MAX_DATA_AGE_MS		4000UL
#define DIRECTION_TO_STEER_MAX_DATA_AGE_MS		4000UL
#define BEARING_TO_DESTINATION_MAX_DATA_AGE_MS	4000UL
#define DISTANCE_TO_DESTINATION_MAX_DATA_AGE_MS	4000UL
#define WMM_CALCULATION_MAX_DATA_AGE			(60UL * 60UL * 1000UL)

typedef struct
{
	uint32_t depth_received_time;
	uint32_t boat_speed_received_time;
	uint32_t temperature_received_time;
	uint32_t apparent_wind_angle_received_time;
	uint32_t apparent_wind_speed_received_time;
	uint32_t heading_magnetic_received_time;
	uint32_t trip_received_time;
	uint32_t log_received_time;
	uint32_t true_wind_speed_received_time;
	uint32_t true_wind_angle_received_time;
	uint32_t speed_over_ground_received_time;
	uint32_t course_over_ground_received_time;
	uint32_t latitude_received_time;
	uint32_t longitude_received_time;
	uint32_t gmt_received_time;
	uint32_t date_received_time;
	uint32_t gps_quality_indicator_received_time;
	uint32_t satellites_in_use_received_time;
	uint32_t hdop_received_time;
	uint32_t altitude_received_time;
	uint32_t geoidal_separation_received_time;
	uint32_t pressure_received_time;
	uint32_t wind_direction_magnetic_received_time;
	uint32_t cross_track_error_received_time;
	uint32_t direction_to_steer_received_time;
	uint32_t range_to_destination_received_time;
	uint32_t bearing_to_destination_true_received_time;
	uint32_t bearing_to_destination_magnetic_received_time;
	uint32_t arrival_circle_entered_received_time;
	uint32_t perpendicular_passed_received_time;
	uint32_t waypoint_id_received_time;
	uint32_t wmm_calculation_time;
} boat_data_reception_time_t;

static void callback_seatalk_message(uint8_t message_type);
static void vTimerCallback1s(TimerHandle_t xTimer);
static void vTimerCallback25ms(TimerHandle_t xTimer);
static void vTimerCallback10s(TimerHandle_t xTimer);
static void RMC_receive_callback(char *data);
static void GGA_receive_callback(char *data);
static void RMC_transmit_callback(void);
static void GGA_transmit_callback(void);
static void XDR_transmit_callback(void);
static void MDA_transmit_callback(void);
static void DPT_transmit_callback(void);
static void VHW_transmit_callback(void);
static void VLW_transmit_callback(void);
static void MWV_transmit_callback(void);
static void MTW_transmit_callback(void);
static void MWD_transmit_callback(void);
static void VDM_receive_callback(char *data);
static void VDM_transmit_callback(void);
static void RMB_receive_callback(char *data);
static void APB_receive_callback(char *data);
static void HDT_transmit_callback(void);
static void HDM_transmit_callback(void);

// main task
static void main_task(void *parameters);
static StackType_t main_stack[MAIN_TASK_STACK_SIZE];
static StaticTask_t main_task_buffer;
static TimerHandle_t xTimers[MAIN_TASK_SW_TIMER_COUNT];
static StaticTimer_t xTimerBuffers[MAIN_TASK_SW_TIMER_COUNT];

// pressure sensor task
static StackType_t pressure_sensor_stack[PRESSURE_SENSOR_TASK_STACK_SIZE];
static StaticTask_t pressure_sensor_task_handle;
static StaticQueue_t pressure_sensor_queue;
static uint8_t pressure_sensor_queue_buffer[sizeof(float)];
static QueueHandle_t pressure_sensor_queue_handle;

// lcd task
static StackType_t lcd_stack[LCD_TASK_STACK_SIZE];
static StaticTask_t lcd_task_buffer;
static StaticQueue_t lcd_queue;
static uint8_t lcd_queue_buffer[sizeof(lcd_packet_t) * LCD_QUEUE_CAPACITY];
static QueueHandle_t lcd_queue_handle;

// autopilot remote task
static StackType_t autopilot_remote_stack[AUTOPILOT_REMOTE_TASK_STACK_SIZE];
static StaticTask_t autopilot_remote_task_buffer;
static StaticQueue_t autopilot_remote_queue;
static uint8_t autopilot_remote_queue_buffer[sizeof(float)];
static QueueHandle_t autopilot_remote_queue_handle;

// these come from either nmea or seatalk, sent to nmea (all) or seatalk (some)
static volatile float altitude_dual_source_data;
static volatile float hdop_dual_source_data;
static volatile float speed_over_ground_dual_source_data;
static volatile float geoidal_separation_dual_source_data;
static volatile seatalk_time_t gmt_dual_source_data;
static volatile seatalk_date_t date_dual_source_data;
static volatile float latitude_dual_source_data;
static volatile float longitude_dual_source_data;
static volatile int16_t course_over_ground_dual_source_data;
static volatile uint8_t gps_quality_indicator_dual_source_data;
static volatile uint8_t satellites_in_use_dual_source_data;

// these come from nmea, used for calculations
static int16_t bearing_to_destination_true_data;

// comes from wmm
static float variation_wmm_data;

// these come from nmea, sent to seatalk
static char direction_to_steer_data;
static float range_to_destination_data;
static int16_t bearing_to_destination_magnetic_data;
static bool arrival_circle_entered_data;
static bool perpendicular_passed_data;
static float cross_track_error_data;
static char waypoint_id_data[NMEA_APB_WAYPOINT_NAME_MAX_LENGTH + 1];
static float pressure_data;

static volatile boat_data_reception_time_t boat_data_reception_time;
static nmea_message_data_VDM_t nmea_message_data_VDM;
static nmea_message_data_XDR_t nmea_message_data_XDR;
static nmea_message_data_MDA_t nmea_message_data_MDA;
static nmea_message_data_DPT_t nmea_message_data_DPT;
static nmea_message_data_VHW_t nmea_message_data_VHW;
static nmea_message_data_MTW_t nmea_message_data_MTW;
static nmea_message_data_VLW_t nmea_message_data_VLW;
static nmea_message_data_MWV_t nmea_message_data_MWV;
static nmea_message_data_GGA_t nmea_message_data_GGA;
static nmea_message_data_MWD_t nmea_message_data_MWD;
static nmea_message_data_RMC_t nmea_message_data_RMC;
static nmea_message_data_RMB_t nmea_message_data_RMB;
static nmea_message_data_APB_t nmea_message_data_APB;
static nmea_message_data_HDM_t nmea_message_data_HDM;
static nmea_message_data_HDT_t nmea_message_data_HDT;

void clear_all_data_reception_times(void)
{
	(void)memset((void *)&boat_data_reception_time, 0, sizeof(boat_data_reception_time_t));
}

uint32_t get_sog_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.speed_over_ground_received_time) / 1000UL;
}

uint32_t get_latitiude_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.latitude_received_time) / 1000UL;
}

uint32_t get_longitiude_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.longitude_received_time) / 1000UL;
}

uint32_t get_depth_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.depth_received_time) / 1000UL;
}

uint32_t get_aws_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.apparent_wind_speed_received_time) / 1000UL;
}

uint32_t get_heading_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.heading_magnetic_received_time) / 1000UL;
}

uint32_t get_pressure_data_age_s(void)
{
	return (timer_get_time_ms() - boat_data_reception_time.pressure_received_time) / 1000UL;
}

float get_longitude_dual_source_data(void)
{
	return latitude_dual_source_data;
}

float get_latitude_dual_source_data(void)
{
	return longitude_dual_source_data;
}

float get_sog_dual_source_data(void)
{
	return speed_over_ground_dual_source_data;
}

float get_pressure_data(void)
{
	return pressure_data;
}

/***********
* AIS PORT *
* *********/

/* VDM - receive from VHF */
static const nmea_receive_message_details_t nmea_receive_message_details_VDM = {nmea_message_VDM, PORT_AIS, VDM_receive_callback};

static void VDM_receive_callback(char *data)
{
	if (data)
	{
		if (nmea_decode_VDM(data, &nmea_message_data_VDM) == nmea_error_none)
		{
			nmea_transmit_message_now(PORT_BLUETOOTH, nmea_message_VDM);
		}
	}
}

/***********
* GPS PORT *
* *********/

/* GGA receive from GPS mouse */
static const nmea_receive_message_details_t nmea_receive_message_details_GGA = { nmea_message_GGA, PORT_GPS, GGA_receive_callback };

static void GGA_receive_callback(char *data)
{
	uint32_t time_ms = timer_get_time_ms();

	// only do GGA decoding if gps from mouse
    if (!settings_get_gps_from_seatalk())
    {
		if (data)
		{
			if (nmea_decode_GGA(data, &nmea_message_data_GGA) == nmea_error_none)
			{
				if (nmea_message_data_GGA.data_available & NMEA_GGA_QUALITY_INDICATOR_PRESENT)
				{
					gps_quality_indicator_dual_source_data = nmea_message_data_GGA.quality_indicator;
					boat_data_reception_time.gps_quality_indicator_received_time = time_ms;
				}

				if (nmea_message_data_GGA.data_available & NMEA_GGA_SATELLITES_IN_USE_PRESENT)
				{
					satellites_in_use_dual_source_data = nmea_message_data_GGA.satellites_in_use;
					boat_data_reception_time.satellites_in_use_received_time = time_ms;
				}

				if (nmea_message_data_GGA.data_available & NMEA_GGA_HDOP_PRESENT)
				{
					hdop_dual_source_data = nmea_message_data_GGA.HDOP;
					boat_data_reception_time.hdop_received_time = time_ms;
				}

				if (nmea_message_data_GGA.data_available & NMEA_GGA_ALTITUDE_PRESENT)
				{
					altitude_dual_source_data = nmea_message_data_GGA.altitude;
					boat_data_reception_time.altitude_received_time = time_ms;
				}

				if (nmea_message_data_GGA.data_available & NMEA_GGA_GEIODAL_SEPARATION_PRESENT)
				{
					geoidal_separation_dual_source_data = nmea_message_data_GGA.geoidal_separation;
					boat_data_reception_time.geoidal_separation_received_time = time_ms;
				}
			}
		}
    }
}

/* RMC receive from GPS mouse */
static const nmea_receive_message_details_t nmea_receive_message_details_RMC_GPS = { nmea_message_RMC, PORT_GPS, RMC_receive_callback };

static void RMC_receive_callback(char *data)
{
	float int_part;
	float frac_part;
	uint32_t time_ms = timer_get_time_ms();

	if (data)
	{
		if (nmea_decode_RMC(data, &nmea_message_data_RMC) == nmea_error_none)
		{
			if (nmea_message_data_RMC.status == 'A')
			{
				if (nmea_message_data_RMC.data_available & NMEA_RMC_UTC_PRESENT)
				{
					gmt_dual_source_data.hour = nmea_message_data_RMC.utc.hours;
					gmt_dual_source_data.minute = nmea_message_data_RMC.utc.minutes;
					gmt_dual_source_data.second = nmea_message_data_RMC.utc.seconds;
					boat_data_reception_time.gmt_received_time = time_ms;
				}

				if (nmea_message_data_RMC.data_available & NMEA_RMC_DATE_PRESENT)
				{
					date_dual_source_data.year = nmea_message_data_RMC.date.year - 2000U;
					date_dual_source_data.month = nmea_message_data_RMC.date.month;
					date_dual_source_data.date = nmea_message_data_RMC.date.date;
					boat_data_reception_time.date_received_time = time_ms;
				}

				if (nmea_message_data_RMC.data_available & NMEA_RMC_SOG_PRESENT)
				{
					speed_over_ground_dual_source_data = nmea_message_data_RMC.SOG;
					boat_data_reception_time.speed_over_ground_received_time = time_ms;
				}

				if (nmea_message_data_RMC.data_available & NMEA_RMC_COG_PRESENT)
				{
					course_over_ground_dual_source_data = nmea_message_data_RMC.COG;
					boat_data_reception_time.course_over_ground_received_time = time_ms;
				}

				if (nmea_message_data_RMC.data_available & NMEA_RMC_LATITUDE_PRESENT)
				{
					frac_part = modff(nmea_message_data_RMC.latitude / 100.0f, &int_part);
					latitude_dual_source_data = int_part;
					latitude_dual_source_data += frac_part / 0.6f;
					boat_data_reception_time.latitude_received_time = time_ms;
				}

				if (nmea_message_data_RMC.data_available & NMEA_RMC_LONGITUDE_PRESENT)
				{
					frac_part = modff(nmea_message_data_RMC.longitude / 100.0f, &int_part);
					longitude_dual_source_data = int_part;
					longitude_dual_source_data += frac_part / 0.6f;
					boat_data_reception_time.longitude_received_time = time_ms;
				}
			}
		}
	}
}

/* RMC transmit to VHF */
static const transmit_message_details_t nmea_transmit_message_details_RMC_GPS = { nmea_message_RMC, PORT_GPS, 1000UL, RMC_transmit_callback, &nmea_message_data_RMC, (nmea_encoder_function_t)nmea_encode_RMC };

static void RMC_transmit_callback(void)
{
	float int_part;
	float frac_part;

	nmea_message_data_RMC.status = 'A';
	nmea_message_data_RMC.utc.seconds = gmt_dual_source_data.second;
	nmea_message_data_RMC.utc.minutes = gmt_dual_source_data.minute;
	nmea_message_data_RMC.utc.hours = gmt_dual_source_data.hour;
	nmea_message_data_RMC.date.year = date_dual_source_data.year + 2000U;
	nmea_message_data_RMC.date.month = date_dual_source_data.month;
	nmea_message_data_RMC.date.date = date_dual_source_data.date;
	nmea_message_data_RMC.SOG = speed_over_ground_dual_source_data;
	nmea_message_data_RMC.COG = course_over_ground_dual_source_data;
	frac_part = modff(latitude_dual_source_data, &int_part);
	nmea_message_data_RMC.latitude = int_part * 100.0f;
	nmea_message_data_RMC.latitude += frac_part * 60.0f;
	frac_part = modff(longitude_dual_source_data, &int_part);
	nmea_message_data_RMC.longitude = int_part * 100.0f;
	nmea_message_data_RMC.longitude += frac_part * 60.0f;
	nmea_message_data_RMC.mode = 'A';
	if (variation_wmm_data < 0.0f)
	{
		nmea_message_data_RMC.magnetic_variation = - variation_wmm_data;
		nmea_message_data_RMC.magnetic_variation_direction = 'W';
	}
	else
	{
		nmea_message_data_RMC.magnetic_variation = variation_wmm_data;
		nmea_message_data_RMC.magnetic_variation_direction = 'E';
	}
	nmea_message_data_RMC.navigation_status = 'S';
	nmea_message_data_RMC.data_available = NMEA_RMC_UTC_PRESENT | NMEA_RMC_STATUS_PRESENT |
			NMEA_RMC_SOG_PRESENT | NMEA_RMC_COG_PRESENT | NMEA_RMC_DATE_PRESENT | NMEA_RMC_LATITUDE_PRESENT |
			NMEA_RMC_LONGITUDE_PRESENT | NMEA_RMC_MODE_PRESENT | NMEA_RMC_NAV_STATUS_PRESENT | NMEA_RMC_MAG_VARIATION_PRESENT | NMEA_RMC_MAG_DIRECTION_PRESENT;
}

/***********
* AIS PORT *
* *********/

/* RMB receive from OpenCPN */
static const nmea_receive_message_details_t nmea_receive_message_details_RMB = { nmea_message_RMB, PORT_BLUETOOTH, RMB_receive_callback };

static void RMB_receive_callback(char *data)
{
	uint32_t time_ms = timer_get_time_ms();

	if (data)
	{
		if (nmea_decode_RMB(data, &nmea_message_data_RMB) == nmea_error_none)
		{
			if (nmea_message_data_RMB.data_available & NMEA_RMB_STATUS_PRESENT && nmea_message_data_RMB.status == 'A')
			{
				if (nmea_message_data_RMB.data_available & NMEA_RMB_MODE_PRESENT && (nmea_message_data_RMB.mode == 'A' || nmea_message_data_RMB.mode == 'D'))
				{
					if (nmea_message_data_RMB.data_available & NMEA_RMB_CROSS_TRACK_ERROR_PRESENT)
					{
						// not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_DIR_TO_STEER_PRESENT)
					{
						// not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_ORIG_WAYPOINT_ID_PRESENT)
					{
						// not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_DEST_WAYPOINT_ID_PRESENT)
					{
						// using RMB's waypoint id max length as it is less than APB's and both messages write to the same data buffer
						(void)strncpy(waypoint_id_data, nmea_message_data_RMB.destination_waypoint_name, NMEA_RMB_WAYPOINT_NAME_MAX_LENGTH);
						waypoint_id_data[NMEA_RMB_WAYPOINT_NAME_MAX_LENGTH] = '\0';	// safe use of strncpy
						boat_data_reception_time.waypoint_id_received_time = time_ms;
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_LATITUDE_PRESENT)
					{
						// not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_LONGITUDE_PRESENT)
					{
						// not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_RANGE_TO_DEST_PRESENT)
					{
						range_to_destination_data = nmea_message_data_RMB.range_to_destination;
						boat_data_reception_time.range_to_destination_received_time = time_ms;
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_BEARING_TRUE_PRESENT)
					{
						//not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_VELOCITY_PRESENT)
					{
						// not used
					}

					if (nmea_message_data_RMB.data_available & NMEA_RMB_ARRIVAL_STATUS_PRESENT)
					{
						// ambiguous, use values from APB message instead
					}
				}
			}
		}
	}
}

/* APB receive from OpenCPN */
static const nmea_receive_message_details_t nmea_receive_message_details_APB = { nmea_message_APB, PORT_BLUETOOTH, APB_receive_callback };

static void APB_receive_callback(char *data)
{
	uint32_t time_ms = timer_get_time_ms();

	if (!data)
	{
		return;
	}

	if (nmea_decode_APB(data, &nmea_message_data_APB) != nmea_error_none)
	{
		return;
	}

	if ((nmea_message_data_APB.data_available & NMEA_APB_STATUS1_PRESENT) == 0UL || nmea_message_data_APB.status1 != 'A')
	{
		return;
	}

	if ((nmea_message_data_APB.data_available & NMEA_APB_STATUS2_PRESENT) == 0UL || nmea_message_data_APB.status2 != 'A')
	{
		return;
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_MODE_PRESENT && nmea_message_data_APB.mode != 'A')
	{
		return;
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_CROSS_TRACK_ERROR_PRESENT)
	{
		cross_track_error_data = nmea_message_data_APB.cross_track_error;
		boat_data_reception_time.cross_track_error_received_time = time_ms;
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_DIRECTION_TO_STEER_PRESENT)
	{
		if (nmea_message_data_APB.direction_to_steer == 'L' || nmea_message_data_APB.direction_to_steer == 'R')
		{
			direction_to_steer_data = nmea_message_data_APB.direction_to_steer;
			boat_data_reception_time.direction_to_steer_received_time = time_ms;
		}
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_ARRIVAL_CIRCLE_ENTERED_PRESENT)
	{
		if (nmea_message_data_APB.arrival_circle_entered == 'A')
		{
			arrival_circle_entered_data = true;
			boat_data_reception_time.arrival_circle_entered_received_time = time_ms;
		}
		else if (nmea_message_data_APB.arrival_circle_entered == 'V')
		{
			arrival_circle_entered_data = false;
			boat_data_reception_time.arrival_circle_entered_received_time = time_ms;
		}
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_PERPENDICULAR_PASSED_PRESENT)
	{
		if (nmea_message_data_APB.perpendicular_passed == 'A')
		{
			perpendicular_passed_data = true;
			boat_data_reception_time.perpendicular_passed_received_time = time_ms;
		}
		else if (nmea_message_data_APB.perpendicular_passed == 'V')
		{
			perpendicular_passed_data = false;
			boat_data_reception_time.perpendicular_passed_received_time = time_ms;
		}
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_BEARING_ORIG_TO_DEST_PRESENT &&
			nmea_message_data_APB.data_available & NMEA_APB_BEARING_MAG_OR_TRUE_PRESENT)
	{
		// not used
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_DEST_WAYPOINT_ID_PRESENT)
	{
		(void)strncpy(waypoint_id_data, nmea_message_data_APB.waypoint_name, NMEA_APB_WAYPOINT_NAME_MAX_LENGTH);
		waypoint_id_data[NMEA_APB_WAYPOINT_NAME_MAX_LENGTH] = '\0';	// safe use of strncpy
		boat_data_reception_time.waypoint_id_received_time = time_ms;
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_BEARING_POS_TO_DEST_PRESENT &&
			nmea_message_data_APB.data_available & NMEA_APB_BEARING_POS_TO_DEST_MAG_OR_TRUE_PRESENT)
	{
		if (nmea_message_data_APB.bearing_position_to_destination_magnetic_or_true == 'T')
		{
			bearing_to_destination_true_data = (int16_t)nmea_message_data_APB.bearing_position_to_destination;
			boat_data_reception_time.bearing_to_destination_true_received_time = time_ms;
		}
		else if (nmea_message_data_APB.bearing_position_to_destination_magnetic_or_true == 'M')
		{
			bearing_to_destination_magnetic_data = (int16_t)nmea_message_data_APB.bearing_position_to_destination;
			boat_data_reception_time.bearing_to_destination_magnetic_received_time = time_ms;
		}
	}

	if (nmea_message_data_APB.data_available & NMEA_APB_HEADING_TO_STEER_PRESENT &&
			nmea_message_data_APB.data_available & NMEA_APB_HEADING_TO_STEER_MAG_OR_TRUE_PRESENT)
	{
		// not used
	}
}

/* VDM - transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_VDM = { nmea_message_VDM,	PORT_BLUETOOTH, 0UL, VDM_transmit_callback, &nmea_message_data_VDM, (nmea_encoder_function_t)nmea_encode_VDM };

static void VDM_transmit_callback(void)
{
}

/* XDR transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_XDR = { nmea_message_XDR,	PORT_BLUETOOTH, 10000UL, XDR_transmit_callback,	&nmea_message_data_XDR,	(nmea_encoder_function_t)nmea_encode_XDR };

static void XDR_transmit_callback(void)
{
	nmea_message_data_XDR.measurements[0].decimal_places = 4U;
	nmea_message_data_XDR.measurements[0].transducer_type = 'P';
	nmea_message_data_XDR.measurements[0].transducer_id[0] = '\0';
	nmea_message_data_XDR.measurements[0].units = 'B';
	nmea_message_data_XDR.measurements[0].measurement = pressure_data / 1000.0f;
	nmea_message_data_XDR.data_available = NMEA_XDR_MEASUREMENT_1_PRESENT;
}

/* MDA transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_MDA = { nmea_message_MDA,	PORT_BLUETOOTH, 10000UL, MDA_transmit_callback,	&nmea_message_data_MDA,	(nmea_encoder_function_t)nmea_encode_MDA };

static void MDA_transmit_callback(void)
{
	nmea_message_data_MDA.pressure_bars = pressure_data / 1000.0f;
	nmea_message_data_MDA.data_available = NMEA_MDA_PRESSURE_BARS_PRESENT;
}

/* DPT transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_DPT = { nmea_message_DPT,	PORT_BLUETOOTH, 500UL, DPT_transmit_callback, &nmea_message_data_DPT, (nmea_encoder_function_t)nmea_encode_DPT };

static void DPT_transmit_callback(void)
{
	nmea_message_data_DPT.depth = seatalk_depth_data_retrieve();
	nmea_message_data_DPT.data_available = NMEA_DPT_DEPTH_PRESENT;
}

/* MWD transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_MWD = { nmea_message_MWD,	PORT_BLUETOOTH, 2000UL, MWD_transmit_callback, &nmea_message_data_MWD, (nmea_encoder_function_t)nmea_encode_MWD };

static void MWD_transmit_callback(void)
{
	nmea_message_data_MWD.wind_speed_knots = seatalk_true_wind_speed_retrieve();
	nmea_message_data_MWD.data_available = NMEA_MWD_WIND_SPEED_KTS_PRESENT;

	if (timer_get_time_ms() - boat_data_reception_time.wind_direction_magnetic_received_time < WIND_DIRECTION_MAGNETIC_MAX_DATA_AGE_MS)
	{
		nmea_message_data_MWD.data_available |= NMEA_MWD_WIND_DIRECTION_MAG_PRESENT;
		nmea_message_data_MWD.wind_direction_magnetic = seatalk_wind_direction_magnetic_retrieve();

		nmea_message_data_MWD.data_available |= NMEA_MWD_WIND_DIRECTION_TRUE_PRESENT;
		nmea_message_data_MWD.wind_direction_true = nmea_message_data_MWD.wind_direction_magnetic + variation_wmm_data;
	}
}

/* VHW transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_VHW = { nmea_message_VHW,	PORT_BLUETOOTH, 1000UL, VHW_transmit_callback, &nmea_message_data_VHW, (nmea_encoder_function_t)nmea_encode_VHW };

static void VHW_transmit_callback(void)
{
	nmea_message_data_VHW.water_speed_knots = seatalk_boat_speed_data_retrieve();
	nmea_message_data_VHW.data_available = NMEA_VHW_WATER_SPEED_KTS_PRESENT;
}

/* MTW transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_MTW = { nmea_message_MTW, PORT_BLUETOOTH, 2000UL, MTW_transmit_callback, &nmea_message_data_MTW, (nmea_encoder_function_t)nmea_encode_MTW };

static void MTW_transmit_callback(void)
{
	nmea_message_data_MTW.water_temperature = seatalk_temperature_data_retrieve();
	nmea_message_data_MTW.data_available = NMEA_MTW_WATER_TEMPERATURE_PRESENT;
}

/* VLW transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_VLW = { nmea_message_VLW, PORT_BLUETOOTH, 1000UL, VLW_transmit_callback, &nmea_message_data_VLW, (nmea_encoder_function_t)nmea_encode_VLW };

static void VLW_transmit_callback(void)
{
	uint32_t time_ms = timer_get_time_ms();
	nmea_message_data_VLW.data_available = 0UL;

	if (time_ms - boat_data_reception_time.trip_received_time < TRIP_MAX_DATA_AGE_MS)
	{
		nmea_message_data_VLW.trip_water_distance = seatalk_trip_data_retrieve();
		nmea_message_data_VLW.data_available |= NMEA_VLW_TRIP_WATER_DISTANCE_PRESENT;
	}

	if (time_ms - boat_data_reception_time.log_received_time < LOG_MAX_DATA_AGE_MS)
	{
		nmea_message_data_VLW.total_water_distance = seatalk_log_data_retrieve();
		nmea_message_data_VLW.data_available |= NMEA_VLW_TOTAL_WATER_DISTANCE_PRESENT;
	}
}

/* MWV transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_MWV = { nmea_message_MWV, PORT_BLUETOOTH, 1000UL, MWV_transmit_callback, &nmea_message_data_MWV, (nmea_encoder_function_t)nmea_encode_MWV };

static void MWV_transmit_callback(void)
{
	uint32_t time_ms = timer_get_time_ms();
	static bool message_type_toggle = false;

	nmea_message_data_MWV.data_available = NMEA_MWV_REFERENCE_PRESENT | NMEA_MWV_WIND_SPEED_UNITS_PRESENT;
	nmea_message_data_MWV.wind_speed_units = 'N';

	if (message_type_toggle)
	{
		nmea_message_data_MWV.reference = 'T';

		if (time_ms - boat_data_reception_time.true_wind_angle_received_time < TRUE_WIND_ANGLE_MAX_DATA_AGE_MS)
		{
			nmea_message_data_MWV.wind_angle = seatalk_true_wind_angle_retrieve();
			nmea_message_data_MWV.status = 'A';
			nmea_message_data_MWV.data_available |= (NMEA_MWV_WIND_ANGLE_PRESENT | NMEA_MWV_STATUS_PRESENT);
		}

		if (time_ms - boat_data_reception_time.true_wind_speed_received_time < TRUE_WIND_SPEED_MAX_DATA_AGE_MS)
		{
			nmea_message_data_MWV.wind_speed = seatalk_true_wind_speed_retrieve();
			nmea_message_data_MWV.status = 'A';
			nmea_message_data_MWV.data_available |= (NMEA_MWV_WIND_SPEED_PRESENT | NMEA_MWV_STATUS_PRESENT);
		}
	}
	else
	{
		nmea_message_data_MWV.reference = 'R';

		if (time_ms - boat_data_reception_time.apparent_wind_angle_received_time < APPARENT_WIND_ANGLE_MAX_DATA_AGE_MS)
		{
			nmea_message_data_MWV.wind_angle = seatalk_apparent_wind_angle_retrieve();
			nmea_message_data_MWV.status = 'A';
			nmea_message_data_MWV.data_available |= (NMEA_MWV_WIND_ANGLE_PRESENT | NMEA_MWV_STATUS_PRESENT);
		}

		if (time_ms - boat_data_reception_time.apparent_wind_speed_received_time < APPARENT_WIND_SPEED_MAX_DATA_AGE_MS)
		{
			nmea_message_data_MWV.wind_speed = seatalk_apparent_wind_speed_retrieve();
			nmea_message_data_MWV.status = 'A';
			nmea_message_data_MWV.data_available |= (NMEA_MWV_WIND_SPEED_PRESENT | NMEA_MWV_STATUS_PRESENT);
		}
	}

	message_type_toggle = !message_type_toggle;
}

/* HDM transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_HDM = { nmea_message_HDM, PORT_BLUETOOTH, 1000UL, HDM_transmit_callback, &nmea_message_data_HDM, (nmea_encoder_function_t)nmea_encode_HDM };

static void HDM_transmit_callback(void)
{
	nmea_message_data_HDM.magnetic_heading = seatalk_heading_magnetic_retrieve();
	nmea_message_data_HDM.data_available = NMEA_HDM_MAG_HEADING_PRESENT;
}

/* HDT transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_HDT = { nmea_message_HDT, PORT_BLUETOOTH, 1000UL, HDT_transmit_callback, &nmea_message_data_HDT, (nmea_encoder_function_t)nmea_encode_HDT };

static void HDT_transmit_callback(void)
{
	nmea_message_data_HDT.true_heading = (float)seatalk_heading_magnetic_retrieve() + variation_wmm_data;
	nmea_message_data_HDT.data_available = NMEA_HDT_TRUE_HEADING_PRESENT;
}

/* GGA transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_GGA = { nmea_message_GGA, PORT_BLUETOOTH, 2000UL, GGA_transmit_callback, &nmea_message_data_GGA, (nmea_encoder_function_t)nmea_encode_GGA };

static void GGA_transmit_callback(void)
{
	nmea_message_data_GGA.quality_indicator = gps_quality_indicator_dual_source_data;
	nmea_message_data_GGA.satellites_in_use = satellites_in_use_dual_source_data;
	nmea_message_data_GGA.HDOP = hdop_dual_source_data;
	nmea_message_data_GGA.altitude = altitude_dual_source_data;
	nmea_message_data_GGA.geoidal_separation = geoidal_separation_dual_source_data;
	nmea_message_data_GGA.data_available = NMEA_GGA_QUALITY_INDICATOR_PRESENT | NMEA_GGA_SATELLITES_IN_USE_PRESENT |
			NMEA_GGA_HDOP_PRESENT | NMEA_GGA_ALTITUDE_PRESENT |NMEA_GGA_GEIODAL_SEPARATION_PRESENT;
}

/* RMC transmit to OpenCPN */
static const transmit_message_details_t nmea_transmit_message_details_RMC_bluetooth = { nmea_message_RMC, PORT_BLUETOOTH, 1000UL, RMC_transmit_callback, &nmea_message_data_RMC, (nmea_encoder_function_t)nmea_encode_RMC };

// using earlier defined RMC transmit

static void callback_seatalk_message(uint8_t message_type)
{
	uint32_t time_ms = timer_get_time_ms();
	//seatalk_date_t date;

	switch(message_type)
	{
	case SEATALK_DEPTH:
		boat_data_reception_time.depth_received_time = time_ms;
		break;

	case SEATALK_BOATSPEED:
		boat_data_reception_time.boat_speed_received_time = time_ms;
		break;

	case SEATALK_APPARENT_WIND_ANGLE:
		boat_data_reception_time.apparent_wind_angle_received_time = time_ms;
		break;

	case SEATALK_APPARENT_WIND_SPEED:
		boat_data_reception_time.apparent_wind_speed_received_time = time_ms;
		break;

	case SEATALK_GPS_INFO:
	    if (settings_get_gps_from_seatalk())
	    {
	    	boat_data_reception_time.altitude_received_time = time_ms;
	    	altitude_dual_source_data = seatalk_altitude_retrieve();
	    	boat_data_reception_time.geoidal_separation_received_time = time_ms;
	    	geoidal_separation_dual_source_data = seatalk_geoidal_separation_retrieve();
	    }
		break;

	case SEATALK_HDOP:
	    if (settings_get_gps_from_seatalk())
	    {
	    	boat_data_reception_time.hdop_received_time = time_ms;
	    	hdop_dual_source_data = (float)seatalk_hdop_retrieve();
	    }
		break;

	case SEATALK_NUMBER_OF_SATS:
	    if (settings_get_gps_from_seatalk())
	    {
	    	satellites_in_use_dual_source_data = seatalk_number_of_satellites_retrieve();
	    	boat_data_reception_time.satellites_in_use_received_time = time_ms;
	    }
		break;

	case SEATALK_SIGNAL_QUALITY:
	    if (settings_get_gps_from_seatalk())
	    {
	    	boat_data_reception_time.gps_quality_indicator_received_time = time_ms;
	    	gps_quality_indicator_dual_source_data = seatalk_signal_quality_retrieve();
	    }
		break;

	case SEATALK_SOG:
	    if (settings_get_gps_from_seatalk())
	    {
			speed_over_ground_dual_source_data = seatalk_speed_over_ground_data_retrieve();
			boat_data_reception_time.speed_over_ground_received_time = time_ms;
	    }
		break;

	case SEATALK_COG:
	    if (settings_get_gps_from_seatalk())
	    {
			course_over_ground_dual_source_data = seatalk_boat_speed_data_retrieve();
			boat_data_reception_time.course_over_ground_received_time = time_ms;
	    }
		break;

	case SEATALK_HEADING_MAGNETIC_1:
#ifdef GEORGE_CORRECTOR
		{
			autopilot_command_t autopilot_command;

			if (seatalk_autopilot_state_retrieve() == APS_STANDBY)
			{
				autopilot_command = seatalk_autopilot_command_retrieve();
				if (autopilot_command == AUTOPILOT_COMMAND_AUTO || autopilot_command == AUTOPILOT_COMMAND_VANE || autopilot_command == AUTOPILOT_COMMAND_TRACK)
				{
					seatalk_autopilot_send(autopilot_command);
				}
			}
		}
#endif
		break;

	case SEATALK_HEADING_MAGNETIC_2:
		boat_data_reception_time.heading_magnetic_received_time = time_ms;
		break;

	case SEATALK_TRIPLOG:
		boat_data_reception_time.trip_received_time = time_ms;
		boat_data_reception_time.log_received_time = time_ms;
		break;

	case SEATALK_TRUE_WIND_ANGLE_SPEED:
		boat_data_reception_time.true_wind_speed_received_time = time_ms;
		boat_data_reception_time.true_wind_angle_received_time = time_ms;
		break;

	case SEATALK_LATITUDE:
	    if (settings_get_gps_from_seatalk())
	    {
	    	latitude_dual_source_data = (float)seatalk_latitude_degrees_retrieve() + seatalk_latitude_minutes_retrieve() / 60.0f;
	    	boat_data_reception_time.latitude_received_time = time_ms;
	    }
		break;

	case SEATALK_LONGITUDE:
	    if (settings_get_gps_from_seatalk())
	    {
	    	longitude_dual_source_data = (float)seatalk_longitude_degrees_retrieve() + seatalk_longitude_minutes_retrieve() / 60.0f;
	    	boat_data_reception_time.longitude_received_time = time_ms;
	    }
		break;

	case SEATALK_GMT:
	    if (settings_get_gps_from_seatalk())
	    {
	    	gmt_dual_source_data = seatalk_time_retrieve();
	    	boat_data_reception_time.gmt_received_time = time_ms;
	    }
		break;

	case SEATALK_DATE:
	    if (settings_get_gps_from_seatalk())
	    {
	    	date_dual_source_data = seatalk_date_retrieve();
	    	boat_data_reception_time.date_received_time = time_ms;
	    }
		break;

	case SEATALK_TEMPERATURE:
		boat_data_reception_time.temperature_received_time = time_ms;
		break;

	case SEATALK_MAGNETIC_WIND_DIRECTION:
		boat_data_reception_time.wind_direction_magnetic_received_time = time_ms;
		break;

	default:
		break;
	}
}

int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

   	// configure watchdog to 8 seconds
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(40000UL);
    IWDG_ReloadCounter();
    IWDG_Enable();

	// all 4 bits of interrupt priority set to pre-emption, sub-priority ignored
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

 	// start all needed peripheral clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI2 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 |
			RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO  |
			RCC_APB2Periph_TIM1 | RCC_APB2Periph_USART1, ENABLE);

	// initialize all gpio to 2 MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	// alternate function push-pull
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9 | GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// output push-pull
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_9 | GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// input floating
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// input pull-up
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// alternate function open-drain
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	delay_init();
	backlight_init();
	backlight_set(BACKLIGHT_MAX);

	buzzer_init();
	lcd_init();
	pressure_sensor_init();
	seatalk_init(callback_seatalk_message);
	spi2_init();
	wmm_init();
	autopilot_remote_init();

	// clear reset cause flag - read on each boot by 2.4 GHz radio board init procedure for autopilot remote
	RCC_ClearFlag();

    // data in via Bluetooth port */
    nmea_enable_receive_message(&nmea_receive_message_details_RMB);
    nmea_enable_receive_message(&nmea_receive_message_details_APB);

    // data in via AIS port */
    nmea_enable_receive_message(&nmea_receive_message_details_VDM);

    // data in via GPS port */
    nmea_enable_receive_message(&nmea_receive_message_details_RMC_GPS);
    nmea_enable_receive_message(&nmea_receive_message_details_GGA);

    // data out via Bluetooth port; all other messages sent to this port are enabled/disabled in 1s timer
    nmea_enable_transmit_message(&nmea_transmit_message_details_VDM);

	lcd_queue_handle = xQueueCreateStatic((UBaseType_t)LCD_QUEUE_CAPACITY, (UBaseType_t)(sizeof(lcd_packet_t)), lcd_queue_buffer, &lcd_queue);
	(void)xTaskCreateStatic(lcd_task,
			"LCD",
			LCD_TASK_STACK_SIZE,
			&lcd_queue_handle,
			(UBaseType_t)2,
			lcd_stack,
			&lcd_task_buffer);

	pressure_sensor_queue_handle = xQueueCreateStatic((UBaseType_t)1, (UBaseType_t)(sizeof(float)), pressure_sensor_queue_buffer, &pressure_sensor_queue);
	(void)xTaskCreateStatic(pressure_sensor_task,
			"PRES",
			PRESSURE_SENSOR_TASK_STACK_SIZE,
			&pressure_sensor_queue_handle,
			(UBaseType_t)1,
			pressure_sensor_stack,
			&pressure_sensor_task_handle);

   	autopilot_remote_queue_handle = xQueueCreateStatic((UBaseType_t)1, (UBaseType_t)(sizeof(autopilot_remote_command_t)), autopilot_remote_queue_buffer, &autopilot_remote_queue);
   	(void)xTaskCreateStatic(autopilot_remote_task,
   			"AUTO",
			AUTOPILOT_REMOTE_TASK_STACK_SIZE,
			&autopilot_remote_queue_handle,
			(UBaseType_t)1,
			autopilot_remote_stack,
			&autopilot_remote_task_buffer);

   	(void)xTaskCreateStatic(main_task, "MAIN", MAIN_TASK_STACK_SIZE, NULL, (UBaseType_t)3, main_stack, &main_task_buffer);

	xTimers[SW_TIMER_10_S] = xTimerCreateStatic(
						"10s",
						(TickType_t)10000,
						pdTRUE,
						(void *)0,
						vTimerCallback10s,
						&(xTimerBuffers[SW_TIMER_10_S]));

	xTimers[SW_TIMER_25_MS] = xTimerCreateStatic(
						"25ms",
						(TickType_t)25,
						pdTRUE,
						(void *)0,
						vTimerCallback25ms,
						&(xTimerBuffers[SW_TIMER_25_MS]));

	xTimers[SW_TIMER_1_S] = xTimerCreateStatic(
						"1s",
						(TickType_t)1000,
						pdTRUE,
						(void *)0,
						vTimerCallback1s,
						&(xTimerBuffers[SW_TIMER_1_S]));

	vTaskStartScheduler();

	while (true)
	{
	}
}

static void vTimerCallback10s(TimerHandle_t xTimer)
{
	float date;
	uint32_t time_ms = timer_get_time_ms();

	(void)xTimer;

    if (xQueueReceive(pressure_sensor_queue_handle, (void *)&pressure_data, (TickType_t)0) == pdTRUE)
    {
    	boat_data_reception_time.pressure_received_time = time_ms;
    }

	if (time_ms - boat_data_reception_time.wmm_calculation_time > WMM_CALCULATION_MAX_DATA_AGE &&
			time_ms - boat_data_reception_time.latitude_received_time < LATITUDE_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.longitude_received_time < LONGITUDE_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.date_received_time < DATE_MAX_DATA_AGE_MS)
	{
		// do a wmm calculation
		date = wmm_get_date(date_dual_source_data.year, date_dual_source_data.month, date_dual_source_data.date);
		E0000(0.0f, latitude_dual_source_data, longitude_dual_source_data, date, &variation_wmm_data);
		boat_data_reception_time.wmm_calculation_time = time_ms;
	}
}

static void vTimerCallback25ms(TimerHandle_t xTimer)
{
	autopilot_remote_command_t autopilot_remote_command;
	autopilot_command_t autopilot_command = AUTOPILOT_COMMAND_UNKNOWN;

	(void)xTimer;

	nmea_process();
	watcher_process();

    if (xQueueReceive(autopilot_remote_queue_handle, (void *)&autopilot_remote_command, (TickType_t)0) == pdTRUE)
    {
    	switch (autopilot_remote_command)
    	{
    	case AUTOPILOT_REMOTE_MINUS_1:
    		autopilot_command = AUTOPILOT_COMMAND_MINUS_1;
    		break;

    	case AUTOPILOT_REMOTE_MINUS_10:
    		autopilot_command = AUTOPILOT_COMMAND_MINUS_10;
    		break;

    	case AUTOPILOT_REMOTE_PLUS_1:
    		autopilot_command = AUTOPILOT_COMMAND_PLUS_1;
    		break;

    	case AUTOPILOT_REMOTE_PLUS_10:
    		autopilot_command = AUTOPILOT_COMMAND_PLUS_10;
    		break;
    	}

		seatalk_autopilot_send(autopilot_command);
    }
}

static void vTimerCallback1s(TimerHandle_t xTimer)
{
	uint32_t time_ms = timer_get_time_ms();

	(void)xTimer;

	// update local time by 1 second (unless it's 23:59:59) if latest received time is more than 1 second old
	if (time_ms - boat_data_reception_time.gmt_received_time > 1000UL)
	{
		if (!(gmt_dual_source_data.hour == 23U && gmt_dual_source_data.minute == 59U && gmt_dual_source_data.second == 59U))
		{
			gmt_dual_source_data.second++;
			if (gmt_dual_source_data.second > 59U)
			{
				gmt_dual_source_data.second = 0U;
				gmt_dual_source_data.minute++;
				if (gmt_dual_source_data.minute > 59U)
				{
					gmt_dual_source_data.hour++;
				}
			}
		}
	}

    // send GPS SOG and COG to seatalk if setting is on for it
    if (!settings_get_gps_from_seatalk())
    {
    	if (time_ms - boat_data_reception_time.speed_over_ground_received_time < SOG_MAX_DATA_AGE_MS)
    	{
    		seatalk_speed_over_ground_send(speed_over_ground_dual_source_data);
    	}

    	if (time_ms - boat_data_reception_time.course_over_ground_received_time < COG_MAX_DATA_AGE_MS)
    	{
    		seatalk_course_over_ground_send(course_over_ground_dual_source_data);
    	}
    }

    // send target waypoint id to seatalk
	if (time_ms - boat_data_reception_time.waypoint_id_received_time < WAYPOINT_ID_MAX_DATA_AGE_MS)
	{
		seatalk_target_waypoint_id_send(waypoint_id_data);

	    // send perpendicular passed info to seatalk
		if (time_ms - boat_data_reception_time.arrival_circle_entered_received_time < ARRIVAL_CIRCLE_ENTERED_DATA_MAG_AGE_MS &&
				time_ms - boat_data_reception_time.perpendicular_passed_received_time < PERPENDICULAR_PASSED_MAX_DATA_AGE_MS)
		{
			seatalk_arrival_info_send(waypoint_id_data, arrival_circle_entered_data, perpendicular_passed_data);
		}
	}

    // send navigate to waypoint info to seatalk
	if (time_ms - boat_data_reception_time.cross_track_error_received_time < CROSS_TRACK_ERROR_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.range_to_destination_received_time < DISTANCE_TO_DESTINATION_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.direction_to_steer_received_time < DIRECTION_TO_STEER_MAX_DATA_AGE_MS)
	{
		int16_t bearing_to_destination = bearing_to_destination_magnetic_data;
		bool send_navigate_to_waypoint_info = false;

		if (time_ms - boat_data_reception_time.bearing_to_destination_magnetic_received_time < BEARING_TO_DESTINATION_MAX_DATA_AGE_MS)
		{
			bearing_to_destination = bearing_to_destination_magnetic_data;
			send_navigate_to_waypoint_info = true;
		}
		else
		{
			if (time_ms - boat_data_reception_time.wmm_calculation_time < WMM_CALCULATION_MAX_DATA_AGE)
			{
				bearing_to_destination = bearing_to_destination_true_data - (int16_t)variation_wmm_data;
				send_navigate_to_waypoint_info = true;
			}
		}

		if (send_navigate_to_waypoint_info)
		{
			seatalk_navigate_to_waypoint_info_send(cross_track_error_data,
					range_to_destination_data,
					bearing_to_destination,
					direction_to_steer_data);
		}
	}

    // switch on or off RMC message transmission here
	if (time_ms - boat_data_reception_time.gmt_received_time < GMT_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.date_received_time < DATE_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.speed_over_ground_received_time < SOG_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.course_over_ground_received_time < COG_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.latitude_received_time < LATITUDE_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.longitude_received_time < LONGITUDE_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_RMC_bluetooth);
		nmea_enable_transmit_message(&nmea_transmit_message_details_RMC_GPS);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_RMC);
		nmea_disable_transmit_message(PORT_GPS, nmea_message_RMC);
	}

    // switch on or off GGA message transmission here
	if (time_ms - boat_data_reception_time.gps_quality_indicator_received_time < GPS_QUALITY_INDICATOR_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.satellites_in_use_received_time < SATELLITES_IN_USE_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.hdop_received_time < HDOP_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.altitude_received_time < ALTITUDE_MAX_DATA_AGE_MS &&
			time_ms - boat_data_reception_time.geoidal_separation_received_time < GEOIDAL_SEPARATION_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_GGA);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_GGA);
	}

    // switch on or off XDR and MDA message transmission here
	if (time_ms - boat_data_reception_time.pressure_received_time < PRESSURE_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_XDR);
		nmea_enable_transmit_message(&nmea_transmit_message_details_MDA);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_XDR);
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_MDA);
	}

    // switch on or off MWD message transmission here
	if (time_ms - boat_data_reception_time.wind_direction_magnetic_received_time < WIND_DIRECTION_MAGNETIC_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_MWD);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_MWD);
	}

    // switch on or off DPT message transmission here
	if (time_ms - boat_data_reception_time.depth_received_time < DEPTH_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_DPT);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_DPT);
	}

    // switch on or off VHW message transmission here
	if (time_ms - boat_data_reception_time.boat_speed_received_time < BOAT_SPEED_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_VHW);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_VHW);
	}

    // switch on or off MTW message transmission here
	if (time_ms - boat_data_reception_time.temperature_received_time < TEMPERATURE_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_MTW);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_MTW);
	}

    // switch on or off VLW message transmission here
	if (time_ms - boat_data_reception_time.trip_received_time < TRIP_MAX_DATA_AGE_MS ||
			time_ms - boat_data_reception_time.log_received_time < LOG_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_VLW);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_VLW);
	}

    // switch on or off MWV message transmission here
	if (time_ms - boat_data_reception_time.true_wind_angle_received_time < TRUE_WIND_ANGLE_MAX_DATA_AGE_MS ||
			time_ms - boat_data_reception_time.true_wind_speed_received_time < TRUE_WIND_SPEED_MAX_DATA_AGE_MS ||
			time_ms - boat_data_reception_time.apparent_wind_angle_received_time < APPARENT_WIND_ANGLE_MAX_DATA_AGE_MS ||
			time_ms - boat_data_reception_time.apparent_wind_speed_received_time < APPARENT_WIND_SPEED_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_MWV);
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_MWV);
	}

    // switch on or off HDM/HDT message transmission here
	if (time_ms - boat_data_reception_time.heading_magnetic_received_time < HEADING_MAGNETIC_MAX_DATA_AGE_MS)
	{
		nmea_enable_transmit_message(&nmea_transmit_message_details_HDM);

		if (time_ms - boat_data_reception_time.wmm_calculation_time < WMM_CALCULATION_MAX_DATA_AGE)
		{
			nmea_enable_transmit_message(&nmea_transmit_message_details_HDT);
		}
		else
		{
			nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_HDT);
		}
	}
	else
	{
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_HDM);
		nmea_disable_transmit_message(PORT_BLUETOOTH, nmea_message_HDT);
	}
}

static void main_task(void *parameters)
{
	uint8_t task_started_count = 0U;

	(void)parameters;

	// init all the reception times to some time a long time ago
	(void)memset((void *)&boat_data_reception_time, 0x7f, sizeof(boat_data_reception_time));

	// wait until all server tasks have started
	do
	{
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		task_started_count++;
	}
	while (task_started_count < 3U);

	// check for button press on startup to init bluetooth module
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 1U)
	{
	    // open serial ports
		serial_init(BAUD_RATE_BLUETOOTH_AT, BAUD_RATE_AIS, BAUD_RATE_GPS);
    	vTaskDelay((TickType_t)1000);
		serial_1_send_data(19U, (const uint8_t *)"AT+UART=57600,0,0\r\n");

		lcd_puts(0U, 2U, "Power off/on");
    	while (true)
    	{
    		IWDG_ReloadCounter();
        	vTaskDelay((TickType_t)10);
    	}
	}

    // open serial ports
	serial_init(BAUD_RATE_BLUETOOTH_DATA, BAUD_RATE_AIS, BAUD_RATE_GPS);

	lcd_puts(0U, 0U, "YAPP ELECTRONICS");
	lcd_puts(1U, 1U, "Boat Data Hub");
	buzzer_on();
	vTaskDelay((TickType_t)500);
	buzzer_off();
	vTaskDelay((TickType_t)1500);
	settings_load();
	backlight_set(settings_get_backlight());
	mouse_init(!settings_get_gps_from_seatalk());
	watcher_init();
    buttons_init();

	(void)xTimerStart(xTimers[SW_TIMER_10_S], (TickType_t)0);
	(void)xTimerStart(xTimers[SW_TIMER_25_MS], (TickType_t)0);
	(void)xTimerStart(xTimers[SW_TIMER_1_S], (TickType_t)0);

    while (true)
    {
        IWDG_ReloadCounter();

    	vTaskDelay((TickType_t)10);

    	// every cycle stuff
    	seatalk_parse_next_message();
    	seatalk_send_next_message();
    }
}

/**
 * FreeRTOS memory allocation for idle task
 *
 * @param ppxIdleTaskTCBBuffer Pointer to pointer of idle task TCB
 * @param ppxIdleTaskStackBuffer Pointer to pointer of idle task stack
 * @param pulIdleTaskStackSize Pointer to idle task stack size
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer,
		uint32_t *pulIdleTaskStackSize)
{
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 * FreeRTOS memory allocation for timer task
 *
 * @param ppxTimerTaskTCBBuffer Pointer to pointer of timer task TCB
 * @param ppxTimerTaskStackBuffer Pointer to pointer of timer task stack
 * @param pulTimerTaskStackSize Pointer to timer task stack size
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer,
		uint32_t *pulTimerTaskStackSize)
{
	static StaticTask_t xTimerTaskTCBBuffer;
	static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

	*ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
	*ppxTimerTaskStackBuffer = &xTimerStack[0];
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
