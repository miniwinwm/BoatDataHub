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

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// this includes code that corrects the ST2000+ problem of switching from track, auto or vane mode into standby with no user interaction
#define GEORGE_CORRECTOR

uint32_t get_sog_data_age_s(void);
uint32_t get_latitiude_data_age_s(void);
uint32_t get_longitiude_data_age_s(void);
uint32_t get_depth_data_age_s(void);
uint32_t get_aws_data_age_s(void);
uint32_t get_heading_data_age_s(void);
uint32_t get_pressure_data_age_s(void);
float get_longitude_dual_source_data(void);
float get_latitude_dual_source_data(void);
float get_sog_dual_source_data(void);
float get_pressure_data(void);
void clear_all_data_reception_times(void);

#endif
