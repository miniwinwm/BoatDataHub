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

#include <math.h>
#include "mathematics.h"	

/**
 * Calculate the great circle distance in metres between 2 points specified in degrees lat/long.
 * Uses the Haversine formulae which assumes the earth is spherical. Error can be 0.5% maximum.
 */
float distance_between_points(float lat1, float long1, float lat2, float long2)
{
	float half_dlat;
	float half_dlong;
	float a;
	float c;

	lat1 /= DEGREES_TO_RADS;
	long1 /= DEGREES_TO_RADS;
	lat2 /= DEGREES_TO_RADS;
	long2 /= DEGREES_TO_RADS;
	half_dlat = (lat2 - lat1) / 2.0f;
	half_dlong = (long2 - long1) / 2.0f;
	a = sinf(half_dlat) * sinf(half_dlat) + sinf(half_dlong) * sinf(half_dlong) * cosf(lat1) * cosf(lat2);
	c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

	return 6371000.0f * c;
}
