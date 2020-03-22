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

#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#define RECEIVE_BUFFER_SIZE_1	250U
#define TRANSMIT_BUFFER_SIZE_1	250U
#define RECEIVE_BUFFER_SIZE_2	250U
#define RECEIVE_BUFFER_SIZE_3	150U
#define TRANSMIT_BUFFER_SIZE_3	150U

void serial_init(uint32_t baud_rate_1, uint32_t baud_rate_2, uint32_t baud_rate_3);

uint16_t serial_1_read_data(uint16_t buffer_length, uint8_t *data);
uint16_t serial_1_send_data(uint16_t length, const uint8_t *data);
uint16_t serial_2_read_data(uint16_t buffer_length, uint8_t *data);
uint16_t serial_3_read_data(uint16_t buffer_length, uint8_t *data);
uint16_t serial_3_send_data(uint16_t length, const uint8_t *data);

#endif