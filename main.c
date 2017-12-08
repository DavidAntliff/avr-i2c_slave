/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef F_CPU
#  error "F_CPU undefined"
#endif

#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()
#include <stdbool.h>

#include "usitwislave/usitwislave.h"

#define I2C_ADDRESS 0x44

#define WRITE_LED PA0
#define READ_LED  PA1

// single-byte register storage
static uint8_t data = 0;

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                          uint8_t *output_buffer_length, uint8_t *output_buffer);

static void idle_callback(void);


static void blink_led(volatile uint8_t * port, uint8_t pin, int n)
{
    for (int i = 0; i < n; ++i)
    {
        *port |= 1 << pin;
        _delay_ms(150);
        *port &= ~(1 << pin);
        _delay_ms(150);
    }
}

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                   uint8_t *output_buffer_length, uint8_t *output_buffer)
{
    // This is a simple handler for a single I2C register interface.
    // The first byte of a write is stored, and retrieved on a read.

    if (input_buffer_length > 0)
    {
        // write
        data = input_buffer[0];
        blink_led(&PORTA, WRITE_LED, data);
    }
    else
    {
   	    // read
        blink_led(&PORTA, READ_LED, data);
    }

    // always populate the output buffer, even for a read, otherwise a subsequent read will return an empty buffer.
    *output_buffer_length = 1;
    output_buffer[0] = data;
}

static void idle_callback(void)
{
    //blink_led(&PORTA, READ_LED, 1);
}

int main(void)
{
    // initialize port A
    DDRA = (1 << PA0) | (1 << PA1);
    PORTA = 0b00000000;

    // start the slave loop
    usi_twi_slave(I2C_ADDRESS, false /*use_sleep*/, data_callback, idle_callback);
}

