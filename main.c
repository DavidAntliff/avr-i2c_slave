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

static uint8_t data = 0;

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                   uint8_t *output_buffer_length, uint8_t *output_buffer);

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                   uint8_t *output_buffer_length, uint8_t *output_buffer)
{
    if (input_buffer_length > 0)
    {
        // write
        data = input_buffer[0];
        for (int i = 0; i < data; ++i)
        {
			PORTA ^= 0b00000001;
            _delay_ms(150);
			PORTA ^= 0b00000001;
		    _delay_ms(150);
        }
    }
    else
    {
   	    // read
	    PORTA ^= 0b00000010;
	    _delay_ms(150);
	    PORTA ^= 0b00000010;
	    _delay_ms(150);
    }

    *output_buffer_length = 1;
    output_buffer[0] = data;
}

static void idle_callback(void);

static void idle_callback(void)
{
    /* _delay_ms(950); */
    /* PORTA = ~PORTA; */
    /* _delay_ms(50); */
    /* PORTA = ~PORTA; */
}

int main(void)
{
    DDRA = 0x01;                       // initialize port A
    PORTA = 0b00000000;

    /* while(1) */
    /* { */
    /*     // LED on */
    /*     PORTA = 0b00000001;            // PA0 = High = Vcc */
    /*     _delay_ms(500);                // wait 500 milliseconds */

    /*     //LED off */
    /*     PORTA = 0b00000000;            // PA0 = Low = 0v */
    /*     _delay_ms(500);                // wait 500 milliseconds */
    /* } */

    uint8_t slave_address = 0x44;
    bool use_sleep = false;

    usi_twi_slave(slave_address, use_sleep, data_callback, idle_callback);
}

