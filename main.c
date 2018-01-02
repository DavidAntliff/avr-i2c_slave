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

/*
 * Function
 * ========
 *
 * The ATtiny84A controls two SSRs (solid-state relays) connected to hydro pumps:
 *   - Circulation pump (CP) - a smaller pump used to cycle water through the system
 *   - Purge pump (PP) - a larger pump used intermittently to force air out of the system
 *
 * In addition, a piezoelectric buzzer is also controlled.
 *
 * Communication between the ATtiny84 and the main controller (ESP32) is via the I2C bus.
 * The ATtiny84A implements an I2C slave, and responds to commands sent by the I2C master (ESP32).
 *
 * Due to limitations of the I2C Slave library, the results of a write must be queried by
 * a separate read transaction. It is not possible to combine a write and a read in a single
 * transaction (e.g. SMBus Read/Write Byte protocol) with this library. Use SMBus Send/Receive Byte
 * protocols instead.
 *
 * Four switches are used to provide a user-accessible interface to the system:
 *
 *   - CP Mode (auto/man)
 *   - CP Man (on/off)
 *   - PP Mode (auto/man)
 *   - PP Man (on/off)
 *
 * Therefore each SSR has two corresponding switches. When the Mode switch for an SSR is in the AUTO
 * position, the ESP32 can request a state change (i.e. turn the SSR on or off) and the ATtiny84A
 * will actuate the change. However if the Mode switch is in the MAN (manual) position, the ATtiny84A
 * will ignore ESP32 requests and use the position of the MAN switch to control the SSR. Therefore the
 * MAN switch provides a manual override of the ESP32 when required.
 *
 * Additionally, a piezoelectric buzzer can be turned on or off.
 *
 * I2C Registers
 * =============
 *
 * In general, the internal ADDR register is written first to specify the control/status
 * register for following read/write operations.
 *
 *   ADDR register: specifies the address of the target register for subsequent read and write operations.
 *     It is a single byte value. It defaults to 0x00 at power on.
 *
 *   CONTROL register (0x00): provides I2C master control of SSR and buzzer states:
 *     bit 7: reserved
 *     bit 6: reserved
 *     bit 5: reserved
 *     bit 4: set piezoelectric buzzer state (1 = on, 0 = off)
 *     bit 3: reserved
 *     bit 2: reserved
 *     bit 1: set SSR2 state in AUTO mode (1 = on, 0 = off)
 *     bit 0: set SSR1 state in AUTO mode (1 = on, 0 = off)
 *
 *   STATUS register (0x01): provides I2C master monitoring of switch and SSR states
 *     bit 7: read sw4 (PP Man) state (1 = on, 0 = off) on PA3
 *     bit 6: read sw3 (PP Mode) state (1 = manual, 0 = auto) on PA2
 *     bit 5: read sw2 (CP Man) state (1 = on, 0 = off) on PA1
 *     bit 4: read sw1 (CP Mode) state (1 = manual, 0 = auto) on PA0
 *     bit 3: reserved
 *     bit 2: reserved
 *     bit 1: read actual SSR2 state (1 = on, 0 = off)
 *     bit 0: read actual SSR1 state (1 = on, 0 = off)
 *
 * To set the value of a register:
 *   - Send two bytes (register address, register value)
 *
 * To read the value of a register:
 *   - Send one byte (register address),
 *   - Receive read 1 byte
 *
 *
 * ATtiny84 Configuration
 * ======================
 *
 * Clock output on PORTB2 must be disabled (clear Low Fuse bit 6).
 */


#ifndef F_CPU
#  error "F_CPU undefined"
#endif

#include <avr/io.h>
#include <util/delay.h>                // for _delay_ms()
#include <stdbool.h>

#include "usitwislave/usitwislave.h"

#define I2C_ADDRESS 0x44

#define LED_BANK  PORTB
#define LED_PORT  PB1

#define REGISTER_CONTROL 0x00
#define REGISTER_STATUS  0x01
#define NUM_REGISTERS    2

// Control Register
#define REGISTER_CONTROL_SSR1   (1 << 0)
#define REGISTER_CONTROL_SSR2   (1 << 1)
#define REGISTER_CONTROL_BUZZER (1 << 4)

#define REGISTER_CONTROL_ON     1
#define REGISTER_CONTROL_OFF    0

// Status Register
#define REGISTER_STATUS_SSR1    (1 << 0)
#define REGISTER_STATUS_SSR2    (1 << 1)
#define REGISTER_STATUS_SW1     (1 << 4)    // CP Mode
#define REGISTER_STATUS_SW2     (1 << 5)    // CP Man
#define REGISTER_STATUS_SW3     (1 << 6)    // PP Mode
#define REGISTER_STATUS_SW4     (1 << 7)    // PP Man

#define REGISTER_STATUS_CP_MODE REGISTER_STATUS_SW1
#define REGISTER_STATUS_CP_MAN  REGISTER_STATUS_SW2
#define REGISTER_STATUS_PP_MODE REGISTER_STATUS_SW3
#define REGISTER_STATUS_PP_MAN  REGISTER_STATUS_SW4

#define REGISTER_STATUS_MODE_AUTO   0
#define REGISTER_STATUS_MODE_MANUAL 1
#define REGISTER_STATUS_ON          1
#define REGISTER_STATUS_OFF         0

static uint8_t register_addr = 255;
static uint8_t registers[NUM_REGISTERS] = { 0x00, 0x00 };

// SSR "actual" states
static uint8_t ssr1 = 0;
static uint8_t ssr2 = 0;

#define SSR1_BANK PORTB
#define SSR1_PORT PB0
#define SSR2_BANK PORTB
#define SSR2_PORT PB2

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                          uint8_t *output_buffer_length, uint8_t *output_buffer);

static void idle_callback(void);


static void blink_led(volatile uint8_t * port, uint8_t pin, int n)
{
    for (int i = 0; i < n; ++i)
    {
        *port |= 1 << pin;
        _delay_ms(100);
        *port &= ~(1 << pin);
        _delay_ms(100);
    }
}

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                          uint8_t *output_buffer_length, uint8_t *output_buffer)
{
    *output_buffer_length = 1;
    output_buffer[0] = 0;

    // support SMBus Send Byte (1 or 2 incoming data bytes) or Receive Byte (0 incoming data byte)
    switch (input_buffer_length)
    {
        case 0:  // read currently addressed register
            //blink_led(&LED_BANK, LED_PORT, 1);
            if (register_addr < NUM_REGISTERS)
            {
                *output_buffer_length = 1;
                output_buffer[0] = registers[register_addr];
            }
            break;

        case 1:  // write before read - set ADDR register only
            //blink_led(&LED_BANK, LED_PORT, 2);
            register_addr = input_buffer[0];
            if (register_addr < NUM_REGISTERS)
            {
                *output_buffer_length = 1;
                output_buffer[0] = registers[register_addr];
            }
            break;

        case 2:  // write - set command register and addressed register
            //blink_led(&LED_BANK, LED_PORT, 3);
            register_addr = input_buffer[0];
            if (register_addr < NUM_REGISTERS)
            {
                // only write to writable registers:
                if (register_addr == REGISTER_CONTROL)
                {
                    registers[register_addr] = input_buffer[1];
                }
                // as a convenience, allow for immediate read-back of selected register
                *output_buffer_length = 1;
                output_buffer[0] = registers[register_addr];
            }
            break;

        default:
            // ignore
            //blink_led(&LED_BANK, LED_PORT, 4);
            break;
    }
}

static void read_switches(void)
{

}

static void calculate_ssr_state(uint8_t * ssr, uint8_t mode)
{

}

static void update_ssr_output(uint8_t * ssr, volatile uint8_t * port, uint8_t pin)
{

}

static void update_status_register(uint8_t * ssr, uint8_t status)
{

}

static void idle_callback(void)
{
    //blink_led(&LED_BANK, LED_PORT, 1);

    // read switch states from first 4 bits of PINA
    uint8_t switches = PINA & 0x0f;
    registers[REGISTER_STATUS] = switches << 4;

    // manual switches are inverted
    registers[REGISTER_STATUS] ^= REGISTER_STATUS_CP_MAN;
    registers[REGISTER_STATUS] ^= REGISTER_STATUS_PP_MAN;

    // Calculate SSR actual states
    if ((registers[REGISTER_STATUS] & REGISTER_STATUS_CP_MODE) == REGISTER_STATUS_MODE_AUTO)
    {
        // use the requested control value
        ssr1 = (registers[REGISTER_CONTROL] & REGISTER_CONTROL_SSR1) ? 1 : 0;
    }
    else
    {
        // use the manual switch value
        ssr1 = (registers[REGISTER_STATUS] & REGISTER_STATUS_CP_MAN) ? 1 : 0;
    }

    if ((registers[REGISTER_STATUS] & REGISTER_STATUS_PP_MODE) == REGISTER_STATUS_MODE_AUTO)
    {
        // use the requested control value
        ssr2 = (registers[REGISTER_CONTROL] & REGISTER_CONTROL_SSR2) ? 1 : 0;
    }
    else
    {
        // use the manual switch value
        ssr2 = (registers[REGISTER_STATUS] & REGISTER_STATUS_PP_MAN) ? 1 : 0;
    }

    // Update the actual status of the SSR and drive the output
    if (ssr1)
    {
        registers[REGISTER_STATUS] |= REGISTER_STATUS_SSR1;

        // drive low to turn SSR on
        SSR1_BANK &= ~(1 << SSR1_PORT);
    }
    else
    {
        registers[REGISTER_STATUS] &= ~REGISTER_STATUS_SSR1;
        SSR1_BANK |= 1 << SSR1_PORT;
    }

    if (ssr2)
    {
        registers[REGISTER_STATUS] |= REGISTER_STATUS_SSR2;

        // drive low to turn SSR on
        SSR2_BANK &= ~(1 << SSR2_PORT);
    }
    else
    {
        registers[REGISTER_STATUS] &= ~REGISTER_STATUS_SSR2;
        SSR2_BANK |= 1 << SSR2_PORT;
    }
}

int main(void)
{
    // initialize PA0-3 as inputs, PA7 as output
    DDRA = (0 << PA0) | (0 << PA1) | (0 << PA2) | (0 << PA3) | (1 << PA7);

    // activate pullups on PA0-3
    PORTA = (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);

    // initialize PB0, PB1, PB2 as outputs
    DDRB = (1 << PB0) | (1 << PB1) | (1 << PB2);

    // initialise PB0 and PB2 high
    PORTB = (1 << PB0) | (0 << PB1) | (1 << PB2);

    // start the slave loop
    usi_twi_slave(I2C_ADDRESS, false /*use_sleep*/, data_callback, idle_callback);
}

