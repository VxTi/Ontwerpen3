//
// Created by Luca Warmenhoven on 03/02/2023.
//

#define F_CPU       32000000UL
#define UARTF0_BAUD 115200UL

#define __AVR_ATxmega256A3U__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "../../lib/nrf24spiXM2.h"
#include "../../lib/nrf24L01.h"
#include "../../lib/serialF0.h"
#include <stdbool.h>

// This is the index of the current device. The device address is hereby selected as addresses[DEVICE_IDX]
#define DEVICE_IDX 4
#define CENTRAL_DEVICE_IDX 0

#define TICK_SPEED 20                      // Ticks per second
#define TIMER_PRESCALER 1024               // Clock pre-scaler
#define DELTA_T (1.0F / TICK_SPEED)        // Time difference per tick
#define INTERPOLATION_FACTOR (1.0f / 2.0f) // Interpolation factor for changing the PWM PER


#define BUFFER_LENGTH 32 // The maximum size of the char buffers

// ADC Values
#define ADC_MAX     4095
#define ADC_MIN      200
#define ADC_REF_V   1.6f
#define ADC_VIN     3200

// Pin locations
#define LED_PINS_RGB            (PIN3_bm | PIN2_bm | PIN4_bm)
#define LED_PIN_MOTION_SENSOR   PIN1_bm
#define LED_PIN_LAMPS           (PIN5_bm | PIN6_bm | PIN7_bm)
#define LED_PER                     311 // Per value, PER = F_CPU / (PRESCALER * Hz) - 1, 331 = 100Hz
#define MAX_BRIGHTNESS             0.06125f // The max brightness. This is a factor ranging from 0 to 1 inclusively
#define LAMP_SLEEP_TIME               60 // After how many seconds the lamp goes to sleep if there hasn't been any motion

#define clampf(x, a, b) ((x) < (a) ? (a) : (x) > (b) ? (b) : (x)) // Macro for clamping a number between boundaries

#define ADC_TO_MVOLT(res, ref) ((ADC_VIN / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN))

// The addresses of all the nodes.
const char *addresses[] = {"stm_0", "stm_1", "stm_2", "stm_3", "stm_4", "stm_5"};

// Buffers for receiving and sending packets.
volatile uint8_t receive_buffer[BUFFER_LENGTH];
volatile uint8_t transmit_buffer[BUFFER_LENGTH];
volatile bool    timer_triggered = false;
volatile bool    packet_received = false;

// Vector struct. handy.
typedef struct {
    float x, y, z, w;
} vec4;

/**
 * Method for translating a 4d vector to a specified position
 * @param vector The destined vector to store the result in.
 */
static inline void v_translate(vec4 * vector, float x, float y, float z, float w) {
    vector->x = x;
    vector->y = y;
    vector->z = z;
    vector->w = w;
}

// Threshold values.
enum {
    TEMPERATURE_MIN       = 18,
    TEMPERATURE_MAX       = 30,
    HUMIDITY_MIN          = 40,
    HUMIDITY_MAX          = 100,
    PM10_MAX          = 20,       // Max yearly average PM10  value in ug/m3
    PM25_MAX          = 10,       // Max yearly average PM2.5 value in ug/m3
    CO2_RES_MAX = 1300,     // Voltage value retrieved from ADC (Sensor gives too arbitrary values for PPM)
    SENSOR_COUNT = 5,
    COLOR_COUNT = 4
};

// Color codes indicating danger levels, xyzw in rgba
const vec4 colors[] = {
        {.x = 0.0f, .y = 0.0f, .z = 1.0f, .w = 1.0f}, // Blue   0x0000FF
        {.x = 1.0f, .y = 1.0f, .z = 0.0f, .w = 1.0f}, // Yellow 0xFFFF00
        {.x = 1.0f, .y = 0.5f, .z = 0.0f, .w = 1.0f}, // Orange 0xFF8000
        {.x = 1.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f}  // Red    0xFF0000
};

// Predefine the functions for later use.
void configure();
void configure_nrf();
void load_pipes_nrf();
void transmit_nrf(char* buffer, uint16_t bufferSize);
void configure_lights();
void set_rgb(float r, float g, float b);
void set_rgba(float r, float g, float b, float a);

// The main function, clearly
int main(void) {

    configure();
    configure_usartf0(F_CPU, UARTF0_BAUD);
    configure_nrf();
    configure_lights();

    vec4 color, color_lerp;
    v_translate(&color, 0, 0, 0, 0);
    v_translate(&color_lerp, 1, 1, 1, 0);

    uint16_t ticks_passed = 0, seconds_passed = 0, PM10 = 0, PM25 = 0;
    bool lamp_enabled = false;

    uint8_t humidity = 0;
    int8_t temperature = 0;
    uint16_t co2_res = 0;

    while (true) {
        if (timer_triggered) {
            ticks_passed = (ticks_passed + 1) % TICK_SPEED;

            // Color interpolates between 0 and 3 (indices of colors)
            // This follows the formula (H + T) * |Fco2 + Fpm10 + Fpm25| * C
            color_lerp = colors[
                    (uint8_t) clampf(
                            (((float) (humidity - HUMIDITY_MIN) / (float) (HUMIDITY_MAX - HUMIDITY_MIN)) + ((float) (temperature - TEMPERATURE_MIN) / (float)(TEMPERATURE_MAX - TEMPERATURE_MIN)))
                            * roundf((
                    ((float) (co2_res - ADC_MIN) / (float) (CO2_RES_MAX - ADC_MIN)) +
                    ((float) (PM10_MAX) / (float)PM10) +
                    ((float) (PM25_MAX) / (float) PM25)) * COLOR_COUNT), 0, COLOR_COUNT - 1)];


            color_lerp.w = lamp_enabled ? MAX_BRIGHTNESS : 0.0f;

            // Linear interpolation
            v_translate(&color,
                        color.x + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.x - color.x),
                        color.y + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.y - color.y),
                        color.z + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.z - color.z),
                        color.w + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.w - color.w));
            set_rgba(color.x, color.y, color.z, color.w);

            // Check whether a second has passed
            if (!ticks_passed) {
                seconds_passed = (seconds_passed + 1) % LAMP_SLEEP_TIME;
                if (PORTD.IN & LED_PIN_MOTION_SENSOR)
                    lamp_enabled = true;
                else if (lamp_enabled && seconds_passed == LAMP_SLEEP_TIME - 1)
                    lamp_enabled = false;

                PORTD.OUT &= ~LED_PIN_LAMPS;
                if (lamp_enabled)
                    PORTD.OUT |= LED_PIN_LAMPS;
            }
            timer_triggered = false;
        }

        if (packet_received) {

            if (!strncmp((char *) receive_buffer, "co2_res=", 8)) {
                co2_res = atoi((char *) &receive_buffer[8]);
            } else if (!strncmp((char *) receive_buffer, "temp=", 5)) {
                temperature = (int8_t) atoi((char *) &receive_buffer[5]);
            } else if (!strncmp((char *) receive_buffer, "humid=", 6)) {
                humidity = atoi((char *) &receive_buffer[6]);
            } else if (!strncmp((char *) receive_buffer, "PM10=", 5)) {
                PM10 = atoi((char *) &receive_buffer[5]);
            } else if (!strncmp((char *) receive_buffer, "PM25=", 5)) {
                PM25 = atoi((char *) &receive_buffer[5]);
            }
            memset((char *) receive_buffer, 0, BUFFER_LENGTH);
            packet_received = false;
        }
    }
}

/**
 * This method sets all the chip configurations to what we'd like to use
 * > 32MHz Clock speed
 * > E1 Timer
 */
void configure() {
    OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc |                   // Select frequency range
                   OSC_XOSCSEL_XTAL_16KCLK_gc;                // Select start-up time
    OSC.CTRL |= OSC_XOSCEN_bm;                                // Enable oscillator
    while (!(OSC.STATUS & OSC_XOSCRDY_bm));                   // Wait for oscillator is ready

    OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | (OSC_PLLFAC_gm & 2);   // Select PLL source and multipl. factor
    OSC.CTRL |= OSC_PLLEN_bm;                                 // Enable PLL
    while (!(OSC.STATUS & OSC_PLLRDY_bm));                    // Wait for PLL is ready

    CCP = CCP_IOREG_gc;                                       // Security signature to modify clock
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;                            // Select system clock source
    OSC.CTRL &= ~OSC_RC2MEN_bm;                               // Turn off 2MHz internal oscillator
    OSC.CTRL &= ~OSC_RC32MEN_bm;

    PMIC.CTRL |= PMIC_LOLVLEN_bm;                             // Enable interrupts

    TCE1.CTRLB    = TC_WGMODE_NORMAL_gc;
    TCE1.CTRLA    = TC_CLKSEL_DIV1024_gc;            // Clock divisor. For 32MHz and D(1024), it does 31250 loops per second
    TCE1.PER      = F_CPU / (TIMER_PRESCALER * TICK_SPEED) - 1;         // Setup the speed of the TIMER
    TCE1.INTCTRLA = TC_OVFINTLVL_LO_gc;             // No interrupts

    sei();
}

/*
 * Method for changing the RGB values for the LED.
 * This changes the frequency of which the PWM operates at,
 * which in turn changes the brightness.
 */
void set_rgb(float r, float g, float b) {
    // Prevention.
    // If this isn't done you might risk frying your retina O_O
    r = clampf(r, 0, MAX_BRIGHTNESS);
    g = clampf(g, 0, MAX_BRIGHTNESS);
    b = clampf(b, 0, MAX_BRIGHTNESS);
    TCD0.CCD = (uint16_t) (LED_PER * r);
    TCD0.CCC = (uint16_t) (LED_PER * g);
    TCD1.CCA = (uint16_t) (LED_PER * b);
}

/*
 * Same method as above, only with an alpha component
 * This is basically the same as changing the brightness.
 */
void set_rgba(float r, float g, float b, float a) {
    set_rgb(r * a, g * a, b * a);
}

/**
 * Method for setting up the RGB lights using PWM
 */
void configure_lights() {
    // RGB -> 3, 2, 4 -> CCD, CCC, CCA
    PORTD.DIRSET = LED_PINS_RGB | LED_PIN_LAMPS;
    PORTD.DIRCLR = LED_PIN_MOTION_SENSOR;
    TCD0.CTRLA = TC_CLKSEL_DIV1024_gc;
    TCD1.CTRLA = TC_CLKSEL_DIV1024_gc;
    TCD0.CTRLB = TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.CTRLB = TC1_CCAEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD0.PER   = LED_PER;
    TCD1.PER   = LED_PER;
    set_rgb(0, 0, 0);
}

/**
 *  This method sets up the configuration for our NRF device.
 *  Individual settings can be changed in the enumerable defined at the top of this file.
 */
void configure_nrf() {
    nrfspiInit();
    nrfBegin();
    // Retry after 1 MS and retransmit if failed
    nrfSetRetries(NRF_SETUP_ARD_1000US_gc, NRF_SETUP_ARC_8RETRANSMIT_gc);
    // Set it to -6dBm for high power amplification
    nrfSetPALevel(NRF_RF_SETUP_PWR_6DBM_gc);

    // Data transmission rate, set at 250Kb
    nrfSetDataRate(NRF_RF_SETUP_RF_DR_250K_gc);

    // Enable cyclic redundancy check. This checks whether the packet is corrupt or not.
    // If not, parse it, else retry
    nrfSetCRCLength (NRF_CONFIG_CRC_8_gc);

    // Set the channel to what we've previously defined.
    // The band frequency is defined as f = (2400 + CH) MHz
    // For channel 6, this means 2,406 MHz
    nrfSetChannel(6);

    // Require acknowledgements
    nrfSetAutoAck(1);
    nrfEnableDynamicPayloads();
    nrfClearInterruptBits();

    // Clear the Rx and Tx-buffers
    nrfFlushRx();
    nrfFlushTx();

    // Interrupt Pin
    PORTF.INT0MASK |= PIN6_bm;
    PORTF.PIN6CTRL  = PORT_ISC_FALLING_gc;
    PORTF.INTCTRL  |= (PORTF.INTCTRL & ~PORT_INT0LVL_gm) | PORT_INT0LVL_LO_gc ;

    // Opening pipes
    load_pipes_nrf();
    nrfStartListening();
    nrfPowerUp();
}

/**
 * Method for loading the pipes for reading and writing.
 * We select our device address for writing, so the central unit can read from our pipe.
 * We open the central pipe for reading only. This means that if other nodes want to communicate
 * with this window_node, we have to send the message to the central window_node first.
 */
void load_pipes_nrf() {

    nrfOpenWritingPipe((uint8_t *) addresses[DEVICE_IDX]);
    nrfOpenReadingPipe(0, (uint8_t *) addresses[CENTRAL_DEVICE_IDX]);
}

/**
 * Method of sending packets via the NRF chip.
 * @note The max size of the buffer can be 32 bytes
 * @param buffer The buffer to be sent
 * @param bufferSize The size of the buffer
 */
void transmit_nrf(char* buffer, uint16_t bufferSize) {
    cli();                                                  // Disable interrupts
    nrfStopListening();                                     // Stop listening
    nrfWrite((uint8_t *) buffer, bufferSize);      // Write to the targetted device
    nrfStartListening();                                    // Start listening for input again
    sei();                                                  // re-enable interrupts
}


/**
 * The TIMER interrupt vector. This one is triggered TICK_SPEED times per second
 * This function sets the timer_triggered flag to true, which is then used in the main loop.
 * This is to minimize the amount of time spent in the interrupt routine.
 */
ISR(TCE1_OVF_vect) {
    timer_triggered = true;
}

/**
 * The interrupt vector for receiving NRF signals.
 * Every time a packet is received, it sets the 'packet_received' flag to true.
 * This is then in turn parsed in the main loop.
 */
ISR(PORTF_INT0_vect) {
    uint8_t  tx_ds, max_rt, rx_dr, len;

    nrfWhatHappened(&tx_ds, &max_rt, &rx_dr);

    if ( rx_dr ) {
        len = nrfGetDynamicPayloadSize();
        nrfRead((char *) receive_buffer, len );
        receive_buffer[len] = '\0';
        packet_received = true;
    }
}
