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
#include "nrf24spiXM2.h"
#include "nrf24L01.h"
#include "serialF0.h"
#include <stdbool.h>

// This is the index of the current device. The device address is hereby selected as addresses[DEVICE_IDX]
#define DEVICE_IDX 4

// The index of the central device. This is always 0
#define CENTRAL_DEVICE_IDX 0

// Speed of which the timer interrupt operates
#define TICK_SPEED 20
#define TIMER_PRESCALER 1024
#define DELTA_T (1.0F / TICK_SPEED)
#define INTERPOLATION_FACTOR (1.0f / 10.0f)

// The maximum size of the char buffers
#define BUFFER_LENGTH 32

// ADC Values
#define ADC_MAX 4095
#define ADC_MIN 200
#define ADC_REF_V 1.6f

#define LED_PIN_MTNSENSOR PIN3_bm
#define LED_PIN_RED    PIN0_bm
#define LED_PIN_GREEN  PIN1_bm
#define LED_PIN_BLUE   PIN2_bm
#define LED_PER        100
#define MAX_BRIGHTNESS (1.0f / 16.0f) // 0.00001 = 1 / (1 << 5)
#define LAMP_SLEEP_TIME 5

// Macros for useful math functions. absf returns the floating point absolute value.

#define clampf(x, a, b) ((x) < (a) ? (a) : (x) > (b) ? (b) : (x))
// Macros for converting ADC res to temperature, according to the datasheet of the LMT85

#define ADC_TO_MVOLT(res, ref) ((3200 / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN))

// The addresses of all the nodes.
const char * addresses[] = {"1_dev", "2_dev", "3_dev",
                            "4_dev", "5_dev", "6_dev"};

// Buffers for receiving and sending packets.
volatile uint8_t rx_buffer[BUFFER_LENGTH];
volatile uint8_t transmit_buffer[BUFFER_LENGTH];
volatile bool    timer_triggered = false;
volatile bool    packet_received = false;

volatile bool    second_passed   = false;

// Vector struct. handy.
typedef struct {
    float x, y, z, w;
} vec4;

inline void v_translate(vec4 * vector, float x, float y, float z, float w) {
    vector->x = x;
    vector->y = y;
    vector->z = z;
    vector->w = w;
}

enum {
    TEMPERATURE_MIN = 18,
    TEMPERATURE_MAX = 30,
    HUMIDITY_MIN = 60,
    HUMIDITY_MAX = 100,
    CO2_MAX_RES = 1600
} THRESHOLDS;

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
    USARTInit(F_CPU, UARTF0_BAUD);
    configure_nrf();
    configure_lights();

    vec4 color, color_lerp;
    v_translate(&color, 0, 0, 0, 0);
    v_translate(&color_lerp, 1, 1, 1, 0);

    uint16_t seconds_passed = 0;
    bool lamp_enabled = false;

    uint8_t humidity = 0;
    int8_t temperature = 0;
    uint16_t co2_res = 0;

    uint8_t danger_level = 0;

    while (true) {

        if (second_passed) {
            seconds_passed++;

            if (PORTD.IN & PIN3_bm) {
                lamp_enabled = true;
                seconds_passed = 0;
            }

            if (seconds_passed >= LAMP_SLEEP_TIME) {
                lamp_enabled = false;
                seconds_passed = 0;
            }

            second_passed = false;
        }

        if (timer_triggered) {
            color_lerp.w = lamp_enabled ? MAX_BRIGHTNESS : 0.0f;

            color_lerp.x = clampf((float)(temperature - TEMPERATURE_MIN) / (TEMPERATURE_MAX - TEMPERATURE_MIN), 0.1f, 1); // RED
            color_lerp.y = clampf((float)(CO2_MAX_RES) / (float)(co2_res - CO2_MAX_RES), 0, 1);
            color_lerp.z = clampf((float)(humidity - HUMIDITY_MIN) / (HUMIDITY_MAX - HUMIDITY_MIN), 0.1f, 1);

            // Linear interpolation
            v_translate(&color,
                        color.x + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.x - color.x),
                        color.y + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.y - color.y),
                        color.z + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.z - color.z),
                        color.w + DELTA_T * INTERPOLATION_FACTOR * (color_lerp.w - color.w));

            set_rgba(color.x, color.y, color.z, color.w);

            timer_triggered = false;
        }

        if (packet_received) {

            printf("%s\n", rx_buffer);

            if (!strncmp((char *) rx_buffer, "co2_res=", 8)) {
                co2_res = atoi((char *) &rx_buffer[8]);
            } else if (!strncmp((char *) rx_buffer, "temp=", 5)) {
                temperature = atoi((char *) &rx_buffer[5]);
            } else if (!strncmp((char *) rx_buffer, "humid=", 6)) {
                humidity = atoi((char *) &rx_buffer[6]);
            }

            // Clear the receive-buffer.
            memset((char *) rx_buffer, 0, BUFFER_LENGTH);
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

    TCE0.CTRLB = TC_WGMODE_NORMAL_gc;
    TCE0.CTRLA = TC_CLKSEL_DIV1024_gc;
    TCE0.PER   = F_CPU / (TIMER_PRESCALER) - 1;
    TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;

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

    TCD0.CCA = (uint16_t) (LED_PER * r);
    TCD0.CCB = (uint16_t) (LED_PER * g);
    TCD0.CCC = (uint16_t) (LED_PER * b);
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

    PORTD.DIRSET = LED_PIN_RED | LED_PIN_GREEN | LED_PIN_BLUE;
    PORTD.DIRCLR = LED_PIN_MTNSENSOR;
    TCD0.CTRLA = TC_CLKSEL_DIV64_gc;
    TCD0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD0.PER   = LED_PER - 1;
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
    PORTF.INTCTRL  |= (PORTF.INTCTRL & ~PORT_INT0LVL_gm) |
                      PORT_INT0LVL_LO_gc ; // Interrupts On

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
 * Interrupt for notifying when a second has passed.
 */
ISR(TCE0_OVF_vect) {
    second_passed = true;
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
        nrfRead((char *) rx_buffer, len );
        rx_buffer[len] = '\0';
        packet_received = true;
    }
}
