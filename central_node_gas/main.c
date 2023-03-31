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
#define TICK_SPEED 2
#define TWO_P_10 1024

// The maximum size of the char buffers
#define BUFFER_LENGTH 32

// ADC Values
#define ADC_MAX 4095
#define ADC_MIN 200
#define ADC_REF_V 1.6f

#define PIN_CH4 PIN0_bm
#define PIN_CO2 PIN0_bm
#define TX_SDS011 PIN0_bm
#define RX_SDS011 PIN1_bm

// Macros for useful math functions. absf returns the floating point absolute value.
#define absf(x)         ((x) < 0   ? -(x) : (x))
#define clampf(x, a, b) ((x) < (a) ? (a) : (x) > (b) ? (b) : (x))

// Macros for converting ADC res to temperature, according to the datasheet of the LMT85
#define ADC_TO_MVOLT(res, ref) ((3200 / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN))

// The addresses of all the nodes.
const char * addresses[] = {"1_dev", "2_dev", "3_dev",
                            "4_dev", "5_dev", "6_dev"};

// Buffers for receiving and sending packets.
volatile uint8_t receive_buffer [BUFFER_LENGTH];
volatile uint8_t transmit_buffer[BUFFER_LENGTH];
volatile bool    timer_triggered = false;
volatile bool    packet_received = false;
volatile int16_t received_byte = -1;

// NRF Configuration. Can change later on.
typedef enum {
    NRF_CHANNEL    = 6,                         // Frequency channel (2400 + CH)MHz
    NRF_DATA_SPEED = NRF_RF_SETUP_RF_DR_250K_gc,// Data transfer speed
    NRF_CRC        = NRF_CONFIG_CRC_8_gc,       // Cyclic redundancy check length
} nrf_cfg;

// Predefine the functions for later use.
void Configure();
void NRFInit();
void NRFLoadPipes();
void NRFSendPacket(char* buffer, uint16_t bufferSize);
void ReadADC(ADC_t* adc, uint16_t * dst);

// The main function, clearly
int main(void) {

    Configure();
    USARTInit(F_CPU, UARTF0_BAUD);
    NRFInit();

    uint16_t ch4_res = 0, co2_res = 0;

    bool phase = true;

    while (true) {

        if (received_byte > -1) {
            printf("%c", received_byte);
            received_byte = -1;
        }


        if (timer_triggered) {

            ReadADC(
                    phase ? &ADCA : &ADCB,
                    phase ? &ch4_res : &co2_res);
            sprintf((char *) transmit_buffer, "%s_ppm=%d",
                    phase ? "ch4" : "co2",
                    phase ? ch4_res : co2_res);

            printf("%s\n", transmit_buffer);

            NRFSendPacket((char *) transmit_buffer, strlen((char *) transmit_buffer));
            memset((char *) transmit_buffer, 0, BUFFER_LENGTH);
            phase = !phase;
            timer_triggered = false;
        }
    }
}

/**
 * This method sets all the chip configurations to what we'd like to use
 * > 32MHz Clock speed
 * > E1 Timer
 */
void Configure() {
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

    PORTA.DIRCLR = PIN_CH4;
    PORTB.DIRCLR = PIN_CO2;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;               // Multiplex selection for pin 2
    ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
    ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc;     // Single ended input without gain
    ADCB.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;                // Reference voltage, INTVCC = 3.3V / 1.6 ~ 2.0V
    ADCB.REFCTRL     = ADC_REFSEL_INTVCC_gc;
    ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc;             // Range of number conversion, 185 - 2^14-1
    ADCB.CTRLB       = ADC_RESOLUTION_12BIT_gc;
    ADCA.PRESCALER   = ADC_PRESCALER_DIV512_gc;             // F_CPU / PRESCALER -> Speed of conversoin
    ADCB.PRESCALER   = ADC_PRESCALER_DIV512_gc;
    ADCA.CTRLA       = ADC_ENABLE_bm;                       // Turn on the ADC converter
    ADCB.CTRLB       = ADC_ENABLE_bm;

    TCE1.CTRLB    = TC_WGMODE_NORMAL_gc;
    TCE1.CTRLA    = TC_CLKSEL_DIV1024_gc;                   // Clock divisor. For 32MHz and D(1024), it does 31250 loops per second
    TCE1.PER      = F_CPU / (TWO_P_10 * TICK_SPEED) - 1;    // Setup the speed of the TIMER
    TCE1.INTCTRLA = TC_OVFINTLVL_LO_gc;

    sei();
}

ISR(USARTE0_RXC_vect) {
    received_byte = USARTE0.DATA;
}

void InitUSART_SDS011() {
    PORTE.DIRCLR = RX_SDS011;
    PORTE.DIRSET = TX_SDS011;
    PORTE.OUTSET = TX_SDS011;
    uint16_t bsel = 3317;
    int8_t bscale = -4;
    USARTE0.BAUDCTRLA = (bsel & USART_BSEL_gm);
    USARTE0.BAUDCTRLB = ((bscale << USART_BSCALE_gp) & USART_BSCALE_gm) |
                        ((bsel >> 8) & ~USART_BSCALE_gm);
    USARTE0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

    USARTE0.CTRLA = USART_RXCINTLVL_MED_gc |
                    USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;

    PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
}

/**
 *  This method sets up the configuration for our NRF device.
 */
void NRFInit(void) {
    // Set up the transmission
    nrfspiInit();
    nrfBegin();
    // Retry after 1 MS and retransmit if failed
    nrfSetRetries(NRF_SETUP_ARD_1000US_gc, NRF_SETUP_ARC_8RETRANSMIT_gc);
    // Set it to -6dBm for high power amplification
    nrfSetPALevel(NRF_RF_SETUP_PWR_6DBM_gc);

    // Data transmission rate, set at 250Kb
    nrfSetDataRate((nrf_rf_setup_rf_dr_t) NRF_DATA_SPEED);

    // Enable cyclic redundancy check. This checks whether the packet is corrupt or not.
    // If not, parse it, else retry
    nrfSetCRCLength((nrf_config_crc_t) NRF_CRC);

    // Set the channel to what we've previously defined.
    // The band frequency is defined as f = (2400 + CH) MHz
    // For channel 6, this means 2,406 MHz
    nrfSetChannel(NRF_CHANNEL);

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
    NRFLoadPipes();
    nrfStartListening();
    nrfPowerUp();
}

/**
 * Method of reading data from the analog to digital converter
 * @param adc The adc object to use
 * @param dst The target variable to store the retrieved data in.
 */
void ReadADC(ADC_t* adc, uint16_t * dst) {
    adc->CH0.CTRL |= ADC_CH_START_bm;                    // start ADC conversion
    while ( !(adc->CH0.INTFLAGS & ADC_CH_CHIF_bm) ) ;    // wait until it's ready
    *dst = adc->CH0.RES;
    adc->CH0.INTFLAGS |= ADC_CH_CHIF_bm;                 // reset interrupt flag
}

/**
 * Method for loading the pipes for reading and writing.
 * We select our device address for writing, so the central unit can read from our pipe.
 * We open the central pipe for reading only. This means that if other nodes want to communicate
 * with this window_node, we have to send the message to the central window_node first.
 */
void NRFLoadPipes() {

    nrfOpenWritingPipe((uint8_t *) addresses[DEVICE_IDX]);
    nrfOpenReadingPipe(0, (uint8_t *) addresses[CENTRAL_DEVICE_IDX]);
}

/**
 * Method of sending packets via the NRF chip.
 * @note The max size of the buffer can be 32 bytes
 * @param buffer The buffer to be sent
 * @param bufferSize The size of the buffer
 */
void NRFSendPacket(char* buffer, uint16_t bufferSize) {

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
