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

#define DEVICE_IDX           4      // The index of the device. Used to retrieve our NRF address.
#define CENTRAL_DEVICE_IDX   0      // The index of the central device. This is always 0

#define TICK_SPEED           3      // The frequency at which the timer interrupt functions, for timed methods.
#define TIMER_PRESCALER   1024      // Clock prescaler for the timer interrupt, PER = F_CPU / (PRESCALER * Hz) - 1

#define BUFFER_LENGTH       32      // The maximum size of the char buffers

#define SDS011_PMx(HI, LO) (((HI) * 256 + (LO)) / 10)  // According to the oh-so-amazing Chinese datasheet
#define SDS011_MESSAGE_LENGTH    10   //These values here and below are found in the datasheet.
#define SDS011_MESSAGE_HEADER   0xAA  // Message header. This is the first byte in the UART sequence
#define SDS011_MESSAGE_TAIL     0xAB  // Message tail. This is the final bit in the sequence
#define SDS011_PM10_HI_IDX 5          // Index for the HI byte for the PM10 value
#define SDS011_PM10_LO_IDX 4          // Index for the LO byte for the PM10 value
#define SDS011_PM25_HI_IDX 3          // Index for the HI byte for the PM2.5 value
#define SDS011_PM25_LO_IDX 2          // Index for the LO byte for the PM2.5 value
#define SDS011_PIN_RX       PIN2_bm   // The port number for USARTE0 RX
#define SDS011_PIN_TX       PIN3_bm   // The port number for USARTE0 TX
#define SDS011_BSEL         (3317)    // BSEL for the SDS011 for a baud rate of 9600
#define SDS011_BSCALE       (-4)      // BSCALE for BAUD 9600

#define PIN_CO2        PIN2_bm        // Port index for the CO2 ADC

#define PACKET_COUNT 3                // The amount of packets this device sends to the main node. CO2, PM10 and PM2.5

#define clampf(x, a, b) ((x) < (a) ? (a) : (x) > (b) ? (b) : (x)) // Macro for clamping a number between bounds.

// The addresses of all the nodes.
const char * addresses[] = {"1_dev", "2_dev", "3_dev",
                            "4_dev", "5_dev", "6_dev"};

// Buffers for receiving and sending packets.
volatile uint8_t receive_buffer [BUFFER_LENGTH];
volatile uint8_t transmit_buffer[BUFFER_LENGTH];
volatile bool    timer_triggered = false;
volatile bool    packet_received = false;
volatile bool    sds011_received = false;
volatile uint8_t sds011_rx_data  = 0;

// Predefine the functions for later use.
void confugure();
void configure_nrf();
void load_pipes_nrf();
void transmit_nrf(char* buffer, uint16_t bufferSize);
void read_adc(ADC_t* adc, uint16_t * dst);
void configure_sds011();

// The main function, clearly
int main(void) {

    confugure();
    USARTInit(F_CPU, UARTF0_BAUD);
    configure_sds011();
    configure_nrf();

    uint16_t co2_res = 0, PM10 = 0, PM25 = 0;

    uint8_t sds011_wr_data[SDS011_MESSAGE_LENGTH], sds011_wr_idx = 0;
    uint8_t packet_phase = 0;

    while (true) {

        // This gets triggered when the SDS011 sends a byte via UART
        if (sds011_received) {

            // Check whether we've received a start byte 0xAA, then set the write idx to 0 and clear the buffer
            if (sds011_rx_data == SDS011_MESSAGE_HEADER) {

                memset(sds011_wr_data, 0, SDS011_MESSAGE_LENGTH);
                sds011_wr_idx = 0;

            // Check whether we've received the tail byte and if the index is MESSAGE_LENGTH-1 (end index)
            // If so, parse the content of the packet and clear the buffer.
            } else if (sds011_rx_data == SDS011_MESSAGE_TAIL && sds011_wr_idx == SDS011_MESSAGE_LENGTH - 1) {

                PM10 = SDS011_PMx(sds011_wr_data[SDS011_PM10_HI_IDX], sds011_wr_data[SDS011_PM10_LO_IDX]);
                PM25 = SDS011_PMx(sds011_wr_data[SDS011_PM25_HI_IDX], sds011_wr_data[SDS011_PM25_LO_IDX]);
                sds011_wr_idx = 0;
                memset(sds011_wr_data, 0, SDS011_MESSAGE_LENGTH);

            // Store the retrieved data in the buffer;
            } else {
                sds011_wr_data[sds011_wr_idx] = sds011_rx_data;
            }

            // Increase the index count. This ranges from 0 to SDS011_MESSAGE_LENGTH-1
            sds011_wr_idx = (sds011_wr_idx + 1) % SDS011_MESSAGE_LENGTH;
            sds011_received = false;
        }

        if (timer_triggered) {
            read_adc(&ADCA,&co2_res);
            sprintf((char *) transmit_buffer, "%s=%d",
                    packet_phase == 0 ? "co2_res" : packet_phase == 1 ? "PM10" : "PM25",
                    packet_phase == 0 ? co2_res   : packet_phase == 1 ? PM10   : PM25);

            packet_phase = (packet_phase + 1) % PACKET_COUNT;

            transmit_nrf((char *) transmit_buffer, strlen((char *) transmit_buffer));
            memset((char *) transmit_buffer, 0, BUFFER_LENGTH);
            timer_triggered = false;
        }
    }
}

/**
 * This method sets all the chip configurations to what we'd like to use
 * > 32MHz Clock speed
 * > E1 Timer
 */
void confugure() {
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

    // CO2 ADC
    PORTA.DIRCLR = PIN_CO2;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;               // Multiplex selection for pin 2
    ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc;     // Single ended input without gain
    ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;                // Reference voltage, INTVCC = 3.3V / 1.6 ~ 2.0V
    ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc;             // Range of number conversion, 185 - 2^14-1
    ADCA.PRESCALER   = ADC_PRESCALER_DIV512_gc;             // F_CPU / TIMER_PRESCALER -> Speed of conversoin
    ADCA.CTRLA       = ADC_ENABLE_bm;                       // Turn on the ADC converter

    TCE1.CTRLB    = TC_WGMODE_NORMAL_gc;
    TCE1.CTRLA    = TC_CLKSEL_DIV1024_gc;                   // Clock divisor. For 32MHz and D(1024), it does 31250 loops per second
    TCE1.PER      = F_CPU / (TIMER_PRESCALER * TICK_SPEED) - 1;    // Setup the speed of the TIMER
    TCE1.INTCTRLA = TC_OVFINTLVL_LO_gc;

    sei();
}

/**
 * Interrupt routine for receiving data from the SDS011 via USARTE0
 * Method sets the received flag to true, which is then parsed in
 * the main method using the retrieved data.
 */
ISR(USARTE0_RXC_vect) {
    sds011_rx_data = USARTE0.DATA;
    sds011_received = true;
}

/**
 * Method for initializing the USART settings for the SDS011 using PORT E
 * The sensor uses a BAUD rate of 9600. The BSEL and BSCALE values are pre-calculated
 * to suit this baud rate.
 */
void configure_sds011() {
    PORTE.DIRCLR = SDS011_PIN_RX;
    PORTE.DIRSET = SDS011_PIN_TX;
    PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
    USARTE0.BAUDCTRLA = (SDS011_BSEL & USART_BSEL_gm);
    USARTE0.BAUDCTRLB = ((SDS011_BSCALE << USART_BSCALE_gp) & USART_BSCALE_gm) |
                        ((SDS011_BSEL >> 8) & ~USART_BSCALE_gm);
    USARTE0.CTRLB = USART_RXEN_bm;

    USARTE0.CTRLA = USART_RXCINTLVL_MED_gc;

    PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
}

/**
 *  This method sets up the configuration for our NRF device.
 */
void configure_nrf(void) {
    // Set up the transmission
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
    nrfSetCRCLength(NRF_CONFIG_CRC_8_gc);

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
 * Method of reading data from the analog to digital converter
 * @param adc The adc object to use
 * @param dst The target variable to store the retrieved data in.
 */
void read_adc(ADC_t* adc, uint16_t * dst) {
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
