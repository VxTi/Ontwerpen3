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

#define BUFFER_LENGTH 32
#define ADC_MAX 4095
#define ADC_MIN 200

#define ADC_REF_V 1.6F

// This is the index of the current device. The device address is hereby selected as addresses[DEVICE_IDX]
#define DEVICE_IDX 2

// The index of the central device. This is always 0
#define CENTRAL_DEVICE_IDX 0

// The speed of sending the data from our sensor
#define TICK_SPEED 10
#define TWO_P_10 1024     // 2 ^ 10
#define DEFAULT_TEMPERATUER_VALUE 20

// The pin indices for the ports we'd like to use
#define PIN_MOTOR_LEFT   PIN5_bm
#define PIN_MOTOR_RIGHT  PIN4_bm
#define PIN_SWITCH_LEFT  PIN0_bm
#define PIN_SWITCH_RIGHT PIN1_bm
#define PIN_MANUAL_CTRL  PIN2_bm

// Macros to convert ADC res to temperature (This is for the LMT85)
#define ADC_TO_MVOLT(res, ref) ((3200 / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN))
#define MVOLT_TO_C(v)           ((8.194f - sqrtf(67.141636f + 4 * 0.00262f * (1324.0f - (v)))) / (2 * -0.00262f) + 30.0f)

// The addresses of all the nodes.
const char * addresses[] = {"1_dev", "2_dev", "3_dev",
                           "4_dev", "5_dev", "6_dev"};

// Window command flags and values.
// Triggering command flags is used the same way as the process flag as stated above.
typedef enum {
    WDW_CMD_DO_NOTHING  = 0x0,
    WDW_CMD_OPEN        = 0x1,
    WDW_CMD_CLOSE       = 0x2,
    WDW_HUMID_THRESHOLD = 60,
    WDW_TEMP_CLOSE      = 15,       // Window closes under this temperature
    WDW_TEMP_OPEN       = 23,       // Window opens above this temperature
    WDW_TEMP_CLOSE_THRESHOLD = 8,   // The outside temperature at which the window forcefully closes, unless manual mode is turned on.
    WDW_TEMP_OPEN_THRESHOLD  = 30,  // The outside temperature at which the window forcefully opens.
} wdw_flags;

// NRF Configuration. Can change later on.
typedef enum {
    NRF_CHANNEL    = 6,                         // Frequency channel (2400 + CH)MHz
    NRF_DATA_SPEED = NRF_RF_SETUP_RF_DR_250K_gc,// Data transfer speed
    NRF_CRC        = NRF_CONFIG_CRC_8_gc,       // Cyclic redundancy check length
} nrf_cfg;

// Buffers for receiving and sending packets.
volatile char receive_buffer[BUFFER_LENGTH];

// Flags that can be called by interrupt functions.
volatile bool packet_received = false;
volatile bool timer_triggered = false;

// Predefine the functions for later use.
void Configure();
void NRFInit();
void NRFLoadPipes();
void NRFSendPacket(char* buffer, uint16_t bufferSize);
void ReadADC(ADC_t* adc, uint16_t * dst);
bool ShouldOpen(uint8_t humidity, int16_t inside_temperature, int16_t outside_temperature, bool manual_mode);
bool ShouldClose(int16_t inside_temperature, int16_t outside_temperature, bool manual_mode);

// The main function, clearly
int main(void) {

    Configure();
    USARTInit(F_CPU, UARTF0_BAUD);
    NRFInit();

    char transmit_buffer[BUFFER_LENGTH];
    uint8_t switches;
    uint8_t manual_mode, motor_pin_cmd = 0;

    uint8_t inside_humidity    = DEFAULT_TEMPERATUER_VALUE;
    int8_t  inside_temperature  = DEFAULT_TEMPERATUER_VALUE;
    int16_t outside_temperature = DEFAULT_TEMPERATUER_VALUE;

    uint16_t outside_temperature_res = 0;
    wdw_flags window_state = 0;


    // Set the DIR bits of the switches to LOW to define them as inputs.
    // Set the DIR bits for the motor rotations to HIGH to define them as outputs.
    PORTD.DIRCLR = PIN_SWITCH_LEFT | PIN_SWITCH_RIGHT | PIN_MANUAL_CTRL;
    PORTD.DIRSET = PIN_MOTOR_LEFT | PIN_MOTOR_RIGHT;
    PORTD.OUTCLR = PIN_MOTOR_LEFT | PIN_MOTOR_RIGHT;

    while (1) {

        // Check whether we have to read the sensors.
        // The frequency of this check is defined above as TICK_SPEED
        if (timer_triggered) {

            // Calculate the voltage from the ADC res, then use the formula we gathered from the LMT85
            // to calculate the inside_temperature. We can then use this in further calculations and operations.
            ReadADC(&ADCA, &outside_temperature_res);
            outside_temperature = (int16_t) MVOLT_TO_C(ADC_TO_MVOLT(outside_temperature_res, ADC_REF_V));

            // This value is higher than one if the bit is set.
            manual_mode = (PORTD.IN & PIN_MANUAL_CTRL); // 1 << pin

            // Set the bits of the selected pins to high.
            // We can use these values later for calculations.
            switches = (PORTD.IN & PIN_SWITCH_LEFT) | (PORTD.IN & PIN_SWITCH_RIGHT);

            // Check if the window isn't doing anything
            if (window_state == WDW_CMD_DO_NOTHING) {

                // Check whether the window should open or close, depending on various variables.
                window_state =
                        ShouldOpen(inside_humidity, inside_temperature, outside_temperature, manual_mode) ? WDW_CMD_OPEN :
                        ShouldClose(inside_temperature, outside_temperature, manual_mode) ? WDW_CMD_CLOSE :
                            WDW_CMD_DO_NOTHING;
            }

            // Set the pin command to the window state. If the switches are turned on, prevent the motor
            // from moving any further. If not, we just check in what direction the motor is moving.
            motor_pin_cmd = manual_mode ? 0 :
                    (window_state & WDW_CMD_OPEN) ?  (switches & PIN_SWITCH_LEFT  ? 0 :  PIN_MOTOR_LEFT)  :
                    (window_state & WDW_CMD_CLOSE) ? (switches & PIN_SWITCH_RIGHT ? 0 :  PIN_MOTOR_RIGHT) : motor_pin_cmd;

            sprintf(transmit_buffer, "%s%s%s%s[%d]",
                    switches & PIN_SWITCH_LEFT ? "#" : "_",
                    switches & PIN_SWITCH_RIGHT ? "#" : "_",
                    manual_mode > 0 ? "#" : "_",
                    motor_pin_cmd == PIN_MOTOR_LEFT ? "L" :
                    motor_pin_cmd == PIN_MOTOR_RIGHT ? "R" : "N", outside_temperature);
            NRFSendPacket(transmit_buffer, strlen(transmit_buffer));

            timer_triggered = false;
        }

        // First we clear the bits of the PIN_MOTOR_LEFT and PIN_MOTOR_RIGHT then we AND the command.
        PORTD.OUT = (PORTD.OUT & ~(PIN_MOTOR_LEFT | PIN_MOTOR_RIGHT)) | motor_pin_cmd;

        // Checking whether there's a packet
        if (packet_received) {

            if (!strncmp((char *) receive_buffer, "temp=", 5))
                inside_temperature = atoi((char *) &receive_buffer[5]);

            if (!strncmp((char *) receive_buffer, "humid=", 6))
                inside_humidity = atoi((char *) &receive_buffer[6]);

            // Clear the receive-buffer.
            memset((char *) receive_buffer, 0, BUFFER_LENGTH);
            packet_received = false;
        }
    }
}

bool ShouldClose(int16_t inside_temperature, int16_t outside_temperature, bool manual_mode) {
    return manual_mode ? (inside_temperature < WDW_TEMP_CLOSE_THRESHOLD) :
           outside_temperature < WDW_TEMP_CLOSE_THRESHOLD ||
           inside_temperature  < WDW_TEMP_CLOSE;
}

bool ShouldOpen(uint8_t humidity, int16_t inside_temperature, int16_t outside_temperature, bool manual_mode) {
    return humidity > WDW_HUMID_THRESHOLD ||
           manual_mode ?
           (inside_temperature > WDW_TEMP_OPEN_THRESHOLD || outside_temperature > WDW_TEMP_OPEN_THRESHOLD) :
           inside_temperature > WDW_TEMP_OPEN;
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

    PORTA.DIRCLR     = PIN2_bm;                             // We'd like to use pin 2 for our converter
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;               // Multiplex selection for pin 2
    ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc;     // Single ended input without gain
    ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;                // Reference voltage, INTVCC = 3.3V / 1.6 ~ 2.0V
    ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc;             // Range of number conversion, 185 - 2^14-1
    ADCA.PRESCALER   = ADC_PRESCALER_DIV512_gc;             // F_CPU / PRESCALER -> Speed of conversoin
    ADCA.CTRLA       = ADC_ENABLE_bm;                       // Turn on the ADC converter

    TCE1.CTRLB    = TC_WGMODE_NORMAL_gc;
    TCE1.CTRLA    = TC_CLKSEL_DIV1024_gc;            // Clock divisor. For 32MHz and D(1024), it does 31250 loops per second
    TCE1.PER      = F_CPU / (TWO_P_10 * TICK_SPEED) - 1;         // Setup the speed of the TIMER
    TCE1.INTCTRLA = TC_OVFINTLVL_LO_gc;              // No interrupts

    sei();
}

/**
 *  This method sets up the configuration for our NRF device.
 *  Individual settings can be changed in the enumerable defined
 *  at the top of this file.
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
 * @return The value retrieved by the adc
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
 * The TIMER interrupt vector. This one is triggered TICK_SPEED times per second.
 * The function sets the timer_triggered flag to true, which is then parsed in the main loop.
 * This is to minimize the amount of time spent in the interrupt function.
 */
ISR(TCE1_OVF_vect) {
    timer_triggered = true;
}

/**
 * The interrupt vector for receiving NRF signals.
 * Every time a packet is received, it sets the packet_received flag to true,
 * which is then in turn parsed in the main loop.
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
