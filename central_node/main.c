//
// Created by Luca Warmenhoven on 03/02/2023.
//

#define F_CPU       32000000UL
#define UARTF0_BAUD 115200UL

#define __AVR_ATxmega256A3U__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include "nrf24spiXM2.h"
#include "nrf24L01.h"
#include "serialF0.h"
#include <stdbool.h>

#define CENTRAL_DEVICE_IDX         (0)

// The humidity is logarithmically linear. This value is calculated from the data from the datasheet
// The resistance has to be high to create an accurate voltage divider.
#define HUMID_RESISTANCE         (5600000UL)  // Resistor value
#define HUMID_R_TEMP_GROWTH_FACTOR  (0.923F)  // Average growth factor of temperature.
#define HUMID_GROWTH_FACTOR       (-22.953F)  // ΔH / ΔLOG -> ΔT = 1, ΔLOG ≈ -0.04356696835
#define HUMID_OFFSET_VALUE     (168.010926F)  // Calculated from sheet. Hoff = Δ - L. * ΔH / ΔLOG

#define TICK_SPEED          5 // Frequency at which the timer interrupt is called.

#define ADC_REF_V       1.6f  // Input voltage divisor
#define ADC_MIN          200  // Minimal ADC value with 12 bit res
#define ADC_MAX         4095  // Max ADC value with 12 bit res
#define ADC_VIN         3200  // Input voltage reference in mV
#define ADC_TO_MVOLT(res, ref) ((ADC_VIN / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN)) // Macro for converting ADC to mV

#define TIMER_PRESCALER 1024  // Clock speed prescaler. PER = F_CPU / (PRESCALER * Hz) - 1

#define BUFFER_SIZE (32)      // Max char buffer size

/**
 * Method for converting the ADC res to temperature based on the formula from the datasheet.
 * The result of the calculation is then stored in the provided dst variable
 * @param temperatureRes  The ADC result
 * @param varDst          The variable to store the result in
 */
static void calculate_temperature(uint16_t temperatureRes, int16_t * varDst) {
    *varDst = (int16_t) ((8.194f - sqrtf(67.141636f + 4 * 0.00262f * (1324.0f - ADC_TO_MVOLT(temperatureRes, ADC_REF_V)))) / (2 * -0.00262f) + 30.0f);
}

/**
 * Method for calculating the relative humidity. This method uses values that are calculated from the
 * datasheet. This converts the resistance, which is logarithmic, to a linear function of humidity
 * (This has a slight accuracy deficiency of about +/- 5%)
 * @param adcRes        The ADC result from the humidity resistor
 * @param temperature   The temperature, which is measured based on thermal conductivity (also based on humidity)
 * @param varDst        The variable to store the result of the calculation in
 */
static void calculate_humidity(uint16_t adcRes, uint16_t temperature, uint8_t * varDst) {
    float R_humid = HUMID_RESISTANCE * ((float)(ADC_MAX - ADC_MIN) / (float)(adcRes - ADC_MIN) - 1);
    *varDst = (uint8_t) (HUMID_OFFSET_VALUE +
                        log10f(R_humid * powf(HUMID_R_TEMP_GROWTH_FACTOR, temperature)) * HUMID_GROWTH_FACTOR);
}

const char *addresses[] = {"stm_0", "stm_1", "stm_2", "stm_3", "stm_4", "stm_5"};

volatile uint8_t receive_buffer[BUFFER_SIZE];
volatile bool packet_received = false;
volatile bool timer_triggered = false;

void load_pipes_nrf();
void configure_nrf ();
void transmit_nrf  (char * buffer, uint16_t bufferSize);
void read_adc      (ADC_t * adc, uint16_t * dst);
void configure     ();

// here we go again
int main(void) {

    configure();
    configure_usartf0(F_CPU, UARTF0_BAUD);
    configure_nrf();

    char transmit_buffer[BUFFER_SIZE];

    uint16_t temperature_res, humidity_res;
    int16_t temperature;
    uint8_t humidity;

    bool phase = false;

    while (true) {

        if (timer_triggered) {
            read_adc(&ADCA, &temperature_res);                       // Read ADC value from temperature sensor
            read_adc(&ADCB, &humidity_res);                          // Read ADC value from humidity sensor
            calculate_temperature(temperature_res, &temperature);
            calculate_humidity(humidity_res, temperature, &humidity);

            // Clear, fill, transmit :)
            memset(transmit_buffer, 0, BUFFER_SIZE);
            sprintf((char *) transmit_buffer, "%s=%d", phase ? "temp" : "humid", phase ?  temperature : humidity);
            transmit_nrf((char *) transmit_buffer, strlen((char *) transmit_buffer));

            phase = !phase;
            timer_triggered = false;
        }

        if (packet_received) {

            if (!strncmp((char *) receive_buffer, "PM10", 4) || !strncmp((char *) receive_buffer, "PM25", 4) || !strncmp((char *) receive_buffer, "co2_res", 7)) {

                // Copy RX to TX buffer and send
                memset(transmit_buffer, 0, BUFFER_SIZE);
                sprintf(transmit_buffer, "%s", receive_buffer);
                transmit_nrf((char *) transmit_buffer, strlen(((char *) transmit_buffer)));
            }
            memset((char *) receive_buffer, 0, BUFFER_SIZE);
            packet_received = false;
        }
    }
}

/**
 * This method sets all the chip configurations to what we'd like to use,
 * such as setting the clock speed to 32MHz, configuring the ADC, Timer E1, etc.
 */
void configure() {

    // This code is taken from clock.h
    // Written by Wim Dolman
    OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc |                   // Select frequency range
                   OSC_XOSCSEL_XTAL_16KCLK_gc;                // Select start-up time
    OSC.CTRL |= OSC_XOSCEN_bm;                                // Enable oscillator
    while ( ! (OSC.STATUS & OSC_XOSCRDY_bm) );                // Wait for oscillator is ready

    OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | (OSC_PLLFAC_gm & 2);   // Select PLL source and multipl. factor
    OSC.CTRL |= OSC_PLLEN_bm;                                 // Enable PLL
    while ( ! (OSC.STATUS & OSC_PLLRDY_bm) );                 // Wait for PLL is ready

    CCP = CCP_IOREG_gc;                                       // Security signature to modify clock
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;                            // Select system clock source
    OSC.CTRL &= ~OSC_RC2MEN_bm;                               // Turn off 2MHz internal oscillator
    OSC.CTRL &= ~OSC_RC32MEN_bm;

    PMIC.CTRL |= PMIC_LOLVLEN_bm;                           // Enable interrupts

    PORTA.DIRCLR     = PIN2_bm;                             // We'd like to use pin 2 for our converter
    PORTB.DIRCLR     = PIN0_bm;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;               // Multiplex selection for pin 2
    ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
    ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc;     // Single ended input without gain
    ADCB.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;                // Reference voltage, INTVCC = 3.3V / 1.6 ~ 2.0V
    ADCB.REFCTRL     = ADC_REFSEL_INTVCC_gc;
    ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc;             // Range of number conversion, 185 - 2^14-1
    ADCB.CTRLB       = ADC_RESOLUTION_12BIT_gc;
    ADCA.PRESCALER   = ADC_PRESCALER_DIV512_gc;             // F_CPU / TIMER_PRESCALER -> Speed of conversoin
    ADCB.PRESCALER   = ADC_PRESCALER_DIV512_gc;
    ADCA.CTRLA       = ADC_ENABLE_bm;                       // Turn on the ADC converter
    ADCB.CTRLA       = ADC_ENABLE_bm;

    TCE0.CTRLB    = TC_WGMODE_NORMAL_gc;
    TCE0.CTRLA    = TC_CLKSEL_DIV1024_gc;                   // Clock divisor. For 32MHz and D(1024), it does 31250 loops per second
    TCE0.PER      = F_CPU / (TIMER_PRESCALER * TICK_SPEED) - 1;        // Setup the speed of the TIMER
    TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;                     // No interrupts

    sei();
}

/**
 * Method of reading data from the analog to digital converter
 * @param adc The ADC to read from
 * @return The value retrieved by the adc
 */
void read_adc(ADC_t * adc, uint16_t * dst) {
    adc->CH0.CTRL |= ADC_CH_START_bm;                    // start ADC conversion
    while (!(adc->CH0.INTFLAGS & ADC_CH_CHIF_bm)) ;      // wait until it's ready
    *dst = adc->CH0.RES;
    adc->CH0.INTFLAGS |= ADC_CH_CHIF_bm;                 // reset interrupt flag
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
    nrfSetAutoAck(false);
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
 * Since this is the central_node, it can communicate to every single device.
 * Other devices can only send data to this one.
 */
void load_pipes_nrf() {

    // Open our pipe for writing.
    nrfOpenWritingPipe((uint8_t *) addresses[CENTRAL_DEVICE_IDX]);

    for (int i = 0; i < sizeof(addresses)/sizeof(addresses[0]); i++) {
        nrfOpenReadingPipe(i, (uint8_t *) addresses[i]);
    }
}

/**
* Method of sending packets via the NRF chip.
* @note The max size of the buffer can be 32 bytes
* @param buffer The buffer to be sent
* @param bufferSize The size of the buffer
*/
void transmit_nrf(char * buffer, uint16_t bufferSize) {
    cli();                                   // Disable interrupts
    nrfStopListening();                      // Stop listening
    nrfWrite((uint8_t *) buffer, bufferSize);   // Write to the targetted device
    nrfStartListening();                     // Start listening for input again
    sei();                                   // re-enable interrupts
}

/**
 * The TIMER interrupt vector. This one is triggered TICK_SPEED times
 * The function puts the lux value in the transmit buffer, then triggers
 * the process state to send the buffer to the NRF chip in the main loop.
 */
ISR(TCE0_OVF_vect) {
    timer_triggered = true;
}

/**
 * The interrupt vector for receiving NRF signals.
 * Once a packet is received, we set the 'packet_received' flag to true,
 * which is then in turn parsed in the main loop.
 */
ISR(PORTF_INT0_vect) {
    uint8_t  tx_ds, max_rt, rx_dr;
    uint8_t  packet_length;

    nrfWhatHappened(&tx_ds, &max_rt, &rx_dr);

    if ( rx_dr ) {
        packet_length = nrfGetDynamicPayloadSize();
        nrfRead((uint8_t *) receive_buffer, packet_length );
        receive_buffer[packet_length] = '\0';
        packet_received = true;
    }
}
