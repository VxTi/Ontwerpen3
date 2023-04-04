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

// The humidity is logarithmically linear. This value is calculated from the data from the datasheet
// The resistance has to be high to create an accurate voltage divider.

// The HUMID_GROWTH_FACTOR is calculated by
#define HUMID_RESISTANCE           (5600000UL)    // Resistor value
#define HUMID_R_TEMP_GROWTH_FACTOR (0.92302818069F)  // Average growth factor of temperature.
#define HUMID_GROWTH_FACTOR        (-22.9531693F)  // ΔH / ΔLOG -> ΔT = 1, ΔLOG ≈ -0.04356696835
#define HUMID_OFFSET_VALUE         (168.010926F)  // Calculated from sheet. Hoff = Δ - L. * ΔH / ΔLOG

#define TICK_SPEED 1 // Frequency at which the timer interrupt is called.

#define ADC_REF_V (1.6F)
#define ADC_MIN   (200)
#define ADC_MAX   (4095)


#define SENSOR_COUNT (3)
#define TIMER_PRESCALER (1024)

#define BUFFER_SIZE (32) // Max char buffer size

#define CENTRAL_DEVICE_IDX (0)

#define ADC_TO_MVOLT(res, ref) ((3200 / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN))

inline void calculate_temperature(uint16_t temperatureRes, int16_t * varDst) {
    *varDst = (int16_t) ((8.194f - sqrtf(67.141636f + 4 * 0.00262f * (1324.0f - ADC_TO_MVOLT(temperatureRes, ADC_REF_V)))) / (2 * -0.00262f) + 30.0f);
}
inline void calculate_humidity(uint16_t adcRes, uint16_t temperature, uint8_t * varDst) {
    float R_humid = HUMID_RESISTANCE * ((float)(ADC_MAX - ADC_MIN) / (float)(adcRes - ADC_MIN) - 1);
    *varDst = (uint8_t) (HUMID_OFFSET_VALUE +
                        log10f(R_humid * powf(HUMID_R_TEMP_GROWTH_FACTOR, temperature)) * HUMID_GROWTH_FACTOR);
}

const char *addresses[] = {"1_dev", "2_dev", "3_dev",
                           "4_dev", "5_dev", "6_dev"};

volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile bool packet_received = false;
volatile bool timer_triggered = false;

/**
 * Config struct for NRF settings, which is loaded in the main function
 * using a 'initialize_nrf(struct)' call
 */
typedef struct {
    NRF_SETUP_AW_ARD_t NRF_RETRY_DELAY;
    NRF_SETUP_AW_ARC_t NRF_RETRY_ATTEMPTS;
    nrf_rf_setup_pwr_t NRF_POWER_LEVEL;
    nrf_rf_setup_rf_dr_t NRF_DATA_SPEED;
    nrf_config_crc_t NRF_CRC_LENGTH;
    uint8_t NRF_CHANNEL;
    uint8_t NRF_AUTO_ACK;
} nrf_settings;

// Struct containing the current NRF settings.
const nrf_settings nrfSettings = {
        .NRF_AUTO_ACK       = true,
        .NRF_POWER_LEVEL    = NRF_RF_SETUP_PWR_6DBM_gc,
        .NRF_RETRY_DELAY    = NRF_SETUP_ARD_1000US_gc,
        .NRF_RETRY_ATTEMPTS = NRF_SETUP_ARC_8RETRANSMIT_gc,
        .NRF_DATA_SPEED     = NRF_RF_SETUP_RF_DR_250K_gc,
        .NRF_CRC_LENGTH     = NRF_CONFIG_CRC_8_gc,
        .NRF_CHANNEL        = 6
};

void load_pipes_nrf();
void configure_nrf (nrf_settings settings);
void transmit_nrf  (char * buffer, uint16_t bufferSize);
void str_to_int    (const char * input, uint16_t * dst);
void read_adc      (ADC_t * adc, uint16_t * dst);
void configure     ();

// here we go again
int main(void) {

    configure();
    USARTInit(F_CPU, UARTF0_BAUD);
    configure_nrf(nrfSettings);

    char transmit_buffer[BUFFER_SIZE];

    uint16_t temperature_res, humidity_res, co2_res = 0;
    int16_t temperature;
    uint8_t humidity;

    uint8_t phase = 0;

    while (true) {
        TIMER:
        if (timer_triggered) {

            phase = (phase + 1) % SENSOR_COUNT;

            read_adc(&ADCA, &temperature_res);                       // Read ADC value from temperature sensor
            read_adc(&ADCB, &humidity_res);                          // Read ADC value from humidity sensor
            calculate_temperature(temperature_res, &temperature);
            calculate_humidity(humidity_res, temperature, &humidity);

            if (phase > 1 && co2_res == 0)
                goto TIMER;

            sprintf((char *) transmit_buffer, "%s=%d",
                    phase == 0 ? "temp" : phase == 1 ? "humid" : "co2_res",
                    phase == 0 ? temperature : phase == 1 ? humidity : co2_res);
            transmit_nrf((char *) transmit_buffer, strlen((char *) transmit_buffer));

            timer_triggered = false;
        }

        if (packet_received) {
            if (!strncmp((char *) rx_buffer, "co2_res=", 8))
                str_to_int((char *) &rx_buffer[8], &co2_res);


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

    PORTD.DIRSET = PIN5_bm;
    TCD1.CTRLA = TC_CLKSEL_DIV64_gc;
    TCD1.CTRLB = TC0_CCBEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.PER   = LED_PER - 1;
    TCD1.CCB   = LED_PER / 2;

    sei();
}

/**
 * Method for converting a string to an integer number.
 * @param input The input string to convert.
 * @param dst The targeted variable to store the result in.
 */
void str_to_int(const char * input, uint16_t * dst) {
    *dst = 0;
    for (uint16_t i = 0; input[i] != '\0'; i++) {
        if (!(input[i] >= '0' && input[i] <= '9'))
            return;
        *dst = *dst * 10 + input[i] - '0';
    }
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
void configure_nrf(nrf_settings settings) {
    // Set up the transmission
    nrfspiInit();
    nrfBegin();
    // Retry after 1 MS and retransmit if failed
    nrfSetRetries(settings.NRF_RETRY_DELAY, settings.NRF_RETRY_ATTEMPTS);
    // Set it to -6dBm for high power amplification
    nrfSetPALevel(settings.NRF_POWER_LEVEL);

    // Data transmission rate, set at 250Kb
    nrfSetDataRate(settings.NRF_DATA_SPEED);

    // Enable cyclic redundancy check. This checks whether the packet is corrupt or not.
    // If not, parse it, else retry
    nrfSetCRCLength(settings.NRF_CRC_LENGTH);

    // Set the channel to what we've previously defined.
    // The band frequency is defined as f = (2400 + CH) MHz
    // For channel 6, this means 2,406 MHz
    nrfSetChannel(settings.NRF_CHANNEL);

    // Require acknowledgements
    nrfSetAutoAck(settings.NRF_AUTO_ACK);
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
    printf("%s\n", buffer);
    cli();                                   // Disable interrupts
    nrfStopListening();                      // Stop listening
    nrfWrite(buffer, bufferSize);   // Write to the targetted device
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
        nrfRead((uint8_t *) rx_buffer, packet_length );
        rx_buffer[packet_length] = '\0';
        packet_received = true;
    }
}
