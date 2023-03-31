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

#define ADC_MIN (200)
#define ADC_MAX (4095)

#define MV_TO_V   (0.001F)
#define V_IN      (3.3F)
#define ADC_REF_V (1.6F)

// The humidity is logarithmically linear. This value is calculated from the data from the datasheet
// The resistance has to be high to create an accurate voltage divider.
// The HUMID_GROWTH_FACTOR is calculated by
#define HUMID_RESISTANCE           (5600000UL)    // Resistor value
#define HUMID_R_TEMP_GROWTH_FACTOR (0.92302818069F)  // Average growth factor of temperature.
#define HUMID_GROWTH_FACTOR        (-22.9531693F)  // ΔH / ΔLOG -> ΔT = 1, ΔLOG ≈ -0.04356696835
#define HUMID_OFFSET_VALUE         (168.010926F)  // Calculated from sheet. Hoff = Δ - L. * ΔH / ΔLOG

#define TICK_SPEED 1 // Frequency at which the timer interrupt is called.

#define BUFFER_SIZE 32 // Max char buffer size

#define CENTRAL_DEVICE_IDX 0

#define LED_PER 999

#define ADC_TO_MVOLT(res, ref) ((3200 / (ref) / (ADC_MAX - ADC_MIN)) * ((res) - ADC_MIN))
#define MVOLT_TO_C(v)          ((8.194f - sqrtf(67.141636f + 4 * 0.00262f * (1324.0f - (v)))) / (2 * -0.00262f) + 30.0f)
#define HEAT_INDEX(T, RH) (-8.7847 + 1.6114*T + 2.3385*RH - 0.1461*T*RH - 0.01231*T*T - 0.0162*RH*RH + 0.0022*T*T*RH + 0.0007*T*RH*RH - .00000352*T*T*RH*RH)


const char *addresses[] = {"1_dev", "2_dev", "3_dev",
                           "4_dev", "5_dev", "6_dev"};

volatile uint8_t receive_buffer[BUFFER_SIZE];
volatile bool packet_received = false;
volatile bool timer_triggered = false;

// Configuration. Can change later on.
typedef enum NRF_CONFIG {
    NRF_CHANNEL    = 6,                         // Frequency channel (2400 + CH)MHz
    NRF_DATA_SPEED = NRF_RF_SETUP_RF_DR_250K_gc,// Data transfer speed
    NRF_CRC        = NRF_CONFIG_CRC_8_gc,       // Cyclic redundancy check length
} nrf_cfg;

void Configure();
void NRFInit();
void NRFLoadPipes();
void NRFSendPacket(uint8_t* buffer, uint16_t bufferSize);
void ReadADC(ADC_t * adc, uint16_t * dst);

// here we go again
int main(void) {

    Configure();
    USARTInit(F_CPU, UARTF0_BAUD);
    NRFInit();

    PORTD.DIRSET = PIN5_bm;
    TCD1.CTRLA = TC_CLKSEL_DIV64_gc;
    TCD1.CTRLB = TC0_CCBEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.PER   = LED_PER - 1;
    TCD1.CCB   = LED_PER / 2;

    bool phase = false;
    char transmit_buffer[BUFFER_SIZE];

    uint16_t temperature_res, humidity_res;
    int8_t temperature, humidity;
    float mV_temperature, V_humidity, R_humid;

    printf("\e[H\e[2J\e[3J");


    while (true) {
        if (timer_triggered) {
            phase = !phase;

            // H = 165.71594 + log10(((R_humidity) * 0.923778408^T) / -0.04356696835

            ReadADC(&ADCA, &temperature_res);                       // Read ADC value from temperature sensor
            ReadADC(&ADCB, &humidity_res);                          // Read ADC value from humidity sensor
            mV_temperature = ADC_TO_MVOLT(temperature_res, ADC_REF_V);       // Convert ADC to mV
            V_humidity     = ADC_TO_MVOLT(humidity_res, 1.0f) * MV_TO_V;    // Convert ADC to mV and then to V
            temperature    = (int8_t) MVOLT_TO_C  (mV_temperature);

            R_humid = HUMID_RESISTANCE * (V_IN / V_humidity - 1);          // Calculate the resistance going through the DHT20
            humidity = (int8_t) (HUMID_OFFSET_VALUE +
                                  log10f(R_humid * powf(HUMID_R_TEMP_GROWTH_FACTOR, temperature)) * HUMID_GROWTH_FACTOR);

            if (phase)
                sprintf(transmit_buffer, "temp=%d", temperature);
            else
                sprintf(transmit_buffer, "humid=%d", humidity);

            NRFSendPacket((uint8_t *) transmit_buffer, strlen(transmit_buffer));

            printf("U: %.1fV R: %.0fΩ T: %dC H: %d%s Hi: %.1f [%s]      \r",
                   V_humidity,
                   (R_humid), temperature, humidity, "%",
                   HEAT_INDEX(temperature, humidity),
                   receive_buffer);

            timer_triggered = false;
        }
        if (packet_received) {
            packet_received = false;
        }
    }
}

/**
 * Method of reading data from the analog to digital converter
 * @param adc The ADC to read from
 * @return The value retrieved by the adc
 */
void ReadADC(ADC_t * adc, uint16_t * dst) {
    adc->CH0.CTRL |= ADC_CH_START_bm;                    // start ADC conversion
    while (!(adc->CH0.INTFLAGS & ADC_CH_CHIF_bm)) ;      // wait until it's ready
    *dst = adc->CH0.RES;
    adc->CH0.INTFLAGS |= ADC_CH_CHIF_bm;                 // reset interrupt flag
}

/**
 * This method sets all the chip configurations to what we'd like to use,
 * such as setting the clock speed to 32MHz, configuring the ADC, Timer E1, etc.
 */
void Configure() {

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
    ADCA.PRESCALER   = ADC_PRESCALER_DIV512_gc;             // F_CPU / PRESCALER -> Speed of conversoin
    ADCB.PRESCALER   = ADC_PRESCALER_DIV512_gc;
    ADCA.CTRLA       = ADC_ENABLE_bm;                       // Turn on the ADC converter
    ADCB.CTRLA       = ADC_ENABLE_bm;

    TCE0.CTRLB    = TC_WGMODE_NORMAL_gc;
    TCE0.CTRLA    = TC_CLKSEL_DIV1024_gc;                   // Clock divisor. For 32MHz and D(1024), it does 31250 loops per second
    TCE0.PER      = F_CPU / (1024 * TICK_SPEED) - 1;        // Setup the speed of the TIMER
    TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;                     // No interrupts

    sei();
}

/**
 *  This method sets up the configuration for our NRF device.
 *  Individual settings can be changed in the enumerable defined at the top of this file.
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


    nrfSetCRCLength((nrf_config_crc_t) NRF_CRC);
    nrfSetChannel(NRF_CHANNEL);

    // Require acknowledgements
    nrfSetAutoAck(1);
    nrfEnableDynamicPayloads();
    nrfClearInterruptBits();
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
 * Method for loading the pipes for reading and writing.
 * Since this is the central_node, it can communicate to every single device.
 * Other devices can only send data to this one.
 */
void NRFLoadPipes() {

    // Open our pipe for writing.
    nrfOpenWritingPipe((uint8_t *) addresses[CENTRAL_DEVICE_IDX]);

    for (int i = 0; i < sizeof(addresses)/sizeof(addresses[0]); i++) {
        nrfOpenReadingPipe(i, (uint8_t *) addresses[i]);
    }
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
 * Method of sending packets via the NRF chip.
 * @note The max size of the buffer can be 32 bytes
 * @param buffer The buffer to be sent
 * @param bufferSize The size of the buffer
 */
void NRFSendPacket(uint8_t* buffer, uint16_t bufferSize) {
    cli();                                   // Disable interrupts
    nrfStopListening();                      // Stop listening
    nrfWrite(buffer, bufferSize);   // Write to the targetted device
    nrfStartListening();                     // Start listening for input again
    sei();                                   // re-enable interrupts
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
        nrfRead( (uint8_t *) receive_buffer, packet_length );
        receive_buffer[packet_length] = '\0';
        packet_received = true;
    }
}
