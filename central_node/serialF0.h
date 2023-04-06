/*!
 *  \file    serialF0.h
 *  \author  Wim Dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *  \date    16-04-2018
 *  \version 1.8
 *
 *  \brief   Serial interface voor HvA-Xmegaboard
 *
 *  \details This serial interface doesn't use the drivers of Atmel
 *           The interface uses two \e non circulair buffers for sending and 
 *           receiving the data.
 *           It is based om md_serial.c from J.D.Bakker.
 *
 *           It is a serial interface for the HvA-Xmegaboard (Version 2) with a
 *           Xmega256a3u and Xmega32a4u for programming and the serial interface.
 *           You can use the standard printf, putchar, puts, scanf, getchar, ...
 *           functions.
 *
 *           The baud rate is 115200
 */
 
#ifndef SERIALF0_H_
#define SERIALF0_H_

#include <stdio.h>
#include <avr/io.h>

#define TXBUF_DEPTH_F0    512      //!<  size of transmit buffer
#define RXBUF_DEPTH_F0    512      //!<  size of receive buffer

#define UART_NO_DATA      0x0100                      //!< Macro UART_NO_DATA is returned by uart_getc when no data is present
#define clear_screen()    printf("\e[H\e[2J\e[3J");   //!< Macro to reset and clear the terminal

void      configure_usartf0(uint32_t f_cpu, uint32_t baud);
uint16_t  ReadByte(void);
void      WriteByte(uint8_t data);
uint8_t   CanRead_F0(void);
uint8_t   CanWrite_F0(void);

#endif // SERIALF0_H_ 
