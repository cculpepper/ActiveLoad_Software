/**************************************************************************/
/*!
 @file     main.c
 
 @section LICENSE
 
 Software License Agreement (BSD License)
 
 Copyright (c) 2013, K. Townsend (microBuilder.eu)
 All rights reserved.
 
 Modified James Coxon (jacoxon@googlemail.com) 2014-2015
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the copyright holders nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**************************************************************************/
//Node settings have been moved seperate file "settings.h"
/*#include "settings.h"*/

#include <stdio.h>
#include "LPC8xx.h"

#include "uart.h"

#include "adc.h"

#include "zombie.h"

char data_out_temp[100 + 1];

/**
 * Setup all pins in the switch matrix of the LPC810
 */
void configurePins()
{
	/* Enable SWM clock */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);

	/* Pin Assign 8 bit Configuration */
	/* U0_TXD */
	LPC_SWM->PINASSIGN0 = 0xffffff00UL;
	/* SPI0_SCK */
	LPC_SWM->PINASSIGN3 = 0x02ffffffUL;
	/* SPI0_MOSI */
	/* SPI0_MISO */
	/* SPI0_SSEL */
	LPC_SWM->PINASSIGN4 = 0xff050304UL;

	/* Pin Assign 1 bit Configuration */
	/* ACMP_I2 */
	LPC_SWM->PINENABLE0 = 0xfffffffdUL;

}

#ifdef SERIAL_IN
/**
 * Checks for incoming serial data which will be send by radio. This function
 * also checks the buffers length to avoid a stackoverflow at the radio FIFO.
 */
inline void checkTxBuffer(void)
{
	uint8_t i;

	if (UART0_available() > 0) {
		for (i = 0; i < serialBuffer_write; i++) {

			if (serialBuffer[i] == '\r' || serialBuffer[i] == '\n' || strlen(data_out_temp) >= 32) {	// Transmit data from buffer

#ifdef DEBUG
				if (strlen(data_out_temp) >= MAX_TX_CHARS)
					printf("max. buffer exceeded\r\n");
#endif

				// Transmit data
				incrementPacketCount();

				uint8_t n;
				data_temp[0] = '\0';

				n = sprintf(data_temp, "%d%c%s[%s]",
					    NUM_REPEATS, data_count,
					    data_out_temp, NODE_ID);
				transmitData(n);

				data_out_temp[0] = '\0';	// Flush buffer

			} else {
				sprintf(data_out_temp, "%s%c", data_out_temp, serialBuffer[i]);	// Read from serial buffer
			}

		}
		serialBuffer_write = 0;
	}
}
#endif

int main(void)
{
#ifdef BrownOut
	LPC_SYSCON->BODCTRL = 0x11;	//Should be set to Level 1 (Assertion 2.3V, De-assertion 2.4V) reset
#endif

#ifdef GPS
	// Initialise the UART0 block for printf output
	uart0Init(9600);
#elif defined(GATEWAY) || defined(DEBUG)
	// Initialise the UART0 block for printf output
	uart0Init(115200);
#endif

	// Configure the multi-rate timer for 1ms ticks

	/* Enable AHB clock to the Switch Matrix , UART0 , GPIO , IOCON, MRT , ACMP */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7) | (1 << 14)
	    /*| (1 << 6) */
	    /* | (1 << 18) */
	    | (1 << 10) | (1 << 19);

	// Configure the switch matrix (setup pins for UART0 and SPI)
	configurePins();

#ifdef DEBUG
	printf("Node Booted\r\n");
#endif

#ifdef ADC
	//Read ADC
	adc_result = read_adc2();
#endif

#ifdef BrownOut
	adc_result = acmpVccEstimate();
	// Before transmitting if the input V is too low we could sleep again
	if (adc_result < 3100) {
	}
#endif

#ifdef DEBUG
	printf("ADC: %d\r\n", adc_result);

#endif

}
