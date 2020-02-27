/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * comm_uart.c
 *
 *  Created on: 17 aug 2015
 *      Author: benjamin
 */

#include "comm_uart.h"
// #include "ch.h"
// #include "hal.h"
#include "bldc_interface_uart.h"

#include <string.h>



// Private functions
static void send_packet(unsigned char *data, unsigned int len);



// Variables
static uint8_t serial_rx_buffer[1024];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

;



/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void send_packet(unsigned char *data, unsigned int len) {
	// if (len > (PACKET_MAX_PL_LEN + 5)) {
	// 	return;
	// }

    // // write(fd, (void *)d, len);


	// // Wait for the previous transmission to finish.
	// while (UART_DEV.txstate == UART_TX_ACTIVE) {
	// 	chThdSleep(1);
	// }

	// // Copy this data to a new buffer in case the provided one is re-used
	// // after this function returns.
	// static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	// memcpy(buffer, data, len);

	// // Send the data over UART
	// uartStartSend(&UART_DEV, len, buffer);
}

/**
 * This thread is only for calling the timer function once
 * per millisecond. Can also be implementer using interrupts
 * if no RTOS is available.
 */
static THD_FUNCTION(timer_thread, arg) {
	(void)arg;
	chRegSetThreadName("packet timer");

	for(;;) {
		bldc_interface_uart_run_timer();
		chThdSleepMilliseconds(1);
	}
}

void comm_uart_init(void) {


	// Initialize the bldc interface and provide a send function
	bldc_interface_uart_init(send_packet);

}
