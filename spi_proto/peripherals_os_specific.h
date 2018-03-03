/*
 * BSD LICENSE
 *
 * Copyright(c) 2019 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in
 *        the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of Intel Corporation nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pal_os_specific.h"

#ifndef PERIPHERALS_OS_SPECIFIC_H
#define PERIPHERALS_OS_SPECIFIC_H
/*
 * 	Following steps are required to start:
 * 	1). Define pin connection as requested by hardware. If any of pins ex (miso/mosi/clk) will not be used during configuration may be set to 0.
		#define SPI_MISO (num)
		#define SPI_MOSI (num)
		#define SPI_CLK (num)
		#define SPI_CS_HW (num)
		#define GPIO_IRQ (num)
		#define GPIO_RST (num)
		#define GPIO_WAKE (num)

		Values will be written to:

		typedef struct {
			uint32_t miso_pin;
			uint32_t mosi_pin;
			uint32_t clock_pin;
			uint32_t cs_pin;
			uint32_t irq_pin;
			uint32_t reset_pin;
			uint32_t itnitial_clock;
			uint32_t wake_pin;
			#ifndef TX_TASK
			irq_handler_t isr_handler; - Handler which wakes up application from idle state. Triggered by notification (IRQ pin) from FW.
			#endif
		} spi_device_settings_t;



	2). #define SAFE_IRQ_ARMING_DURATION (num) - Used ONLY in multithreaded (TX_TASK) and protocol version without wake pin.

	3). #define MAX_SPI_BUFF_LEN (num) - Define max buffer length which should be used during spi transaction.
										 May be set to the same value as ipc buffer size ex (64/4096).


	4) typedef spi_device_t SPI_DEVICE_T - Define own structure. Should keep information which will be required to successfully operate

	   typedef struct
	   {

       } spi_device_t;

	5) Function should configure spi device and gpio pins from given parameters in spi_device_settings_t
	   Implementation should contain ability to set IRQ_EVENT on event_handle from isr handler or explicity on EventGroupWaitBits call.
	   IMPORTANT: Applies only to Single threaded mode. To accomlish single thread mode, isr handler has to perform same action as for multithreaded.
	   Additionally has to always check if ISR_FORWARD bit is set on event_handle and execute callback given on initialization.
	   ISR_FORWARD means that both FW and application went idle and detection of rising edge while ISR_FORWARD is set means that FW has notification
	   pending and we should take action.

	   int peripherals_init(spi_device_t* spi_dev, spi_device_settings_t* settings, EventGroupHandle_t event_handle);


	6) Release resources
		int peripherals_close(spi_device_t* spi_dev)

	7) Function operate directly on spi device, should perform transaction to tx/rx buffers
		int spi_device_transfer(spi_device_t* spi_dev , uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length);

	8) Function should implement possibility to reconfigure spi clock in runtime. May be left blank
		int spi_device_set_clock(spi_device_t* spi_dev, uint32_t clock);

	9) Function should perform operation on irq pin given in spi_settings
		int irq_get_level(spi_device_settings_t* spi_settings);
		void irq_set_rising_edge(spi_device_settings_t* spi_settings);
		void irq_set_falling_edge(spi_device_settings_t* spi_settings);
*/

#ifdef FREERTOS
#include "FreeRTOS/peripherals.h"
#elif defined(LINUX_US)
#include "Linux/peripherals.h"
#endif

#endif
