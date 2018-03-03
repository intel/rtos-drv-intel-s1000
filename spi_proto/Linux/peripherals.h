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

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "spi_settings.h"

#define SPI_MISO (0)
#define SPI_MOSI (0)
#define SPI_CLK (0)
#define SPI_CS_HW (0)
#define GPIO_IRQ (27)
#define GPIO_RST (22)
#define GPIO_WAKE (23)

#define MAX_SPI_BUFF_LEN 4096

typedef struct {
	int spi_dev;
	spi_device_settings_t* settings;
} spi_device_t;

typedef spi_device_t SPI_DEVICE_T;
typedef group_event_t* EVENT_GROUP_T;

int peripherals_init(spi_device_t* spi_dev, spi_device_settings_t* settings, group_event_t* event);
int peripherals_close(spi_device_t* spi_dev);
int spi_device_transfer(spi_device_t* spi_dev, uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length);
int spi_device_set_clock(spi_device_t* spi_dev, uint32_t clock);

int irq_get_level(spi_device_settings_t* spi_settings);
void irq_set_rising_edge(spi_device_settings_t* spi_settings);
void irq_set_falling_edge(spi_device_settings_t* spi_settings);
void gpio_set_level(int pin, int level);
int gpio_get_level(int pin);

#endif
