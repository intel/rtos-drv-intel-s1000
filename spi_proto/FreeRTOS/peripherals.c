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

#include <string.h>
#include "peripherals.h"
#include "spi_lib_error.h"

static const char *LOG_TAG = "Freertos-specific";

#define ESP_INTR_FLAG_DEFAULT	(0)
static void IRAM_ATTR cs_low(spi_transaction_t* trans) {
	GPIO_OUTPUT_SET((int )trans->user, 0);
}
static void IRAM_ATTR cs_high(spi_transaction_t* trans) {
	GPIO_OUTPUT_SET((int )trans->user, 1);
}

static int free_spi_device(spi_device_t* spi_dev)
{
	int res = spi_bus_free(HSPI_HOST);
	RET_ON_ERROR_LOG(res , PROTO_SPI_CONFIGURE_FAILED, "Free spi device failed");
	return PROTO_SUCCESS;
}

#define DISABLE_HARDWARE_CS (-1)
int spi_dev_config(spi_device_t* spi_device, uint32_t clock) {
	int res;
	spi_device_settings_t* settings = spi_device->settings;
	spi_device_interface_config_t* slv_cfg;

	/* Slave configuration. */
	spi_device_interface_config_t dev_cfg = { .clock_speed_hz = clock, .mode = 3, .spics_io_num = DISABLE_HARDWARE_CS,
			.queue_size = 7, .flags = 0, .pre_cb = cs_low, .post_cb = cs_high, };

	slv_cfg = &dev_cfg;
	spi_bus_config_t bus_cfg = { .miso_io_num = settings->miso_pin, .mosi_io_num = settings->mosi_pin,
			.sclk_io_num = settings->clock_pin, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 0, };

	/* Bus configuration. */
	res = spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);
	RET_ON_ERROR_LOG(res, PROTO_SPI_CONFIGURE_FAILED, "Spi bus init failed");

	/* Slave configuration. */
	res = spi_bus_add_device(HSPI_HOST, slv_cfg, &spi_device->dev);
	RET_ON_ERROR_LOG(res , PROTO_SPI_CONFIGURE_FAILED, "Spi bus add device failed");

	return PROTO_SUCCESS;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
#ifdef TX_TASK
	EventGroupHandle_t group = (EventGroupHandle_t)arg;
	EventBits_t* ev_bits = (EventBits_t*)group;
	*ev_bits |= IRQ_STATUS;
	BaseType_t xHigherPriorityTaskWoken = false;
	xEventGroupSetBitsFromISR(group, IRQ_EVENT, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken)
		portYIELD_FROM_ISR();
#else
	isr_c* isr = (isr_c*)arg;
	EventGroupHandle_t group = isr->group;
	if(xEventGroupGetBitsFromISR(group) & ISR_FORWARD)
	{
		EventBits_t* ev_bits = (EventBits_t*)group;
		*ev_bits |= IRQ_EVENT;
		isr->irq_handler->isr_callback(isr->irq_handler->ctx);
	}
	else
	{
		BaseType_t xHigherPriorityTaskWoken = false;
		xEventGroupSetBitsFromISR(group, IRQ_EVENT, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
			portYIELD_FROM_ISR();
	}
#endif
}

#ifdef TX_TASK
static int gpio_intr_config(spi_device_t* spi_device, EventGroupHandle_t isr_ctx)
#else
static int gpio_intr_config(spi_device_t* spi_device, isr_c* isr_ctx)
#endif
{
	/* GPIO configuration. */
	int res;
	gpio_config_t gpio_cfg = { 0 };
	memset(&gpio_cfg, 0, sizeof(gpio_cfg));

	spi_device_settings_t* settings = spi_device->settings;
	gpio_cfg.intr_type = GPIO_PIN_INTR_DISABLE; /* No interrupt. */
	gpio_cfg.pin_bit_mask = (1ULL << settings->reset_pin); /* Pin number. */
	gpio_cfg.mode = GPIO_MODE_OUTPUT; /* Digital output. */
	gpio_cfg.pull_up_en = 0; /* Pull up disabled. */
	gpio_cfg.pull_down_en = 0; /* Pull down disabled. */
	res = gpio_config(&gpio_cfg);
	RET_ON_ERROR_LOG(res , PROTO_GPIO_CONFIGURE_FAILED, "Failed configuring reset pin");

	gpio_cfg.intr_type = GPIO_PIN_INTR_DISABLE; /* No interrupt. */
	gpio_cfg.pin_bit_mask = (1ULL << settings->cs_pin); /* Pin number. */
	gpio_cfg.mode = GPIO_MODE_OUTPUT; /* Digital output. */
	gpio_cfg.pull_up_en = 1; /* Pull up enabled. */
	gpio_cfg.pull_down_en = 0; /* Pull down disabled. */
	res = gpio_config(&gpio_cfg);
	RET_ON_ERROR_LOG(res, PROTO_GPIO_CONFIGURE_FAILED, "Failed configuring cs pin");

#ifdef PROBE_PIN
	gpio_cfg.intr_type = GPIO_PIN_INTR_DISABLE; /* No interrupt. */
	gpio_cfg.pin_bit_mask = (1ULL << IPC_SPI_CS_HW); /* Pin number. */
	gpio_cfg.mode = GPIO_MODE_OUTPUT; /* Digital output. */
	gpio_cfg.pull_up_en = 0; /* Pull up disabled. */
	gpio_cfg.pull_down_en = 0; /* Pull down disabled. */
	gpio_config(&gpio_cfg);
	gpio_set_level(IPC_SPI_CS_HW, 0);
#endif

	gpio_set_level(settings->cs_pin, 1);

	gpio_cfg.intr_type = GPIO_PIN_INTR_POSEDGE; /* GPIO on rising edge. */
	gpio_cfg.pin_bit_mask = (1ULL << settings->irq_pin); /* Pin number. */
	gpio_cfg.mode = GPIO_MODE_INPUT; /* Digital input. */
	//gpio_cfg.pull_up_en = 0;			/* Pull up disabled. */
	gpio_cfg.pull_up_en = 1; /* Pull up enabled. */
	gpio_cfg.pull_down_en = 0; /* Pull down disabled. */
	res = gpio_config(&gpio_cfg);
	RET_ON_ERROR_LOG(res, PROTO_GPIO_CONFIGURE_FAILED, "Failed configuring irq pin");

	if (settings->wake_pin != -1) {
			gpio_cfg.intr_type = GPIO_PIN_INTR_DISABLE;
			gpio_cfg.pin_bit_mask = (1ULL << settings->wake_pin);
			gpio_cfg.mode = GPIO_MODE_OUTPUT;
			gpio_cfg.pull_up_en = 0;
			gpio_cfg.pull_down_en = 0;
			gpio_config(&gpio_cfg);
			res = gpio_config(&gpio_cfg);
			RET_ON_ERROR_LOG(res, PROTO_GPIO_CONFIGURE_FAILED, "Failed configuring wake pin");
		}

	/* Interrupt installation. */
	res = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	RET_ON_ERROR_LOG(res, PROTO_GPIO_CONFIGURE_FAILED, "Failed installing isr service");
	res = gpio_isr_handler_add(settings->irq_pin, gpio_isr_handler, isr_ctx);
	RET_ON_ERROR_LOG(res, PROTO_GPIO_CONFIGURE_FAILED, "Failed adding isr handler");
	return PROTO_SUCCESS;
}


int peripherals_init(spi_device_t* spi_dev, spi_device_settings_t* settings, EventGroupHandle_t event_handle)
{
	int res;
	spi_dev->settings = settings;
	res = spi_dev_config(spi_dev, settings->itnitial_clock);
	RET_ON_ERROR(res);
#ifdef TX_TASK
	res = gpio_intr_config(spi_dev, event_handle);
#else
	spi_dev->isr.irq_handler = &settings->isr_handler;
	spi_dev->isr.group = event_handle;
	res = gpio_intr_config(spi_dev, &spi_dev->isr);
#endif
	RET_ON_ERROR(res);
	return PROTO_SUCCESS;
}

int peripherals_close(spi_device_t* spi_dev) {
	if (gpio_isr_handler_remove(spi_dev->settings->irq_pin))
		LOGE("Failed to remove gpio handler");

	gpio_uninstall_isr_service(ESP_INTR_FLAG_DEFAULT);

	return free_spi_device(spi_dev);
}

int spi_device_transfer(spi_device_t* spi_dev , uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length)
{
	spi_transaction_t t = {
			.length = length * 8,
			.tx_buffer = tx_buffer,
			.rx_buffer = rx_buffer,
			.flags = 0,
			.user = (void*)spi_dev->settings->cs_pin,
	};
	int res = spi_device_transmit(spi_dev->dev , &t);
	RET_ON_ERROR_LOG(res, PROTO_SPI_TRANSFER_FAILED,
			"Spi transfer failed spi_device: %p, tx_buffer: %p, rx_buffer: %p, length: %d", spi_dev, tx_buffer, rx_buffer, length);

	return PROTO_SUCCESS;
}

int spi_device_set_clock(spi_device_t* spi_dev, uint32_t clock) {

	int res = free_spi_device(spi_dev);
	RET_ON_ERROR(res);
	res = spi_dev_config(spi_dev, clock);
	RET_ON_ERROR(res);
	return PROTO_SUCCESS;
}

int irq_get_level(spi_device_settings_t* spi_settings)
{
	return gpio_get_level(spi_settings->irq_pin);
}
void irq_set_rising_edge(spi_device_settings_t* spi_settings)
{
	gpio_set_intr_type(spi_settings->irq_pin, GPIO_INTR_POSEDGE);
}
void irq_set_falling_edge(spi_device_settings_t* spi_settings)
{
	gpio_set_intr_type(spi_settings->irq_pin, GPIO_INTR_NEGEDGE);
}

