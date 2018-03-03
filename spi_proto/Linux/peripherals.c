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

#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <sys/poll.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include <stdarg.h>
#include "pal.h"
#include "peripherals.h"

static const char* LOG_TAG = "Linux_Specific:";

const char* GPIO_PATH = "/sys/class/gpio/";

static void writeline(const char* file_name, const char* value, int length) {
	int fd = open(file_name, O_WRONLY);
	if (fd < 0) {
		LOGE("Coulnd not opern file %s", file_name);
	}
	int n;
	if ((n = write(fd, value, length)) < 0 && errno != EINVAL) {
		LOGE("Coulnd not write file %s, length %d, value %p ,errno %s %d", file_name, length, value, strerror(errno), errno);
	}
	close(fd);
}

void concat(char* buf, const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsprintf(buf, format, args);
	va_end(args);
}

static void gpio_reexport(int pin) {
	size_t pins_s = 10;
	char pins_buf[pins_s];
	char path_buf[100];
	const char* gpio_export = "export";
	const char* gpio_unexport = "unexport";

	concat(pins_buf, "%d", pin);
	concat(path_buf, "%s%s", GPIO_PATH, gpio_unexport);
	writeline(path_buf, pins_buf, pins_s);
	concat(path_buf, "%s%s", GPIO_PATH, gpio_export);
	writeline(path_buf, pins_buf, pins_s);
}

static void configure_gpio(int pin, const char* functionality, const char* value, int size) {
	char path_buf[100];
	concat(path_buf, "%sgpio%d/%s", GPIO_PATH, pin, functionality);
	writeline(path_buf, value, size);
}

static void gpio_config(spi_device_t* spi_dev) {
	gpio_reexport(spi_dev->settings->irq_pin);
	configure_gpio(spi_dev->settings->irq_pin, "edge", "rising", 7);
	configure_gpio(spi_dev->settings->irq_pin, "direction", "in", 3);

	gpio_reexport(spi_dev->settings->reset_pin);
	configure_gpio(spi_dev->settings->reset_pin, "direction", "out", 4);
	configure_gpio(spi_dev->settings->reset_pin, "value", "1", 2);

	gpio_reexport(spi_dev->settings->wake_pin);
	configure_gpio(spi_dev->settings->wake_pin, "direction", "out", 4);
	configure_gpio(spi_dev->settings->wake_pin, "value", "1", 2);
}

static void configure_spi(spi_device_t* spi_dev) {
	int spi = open("//dev//spidev0.0", O_RDWR);
	spi_dev->spi_dev = spi;
	uint8_t bits_per_word = 8;
	uint32_t spi_baud_ = 12600000;
	uint8_t mode = SPI_MODE_3; // | SPI_LSB_FIRST; //SPI_NO_CS |  SPI_LSB_FIRST;
	if (ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, &spi_baud_) == -1) {
		LOGE("SPI_IOC_WR_MAX_SPEED_HZ -1\n");
	}

	if (ioctl(spi, SPI_IOC_RD_MAX_SPEED_HZ, &spi_baud_) == -1) {
		LOGE("SPI_IOC_RD_MAX_SPEED_HZ -1");
	}

	if (ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) == -1) {
		LOGE("SPI_IOC_WR_BITS_PER_WORD -1");
	}
	if (ioctl(spi, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) == -1) {
		LOGE("SPI_IOC_RD_BITS_PER_WORD -1");
	}
	if (ioctl(spi, SPI_IOC_WR_MODE, &mode) == -1) {
		LOGE("SPI_IOC_WR_MODE -1");
	}
	if (ioctl(spi, SPI_IOC_RD_MODE, &mode) == -1) {
		LOGE("SPI_IOC_RD_MODE -1");
	}
}

static void register_event(group_event_t* group_event, int fd, uint32_t events) {
	struct epoll_event evnt = { 0 };
	evnt.data.fd = fd;
	evnt.events = events;
	if (epoll_ctl(group_event->epoll_instance, EPOLL_CTL_ADD, fd, &evnt)) {
		LOGE("nOT SUCCESFULL APPENDING %s", strerror(errno));
	}
}

static void epoll_flush(group_event_t* group_event) {
	int c = 0;
	struct epoll_event tx_evs[2] = { 0 };
	do {
		epoll_wait(group_event->epoll_instance, tx_evs, 2, 100);
	} while (++c < 2);
}

int peripherals_init(spi_device_t* spi_dev, spi_device_settings_t* settings, group_event_t* group_event) {
	spi_dev->settings = settings;
	gpio_config(spi_dev);
	configure_spi(spi_dev);
	int event_fd = eventfd(0, EFD_NONBLOCK);
	register_event(group_event, event_fd, POLLIN | EPOLLET);
	group_event->spi_proto_flags = event_fd;
	char path[100];
	concat(path, "%sgpio%d/%s", GPIO_PATH, settings->irq_pin, "value");
	group_event->irq_fd = open(path, O_RDONLY);
	register_event(group_event, group_event->irq_fd, EPOLLPRI | EPOLLET);
	epoll_flush(group_event);
	return 0;
}

int peripherals_close(spi_device_t* spi_dev) {
	return 0;
}

int irq_get_level(spi_device_settings_t* spi_settings) {
	char buf = 0;
	int n;
	off_t ns;
	if ((ns = lseek(spi_settings->irq_pin, 0, SEEK_SET)) == (off_t) - 1)
		LOGE("gpio lseek failed %ld %d %s", ns, errno, strerror(errno));
	if ((n = read(spi_settings->irq_pin, &buf, 1)) < 0)
		LOGE("gpio read failed %s", strerror(errno));

	return buf;
}
void irq_set_rising_edge(spi_device_settings_t* spi_settings) {
	configure_gpio(spi_settings->irq_pin, "edge", "rising", 7);
}
void irq_set_falling_edge(spi_device_settings_t* spi_settings) {
	configure_gpio(spi_settings->irq_pin, "edge", "falling", 8);
}
void gpio_set_level(int pin, int level) {
	if (level == 0)
		configure_gpio(pin, "value", "0", 2);
	else
		configure_gpio(pin, "value", "1", 2);
}

int gpio_get_level(int pin) {

}

int spi_device_destroy(void* spi_dev) {
	return 0;
}

int spi_device_transfer(spi_device_t* spi_dev, uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length) {
	struct spi_ioc_transfer spi_transfer = { 0 };
	spi_transfer.speed_hz = spi_dev->settings->itnitial_clock;
	spi_transfer.bits_per_word = 8;
	spi_transfer.tx_buf = (__u64 ) tx_buffer;
	spi_transfer.rx_buf = (__u64 ) rx_buffer;
	spi_transfer.len = (__u32 ) length;
	int r = ioctl(spi_dev->spi_dev, SPI_IOC_MESSAGE(1), &spi_transfer);
	if (r < 0) {
		LOGE("ioctl error %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

int spi_device_set_clock(spi_device_t* spi_dev, uint32_t clock) {
	return 0;
}

