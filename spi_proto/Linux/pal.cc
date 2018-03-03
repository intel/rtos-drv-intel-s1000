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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <chrono>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <sys/poll.h>
#include "sys/msg.h"
#include "sys/ipc.h"
#include "sys/types.h"
#include "errno.h"
#include "pal.h"
#include "spi_settings.h"
#include "spi_lib_error.h"

static const char* LOG_TAG = "Pal_Linux";

#define SET_TIME_SEPEC \
    int val;                                                    \
    if(val = clock_gettime(CLOCK_REALTIME, &time_spec))         \
	    RET_ON_PROTO_ERROR_LOG(val, "Failed to retrieve time"); \
                                                                \
	time_spec.tv_nsec += (long) (max_wait % 1000) * 1000000;    \
    time_spec.tv_sec  += (time_t) (max_wait / 1000) + (time_spec.tv_nsec / 1000000); \
    time_spec.tv_nsec %= 1000000; \

sem_t* CreateSemaphore() {
	sem_t* sem = new sem_t();
	sem_init(sem, 0, 0);
	return sem;
}

int SemaphoreGive(sem_t* sem) {
	if (sem_post(sem))
		LOGE("Give %s", strerror(errno));
	return 0;
}

int SemaphoreTake(sem_t* sem, uint32_t max_wait) {
	struct timespec time_spec = { 0 };

	SET_TIME_SEPEC;

	int value;
	while ((value = sem_timedwait(sem, &time_spec)) == -1 && errno == EINTR)
		continue;

	return value == 0 ? true : false ;
}

void SemaphoreDelete(sem_t* sem) {
	delete sem;
}

pthread_mutex_t* CreateMutex() {
	pthread_mutex_t* mutex = new pthread_mutex_t();
	pthread_mutex_init(mutex, NULL);
	return mutex;
}

int MutexTake(pthread_mutex_t* mutex, uint32_t max_wait) {
	struct timespec time_spec = { 0 };

	SET_TIME_SEPEC;

	return pthread_mutex_timedlock(mutex, &time_spec) == 0 ? true : false;
}

void MutexDelete(pthread_mutex_t* mutex) {
	delete mutex;
}

pthread_spinlock_t CreateSpinkLock() {
	pthread_spinlock_t spin_lock;
	pthread_spin_init(&spin_lock, 0);
	return spin_lock;
}

void TaskDelay(uint32_t time) {
	usleep(time * 1000);
}

void swap_buffer(void *data, uint32_t size) {
	uint32_t* u_data = (uint32_t*) data;
	for (uint32_t* x = u_data; x - u_data < size / 4; ++x)
		*x = htonl(*x);
}

typedef void (*callback_t)(void *);
typedef struct {
	callback_t callback;
	void *params;
} args_t;

void* pthread_callback(void * arg) {
	args_t *ar = (args_t *) arg;
	ar->callback(ar->params);
	return NULL;
}

int CreateTask(void* callback, const char * const pcName, const uint32_t usStackDepth, void * const pvParameters, int uxPriority,
		void * const pxCreatedTask, int xCoreID) {
	pthread_attr_t attr;
	pthread_t* pthread = new pthread_t();
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, usStackDepth);
	args_t *args = new args_t();
	args->callback = (callback_t) callback;
	args->params = pvParameters;
	pthread_create(pthread, &attr, pthread_callback, args);
	return true;
}
void TaskDelete(void* task) {

}

group_event_t* CreateEventGroup() {
	group_event_t* group_event = new group_event_t();
	group_event->epoll_instance = epoll_create1(0);
	if (group_event->epoll_instance < 0) {
		delete group_event;
		return NULL;
	}

	pthread_mutex_init(&group_event->mutex, NULL);
	return group_event;
}

uint32_t EventGroupSetBits(const group_event_t* event, const uint32_t bits_to_set) {
	group_event_t* group_event = (group_event_t*) event;
	MutexTake(&group_event->mutex, MAX_DELAY);
	group_event->value |= bits_to_set;
	uint32_t value = group_event->value;
	MutexGive(&group_event->mutex);
	eventfd_write(group_event->spi_proto_flags, 0xFFFF); // Value is for notification purpose only
	return value;
}

uint32_t EventGroupClearBits(const group_event_t* event, const uint32_t bits_to_clear) {
	group_event_t* group_event = (group_event_t*) event;
	MutexTake(&group_event->mutex, MAX_DELAY);
	uint32_t value = group_event->value &= ~bits_to_clear;
	MutexGive(&group_event->mutex);
	return value;
}

uint32_t EventGroupWaitBits(const group_event_t* event, const uint32_t bits_wait_for, const int clear_on_exit,
		const int wait_for_all_bits, uint32_t ticks_to_wait) {
	group_event_t* group_event = (group_event_t*) event;
	int ret_value, c;
	uint32_t value_copy;
	do {
		MutexTake(&group_event->mutex, MAX_DELAY);
		value_copy = group_event->value;
		MutexGive(&group_event->mutex);
		if ((value_copy & bits_wait_for) == bits_wait_for)
			break;
		else if (!wait_for_all_bits && (value_copy & bits_wait_for) != 0)
			break;

		struct epoll_event tx_evs[2] = { 0 };
		int epol_res = epoll_wait(group_event->epoll_instance, tx_evs, 2, ticks_to_wait);
		if (epol_res < 0)
			return 0;
		for (c = 0; c < epol_res; c++) {
			struct epoll_event *evs = tx_evs + c;
			if (evs->data.fd == group_event->irq_fd) {
				MutexTake(&group_event->mutex, MAX_DELAY);
				group_event->value |= IRQ_EVENT;
				MutexGive(&group_event->mutex);
			} else if (evs->data.fd == group_event->spi_proto_flags) {
				//We do not do anything here
			}
		}
	} while (true);

	MutexTake(&group_event->mutex, MAX_DELAY);
	value_copy = group_event->value;
	if (clear_on_exit)
		group_event->value &= ~bits_wait_for;
	MutexGive(&group_event->mutex);

	return value_copy;
}

uint32_t EventGroupSync(const group_event_t* event, const uint32_t uxBitsToSet, const uint32_t uxBitsToWaitFor,
		uint32_t xTicksToWait) {
	EventGroupSetBits(event, uxBitsToSet);
	return EventGroupWaitBits(event, uxBitsToWaitFor, true, false, xTicksToWait);
}

void EventGroupDelete(group_event_t* tx_eg) {
	delete ((group_event_t*) tx_eg);
}

void sha256_init(SHA256_T* sha) {
	if (!SHA256_Init(sha))
		LOGE("Failed to init sha");
}
void sha256_update(SHA256_T* sha, void* buffer, int len) {
	if (!SHA256_Update(sha, buffer, len))
		LOGE("Failed to update sha");
}
void sha256_finish(SHA256_T* sha, void* buffer) {
	if (!SHA256_Final((unsigned char*) buffer, sha))
		LOGE("Failed to calculate sha");
}
