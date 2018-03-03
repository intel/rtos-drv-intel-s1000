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

#ifndef PAL_H
#define PAL_H

#include <deque>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <openssl/sha.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include "semaphore.h"

#define INFO  "32"
#define WARN  "33"
#define ERROR "31"
#define DEBUG INFO

#define _LOG(level, format, ...)  fprintf(stdout, "\033[0;"  level  "m %s-%s:" format "\033[0m\n", LOG_TAG, __FUNCTION__, ## __VA_ARGS__)
#define LOGI(format, ...) _LOG(INFO, format, ## __VA_ARGS__)
#define LOGE(format, ...) _LOG(ERROR, format, ## __VA_ARGS__)
#define LOGW(format, ...) _LOG(WARN, format, ## __VA_ARGS__)
#define LOGD(format, ...) _LOG(DEBUG, format, ## __VA_ARGS__)

typedef sem_t* SEMAPHORE_T;
typedef pthread_spinlock_t SPIN_LOCK_T;
typedef pthread_mutex_t* MUTEX_T;
typedef void * TaskHandle_t;
typedef SHA256_CTX SHA256_T;

typedef struct {
	int epoll_instance;
	int irq_fd;
	int spi_proto_flags;
	pthread_mutex_t mutex;
	uint32_t value;
} group_event_t;

#define MAX_DELAY UINT32_MAX
#define SAFE_IRQ_ARMING_DURATION 0

namespace pal_linux_us_impl {
template<typename T>
class deque {
	std::deque<T> _deque;
	std::mutex _mutex;
	std::condition_variable _cond;
public:
	int size();
	bool push(T*, uint32_t);
	bool peek(T*, uint32_t, bool);
};

template<typename T>
inline bool deque<T>::push(T* item, uint32_t timeout) {
	T& t = static_cast<T&>(*item);
	{
		std::unique_lock <std::mutex> lock(_mutex);
		_deque.push_back(t);
	}
	_cond.notify_one();
	return true;
}

template<typename T>
inline bool deque<T>::peek(T* item, uint32_t timeout, bool pop_on_success) {
	std::unique_lock <std::mutex> lock(_mutex);
	bool is_empty = _deque.empty();
	if ((is_empty && timeout == 0)
			|| (is_empty && _cond.wait_for(lock, std::chrono::milliseconds(timeout)) == std::cv_status::timeout))
		return false;

	T& qitem = _deque.front();
	memcpy(item, &qitem, sizeof(T));

	if (pop_on_success)
		_deque.pop_front();

	return true;
}

template<typename T>
inline int deque<T>::size() {
        std::unique_lock <std::mutex> lock(_mutex);
        return _deque.size();
}

}
using pal_linux_us_impl::deque;
#define QUEUE_HANDLE_T(type) deque<type>*
#define QueueCreate(type, queue_size) new deque<type>()
#define QueueSend(self, item, timeout) self->push(item, timeout)
#define QueuePeek(self, item, timeout) self->peek(item, timeout, false)
#define QueueReceive(self, item, timeout) self->peek(item, timeout, true)
#define QueueMessagesWaiting(self) self->size()
#define QueueDelete(self) delete self

sem_t* CreateSemaphore();
int SemaphoreTake(sem_t* sem, uint32_t max_wait);
int SemaphoreGive(sem_t* sem);
void SemaphoreDelete(sem_t* sempaphore);

pthread_spinlock_t CreateSpinkLock();
#define PROTO_ENTER_CRITICAL(spinlock) pthread_spin_lock(spinlock)
#define PROTO_EXIT_CRITICAL(spinlock) pthread_spin_unlock(spinlock)

pthread_mutex_t* CreateMutex();
int MutexTake(pthread_mutex_t* mutex, uint32_t max_wait);
#define MutexGive(mutex) pthread_mutex_unlock(mutex)
void MutexDelete(pthread_mutex_t* mutex);

int CreateTask(void* callback, const char * const pcName, const uint32_t usStackDepth, void * const pvParameters, int uxPriority,
		void * const pxCreatedTask, int xCoreID);

void TaskDelete(void* task);
void TaskDelay(uint32_t time);

void swap_buffer(void *data, uint32_t size);

group_event_t* CreateEventGroup();
uint32_t EventGroupSetBits(const group_event_t* event, const uint32_t bits_to_set);
uint32_t EventGroupClearBits(const group_event_t* event, const uint32_t bits_to_clear);
uint32_t EventGroupWaitBits(const group_event_t* event, const uint32_t bits_wait_for, const int clear_on_exit,
		const int wait_for_all_bits, uint32_t ticks_to_wait);
#define EventGroupGetBits(event) EventGroupClearBits(event , 0 )
uint32_t EventGroupSync(const group_event_t*, const uint32_t uxBitsToSet, const uint32_t uxBitsToWaitFor, uint32_t xTicksToWait);
void EventGroupDelete(group_event_t*);

void sha256_init(SHA256_T* sha);
void sha256_update(SHA256_T* ctx, void* buffer, int len);
void sha256_finish(SHA256_T* ctx, void* buffer);

#endif
