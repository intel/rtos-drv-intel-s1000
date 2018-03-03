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

#ifndef OS_SPECIFIC_H
#define OS_SPECIFIC_H

/*
    As spi proto libary was originally developed on FreeRTOS.
    Libary expects that every implementation will be working in the same way as FreeRTOS implementation API.
    Please refer to FreeRTOS API https://www.freertos.org/a00106.html

    1) Below has to be defined for all implementation. Single and Multithreaded
	typedef os_specific_type QUEUE_HANDLE_T;
	typedef os_specific_type SEMAPHORE_T;
	typedef os_specific_type EVENT_GROUP_T;
	typedef os_specific_type SHA256_T;
	typedef os_specific_type MUTEX_T;

	#define MAX_DELAY portMAX_DELAY
	#define QueueCreate(item, size) xQueueCreate(item, size)
	#define QueuePeek(self, item, timeout) xQueuePeek(self, item, timeout)
	#define QueueSend(self, item, timeout) xQueueSend(self, item, timeout)
	#define QueueReceive(self, item, timeout) xQueueReceive(self, item, timeout)
	#define QueueMessagesWaiting(self) uxQueueMessagesWaiting(self)
	#define QueueDelete(self) vQueueDelete(self)

	#define CreateEventGroup() xEventGroupCreate()
	#define EventGroupWaitBits(event, uxBitsToWaitFor, xClearOnExit, xWaitForAllBits, xTicksToWait) \
		xEventGroupWaitBits(event, uxBitsToWaitFor, xClearOnExit, xWaitForAllBits, xTicksToWait)
	#define EventGroupSetBits(xEventGroup, uxBitsToSet) \
		xEventGroupSetBits(xEventGroup, uxBitsToSet)
	#define EventGroupClearBits(event, uxBitsToClear) \
		xEventGroupClearBits(event, uxBitsToClear)
	#define EventGroupGetBits(event) \
		xEventGroupGetBits(event)

	#define EventGroupSync(event, uxBitsToSet, uxBitsToWaitFor, xTicksToWait) \
			xEventGroupSync(event, uxBitsToSet, uxBitsToWaitFor, xTicksToWait)

	#define EventGroupDelete(event) vEventGroupDelete(event)

	#define CreateSemaphore() xSemaphoreCreateBinary()
	#define SemaphoreTake(sepamhore, timeout) xSemaphoreTake(sepamhore, timeout)
	#define SemaphoreGive(sepamhore) xSemaphoreGive(sepamhore)
	#define SemaphoreDelete(sepamhore) vSemaphoreDelete(sepamhore)

	#define CreateMutex() xSemaphoreCreateMutex()
	#define MutexTake(mutex, timeout) xSemaphoreTake(mutex, timeout)
	#define MutexGive(mutex) xSemaphoreGive(mutex)
	#define MutexDelete(mutex) vSemaphoreDelete(mutex)

	2) Has to be defined for multithreaded mode

	typedef os_specific_type SPIN_LOCK_T;
	#define CreateSpinkLock() os_specific
	#define PROTO_ENTER_CRITICAL(lock) os_specific
	#define PROTO_EXIT_CRITICAL(lock) os_specific

	#define CreateTask(callback, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID) \
	xTaskCreatePinnedToCore(callback, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID)

	#define TaskDelay(time) vTaskDelay(time)
	#define TaskDelete(task) vTaskDelete(task)

	3) Swap endianess to given buffer and length
	void swap_buffer(void *data, uint32_t size)

	4) Below functions are required to verify image integrity while loading FW using SPI-SLAVE.
	Please refer to https://www.openssl.org/docs/man1.1.0/man3/SHA256_Init.html

	void sha256_init(mbedtls_sha256_context *ctx);
	#define sha256_update(ctx, input, len) mbedtls_sha256_update (ctx, input, len)
	#define sha256_finish(ctx, output) mbedtls_sha256_finish (ctx, output)
 *
 */


#define FREERTOS

//#define TX_TASK
//#define RX_TASK
#define IPC_MODE SUE_IPC_SIZE_64B
#define SUE_SEND_IPC_TIMEOUT   1000
#define SUE_LOAD_FW_TIMEOUT   20000
#define WAIT_FOR_ROM_FW_READY   1000
#define MAX_CALLBACK_ALLLOCATED 8
#ifdef TX_TASK
#define POOL_SIZE 3
#else
#define POOL_SIZE 2
#endif

#ifdef FREERTOS
#include "FreeRTOS/pal.h"
#elif defined(LINUX_US)
#include "Linux/pal.h"
#else
	#error "#### OS has to be specified ####"
#endif

#endif
