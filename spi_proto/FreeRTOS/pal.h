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

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "mbedtls/sha256.h"
#include "esp_log.h"

#define LOGI(format, ...) do { ESP_LOGI(LOG_TAG, format, ##__VA_ARGS__); } while (0)
#define LOGE(format, ...) do { ESP_LOGE(LOG_TAG, format, ##__VA_ARGS__); } while (0)
#define LOGW(format, ...) do { ESP_LOGW(LOG_TAG, format, ##__VA_ARGS__); } while (0)
#define LOGD(format, ...) do { ESP_LOGD(LOG_TAG, format, ##__VA_ARGS__); } while (0)

typedef SemaphoreHandle_t SEMAPHORE_T;
typedef SemaphoreHandle_t MUTEX_T;
typedef portMUX_TYPE SPIN_LOCK_T;
typedef EventGroupHandle_t EVENT_GROUP_T;
typedef mbedtls_sha256_context SHA256_T;


#define PROTO_ENTER_CRITICAL(lock) portENTER_CRITICAL(lock)
#define PROTO_EXIT_CRITICAL(lock) portEXIT_CRITICAL(lock)

#define MAX_DELAY portMAX_DELAY
#define QUEUE_HANDLE_T(type) QueueHandle_t
#define QueueCreate(type, item) xQueueCreate(item, sizeof(type))
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

#ifdef TX_TASK
#define CreateMutex() xSemaphoreCreateMutex()
#define MutexTake(mutex, timeout) xSemaphoreTake(mutex, timeout)
#define MutexGive(mutex) xSemaphoreGive(mutex)
#define MutexDelete(mutex) vSemaphoreDelete(mutex)
#else
#define CreateMutex()
#define MutexTake(mutex, timeout)
#define MutexGive(mutex)
#define MutexDelete(mutex)
#endif



#define CreateSpinkLock() portMUX_INITIALIZER_UNLOCKED

#define CreateTask(callback, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID) \
	xTaskCreatePinnedToCore(callback, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID)

#define TaskDelay(time) vTaskDelay(time)
#define TaskDelete(task) vTaskDelete(task)

void swap_buffer(void *data, uint32_t size);

void sha256_init(mbedtls_sha256_context *ctx);
#define sha256_update(ctx, input, len) mbedtls_sha256_update (ctx, input, len)
#define sha256_finish(ctx, output) mbedtls_sha256_finish (ctx, output)

#endif
