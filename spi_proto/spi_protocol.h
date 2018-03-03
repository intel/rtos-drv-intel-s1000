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

#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include "ipc_msg.h"
#include "pal_os_specific.h"
#include "spi_lib_error.h"
#include "peripherals_os_specific.h"


#define MIN_POOL_SIZE_FOR_SINGLE_THREADED 2
#define MIN_POOL_SIZE_FOR_MULTI_THREADED 3

#if !defined(MAX_CALLBACK_ALLLOCATED) || MAX_CALLBACK_ALLLOCATED <= 0
	#error "#### MAX_CALLBACK_ALLLOCATED has to be defined of min size 1 ####"
#endif

#if !defined(POOL_SIZE) || (!defined(TX_TASK) && POOL_SIZE < MIN_POOL_SIZE_FOR_SINGLE_THREADED) || (defined(TX_TASK) && POOL_SIZE < MIN_POOL_SIZE_FOR_MULTI_THREADED)
	#error "#### POOL_SIZE has to be defined - min size should vary from TX_TASK enabled/disabled  ####"
#endif

#if !defined(IPC_MODE) || IPC_MODE != SUE_IPC_SIZE_64B && IPC_MODE != SUE_IPC_SIZE_4KB
	#error "#### IPC_MODE has to be defined - equal to SUE_IPC_SIZE_64B or SUE_IPC_SIZE_4KB ####"
#endif

typedef enum {
	UNKNOWN, ROM_READY, ROM_LOAD, FW_READY, DFU_READY
}dsp_state_t;

typedef enum {
    BOOT_SPI_MASTER = 0,
    BOOT_SPI_SLAVE = 1,
} sue_boot_mode;

typedef enum spi_proto_version {
	VER_DEFAULT, // decided by code
	VER_NO_WAKE, // 1-wire, "old" protocol
	VER_WAKE_1, // 2-wire hybird
	// VER_WAKE_2, // 2-wire handshake
	VER_CNT
} spi_proto_version;

struct work_item_;
typedef struct work_item_ work_item;
typedef void (*work_item_callback)(void*);

typedef uint16_t (*read_fw_callback_t)(void*, uint8_t*, uint16_t);
typedef struct
{
	uint32_t image_size;
	uint32_t address;
	read_fw_callback_t read_fw;
	void* src;
} fw_loader_t;

typedef struct {
	spi_device_settings_t spi_settings;
	uint32_t spi_chunk_size;
	spi_proto_version proto_version;
} spi_proto_settings;

typedef struct {
	spiproto_cb_func_t handler;
	uint16_t type;
	bool all;
} notif_handler_t;

typedef struct {
	ipc_msg *msg;
	bool directly_from_pool;
} ipc_wrapper;
typedef struct
{
	ipc_msg* msg;
	spiproto_cb_func_t* custom_callback;
}sync_msg_t;

typedef struct {
	ipc_wrapper response;
	sync_msg_t sync_msg;
	enum sue_ipc_size ipc_mode;
	dsp_state_t dsp_state;
#ifndef TX_TASK
	bool is_idle;
#endif
} spi_state;

typedef struct {
	EVENT_GROUP_T tx_eg;
	QUEUE_HANDLE_T(work_item) work_queue;
	QUEUE_HANDLE_T(ipc_msg*) ipc_storage_pool;
#ifdef TX_TASK
	SEMAPHORE_T ipc_msg_sem;
	SEMAPHORE_T boot_sem;
	MUTEX_T ipc_transfer_mtx;
	MUTEX_T send_ipc_mtx;
	MUTEX_T spi_mutex;
	TaskHandle_t work_item_task;
#else
	SPIN_LOCK_T spin_lock;
#endif
	SPI_DEVICE_T spi_device;
	notif_handler_t spi_proto_cb_func[MAX_CALLBACK_ALLLOCATED];
	uint8_t ipc_mem_pool[POOL_SIZE * (IPC_MODE + sizeof(ipc_msg))];
#ifdef RX_TASK
	TaskHandle_t rx_task;
	QUEUE_HANDLE_T(ipc_msg*) rx_task_queue;
#endif
	spi_proto_settings settings;
	spi_state state;
	void* ctx;
} spi_protocol_t;


/* wprapper for xEventGroupWaitBits with params set to "CLEAR, ANY" */
static inline uint32_t takeEvent(EVENT_GROUP_T eg, const uint32_t bit_mask, uint32_t timeout) {
	return bit_mask & EventGroupWaitBits(eg, bit_mask, true, false, timeout);
}

proto_err ipc_msg_transfer(spi_protocol_t* spi_proto, ipc_msg* msg, uint32_t time_out_val);
proto_err send_ipc(spi_protocol_t* spi_proto, uint32_t primary, uint32_t extenion, uint32_t data_length, const void *data);
proto_err send_large_cfg_set(spi_protocol_t* spi_proto, uint32_t module, uint32_t instance, uint32_t param_id, uint32_t data_length, void *data);
void ipc_clear(ipc_msg *msg);
size_t get_ipc_size(spi_proto_settings* settings);
void reset_platform(spi_protocol_t* spi_proto, uint32_t boot_mode);
void change_spi_speed(spi_protocol_t* spi_proto, uint32_t clock);
void ipc_push(spi_protocol_t* spi_proto, ipc_msg *msg);
void push_WI(spi_protocol_t* spi_proto, void *ctx, work_item_callback callback, uint16_t ipc_length);
ipc_msg* get_ipc_storage(spi_protocol_t* spi_proto);
proto_err add_ipc_storage(spi_protocol_t* spi_proto, ipc_msg* msg);
proto_err spi_protocol_destroy(spi_protocol_t* spi_proto);
proto_err spi_protocol_init(spi_protocol_t* spi_proto, spi_proto_settings* settings);
proto_err spi_protocol_deinit(spi_protocol_t* spi_proto);
proto_err spi_transfer(spi_protocol_t *spi_proto, uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t len);
proto_err spi_proto_handler_add(spi_protocol_t* spi_proto, spiproto_callback_t callback_handler, void* args, uint16_t notif_type);
proto_err spi_proto_handler_remove(spi_protocol_t* spi_proto, spiproto_callback_t callback_handler, void* args);
proto_err fw_load(spi_protocol_t* spi_proto, const fw_loader_t* fw_loader);
#ifndef TX_TASK
void schedule(spi_protocol_t *spi_proto);
#endif

#endif
