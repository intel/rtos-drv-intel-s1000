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
#include "spi_protocol.h"
#include "sue_rom.h"

static const char *LOG_TAG = "AudDrv-Proto";

struct work_item_ {
	void *self;
	work_item_callback callback;
	uint16_t ipc_length;
};


typedef struct
{
	SHA256_T sha_ctx ;
	fw_loader_t fw_loader;
}load_fw_ctx_t;

static bool send_null(spi_protocol_t *spi_proto);
static bool boot_handler(spi_protocol_t* spi_proto, ipc_msg* msg);

static void wait_for_irq(EVENT_GROUP_T eg) {
	takeEvent(eg, IRQ_EVENT, MAX_DELAY);
}

static inline bool is_next_ipc_4k(spi_protocol_t* spi_proto) {
	work_item f_item;
	bool res = false;
	if (QueuePeek(spi_proto->work_queue, &f_item, 0))
	{
		if (f_item.ipc_length == 0)
			LOGE("Work item length should be non zero and set "
					"adequatly to ipc_msg length for better spi utilization");
		else if (f_item.ipc_length > SUE_IPC_SIZE_64B)
			res = true;
	}
	return res;
}

static inline bool is_host_req_mode_change(ipc_msg *tx) {
	return ((tx->buf[1] >> 6) & 0xF) == SUE_R64_W4K;
}

static inline bool is_dsp_req_mode_change(ipc_msg *rx) {
	return rx->buf[0] == 0x81000004 && rx->buf[1] == 0x40000000;
}

static inline bool is_msg_reponse(ipc_msg *rx) {
	return (rx->buf[0] >> 29) & 1;
}

static void ipc_msg_dispatch(spi_protocol_t* spi_proto, ipc_msg *msg) {
	void* owner;
	int c;
	bool handled = false;
	spiproto_callback_t callback;
	if (msg->self.owner) {
		callback = msg->self.callback;
		owner = msg->self.owner;
		if (!callback(owner, msg))
			add_ipc_storage(spi_proto, msg);
		return;
	}
	uint16_t notif_type = GET_FIELD(msg->buf[0], MSG_NOTIF_NOTIF_TYPE_FLD_WIDTH, MSG_NOTIF_NOTIF_TYPE_FLD_OFFSET);
	for (c = 0; c < MAX_CALLBACK_ALLLOCATED; c++) {
		callback = spi_proto->spi_proto_cb_func[c].handler.callback;
		owner = spi_proto->spi_proto_cb_func[c].handler.owner;
		if (owner
			&& (spi_proto->spi_proto_cb_func[c].type == notif_type || spi_proto->spi_proto_cb_func[c].all)
			&& callback(owner, msg)) {
				handled = true;
				break;
		}
	}
	if (handled)
		return;

	LOGW("Nobody interested in msg 0x%08x 0x%08x 0x%08x 0x%08x", msg->buf[0], msg->buf[1], msg->buf[2], msg->buf[3]);
	add_ipc_storage(spi_proto, msg);
}

static int32_t proto_send(spi_protocol_t *spi_proto, ipc_wrapper *tx_msg, ipc_wrapper *rx_msg, enum sue_ipc_size *size) {
	int32_t res;
	res = spi_transfer(spi_proto, (uint8_t*) tx_msg->msg->buf, (uint8_t*) rx_msg->msg->buf, *size);
	bool dsp_mode_change = is_dsp_req_mode_change(rx_msg->msg);
	bool is4k = dsp_mode_change || is_host_req_mode_change(tx_msg->msg);
	uint32_t type = GET_FIELD(rx_msg->msg->buf[0], MSG_REPLY_TYPE_FLD_WIDTH, MSG_REPLY_TYPE_FLD_OFFSET)
	if (!dsp_mode_change && (is_msg_reponse(rx_msg->msg) || type == GLB_MSG_T_NOTIFICATION || type == GLB_MSG_T_ROM_CONTROL)) {
		if (type != GLB_MSG_T_NOTIFICATION && type != GLB_MSG_T_ROM_CONTROL) {
			rx_msg->msg->status = GET_FIELD(rx_msg->msg->buf[0], MSG_REPLY_STATUS_FLD_WIDTH, MSG_REPLY_STATUS_FLD_OFFSET);
			if (rx_msg->msg->status)
				LOGE("Got error on response pri: 0x%x, ext: 0x%x, payload[0]: 0x%x, payload[1]: 0x%x, status: %d",
						rx_msg->msg->buf[0], rx_msg->msg->buf[1], rx_msg->msg->buf[2], rx_msg->msg->buf[3], rx_msg->msg->status);

		}
		rx_msg->msg->len = *size;
#ifdef RX_TASK
		if (!QueueSend(spi_proto->rx_task_queue, &rx_msg->msg, SUE_SEND_IPC_TIMEOUT)) {
			LOGE("Did not add to rx queue");
			add_ipc_storage(spi_proto, rx_msg->msg);
		}
#else
		ipc_msg_dispatch(spi_proto, rx_msg->msg);
#endif
		rx_msg->msg = NULL;
	} else if (rx_msg->directly_from_pool) {
		add_ipc_storage(spi_proto, rx_msg->msg);
		rx_msg->msg = NULL;
	}
	*size = is4k ? SUE_IPC_SIZE_4KB : SUE_IPC_SIZE_64B;
	return res;
}

static bool send_null(spi_protocol_t *spi_proto) {
	ipc_msg *msg = get_ipc_storage(spi_proto);
	ipc_wrapper tx = { .msg = msg }, rx = { .msg = msg };
	proto_send(spi_proto, &tx, &rx, &spi_proto->state.ipc_mode);
	if (!rx.msg)
		return false;

	add_ipc_storage(spi_proto, msg);
	return true;
}

static int32_t send_mode_chg(spi_protocol_t *spi_proto, ipc_wrapper *tx_msg, ipc_wrapper *rx_msg) {
	int32_t res;
	uint32_t mod_val = 0;
	uint32_t ext = tx_msg->msg->buf[1];
	SET_FIELD(mod_val, SUE_R4K_W64, SUE_IPC_MODE_SEL_R4W4_FLD_WIDTH, SUE_IPC_MODE_SEL_R4W4_FLD_OFFSET)
	tx_msg->msg->buf[1] = mod_val;
	enum sue_ipc_size size = SUE_IPC_SIZE_64B;
	res = proto_send(spi_proto, tx_msg, rx_msg, &size);
	swap_buffer(tx_msg->msg->buf, SUE_IPC_SIZE_64B);
	tx_msg->msg->buf[1] = ext;
	return res;

}

static void process_work_items(spi_protocol_t *spi_proto)
{
	work_item w_item;
	bool is_anything;
	while ((is_anything = QueueReceive(spi_proto->work_queue, &w_item, 0)) || spi_proto->state.response.msg) {
		if (is_anything)
			w_item.callback(w_item.self);
		else
			ipc_push(spi_proto, NULL);

		EventGroupClearBits(spi_proto->tx_eg, MSG_EVENT);
	}
}

#ifdef TX_TASK
/* wait for MSG or notif
 * return event group bits set*/
static inline uint32_t wait_while_idle(EVENT_GROUP_T eg, spi_device_settings_t* spi_settings) {
	uint32_t ev = 0;
	bool could_skip_wait = false;

	/* (high irq was already consumed) */
	if (!irq_get_level(spi_settings)) {
		/* notif already arrived */
		return IRQ_EVENT;
	}

	/* there is nothing to sent or receive,
	 * wait for notif (low edge) or something in queue */
	EventGroupClearBits(eg, IRQ_STATUS);
	irq_set_falling_edge(spi_settings);

	if (!irq_get_level(spi_settings)) {
		/* IRQ changed to low before arming was effective
		 * (there is need to clear IRQ flag anyway) */
		ev = IRQ_EVENT;
	} else {
		ev = takeEvent(eg, IRQ_EVENT|MSG_EVENT, MAX_DELAY);
		could_skip_wait = ev & IRQ_EVENT;
	}

	/* something arrived, switch back to active irq mode */
	irq_set_rising_edge(spi_settings);

	if (!could_skip_wait) {
		/* if ev==MSG_EVENT:
		 * it is possible that notif with its low IRQ arrived anyway
		 * to ISR after rearming to high,
		 * we must ensure that such situation would not be interpreted
		 * as next IRQ high (the one confirming next null message)
		 *
		 * if ev==IRQ_EVENT:
		 * it is possible that ISR set IRQ_EVENT flag in eg after first
		 * xEventGroupWaitBits() call, to prevent race we must clear it
		 * only after rearming
		 * */
		if (SAFE_IRQ_ARMING_DURATION)
			takeEvent(eg, IRQ_EVENT, SAFE_IRQ_ARMING_DURATION);
		else
			if (EventGroupGetBits(eg) & IRQ_STATUS)
			{
				ev |= takeEvent(eg, IRQ_EVENT, MAX_DELAY);
			}
	}
	return ev;
}

/* wait for MSG or notif */
static inline uint32_t wait_while_idle_wake(spi_protocol_t *spi_proto) {
	EVENT_GROUP_T eg = spi_proto->tx_eg;
	uint32_t ev = takeEvent(eg, IRQ_EVENT|MSG_EVENT, MAX_DELAY);
	if ((ev & MSG_EVENT) && (!(ev & IRQ_EVENT))  && spi_proto->state.dsp_state >= FW_READY) {
		uint32_t wake_pin = spi_proto->settings.spi_settings.wake_pin;
		if (wake_pin != -1) {
			gpio_set_level(wake_pin, 1);
			if (!takeEvent(eg, IRQ_EVENT, 30)) {
				LOGW("Did not get IRQ_EV after asserting WAKE; continuing anyway");
			}
			gpio_set_level(wake_pin, 0);
		}
	}
	return ev;
}

static void enter_idle(spi_protocol_t* spi_proto) {
	bool rx_is_null = send_null(spi_proto);
	if (spi_proto->settings.proto_version == VER_NO_WAKE) {
		wait_for_irq(spi_proto->tx_eg);
			if (rx_is_null) {
					uint32_t bits = wait_while_idle(spi_proto->tx_eg, &spi_proto->settings.spi_settings);
					if ((bits & IRQ_EVENT) && !(bits & MSG_EVENT)) {
							send_null(spi_proto);
							wait_for_irq(spi_proto->tx_eg);
					}
			}
			return;
	}
	if (!rx_is_null) {
			wait_for_irq(spi_proto->tx_eg);
			return;
	} else if (spi_proto->state.dsp_state < FW_READY) {
			/* avoid null msg spam */
			wait_for_irq(spi_proto->tx_eg);
			EventGroupWaitBits(spi_proto->tx_eg, MSG_EVENT, false, false, MAX_DELAY);
			return;
	}
	wait_while_idle_wake(spi_proto);
}

static void work_item_task(void *arg) {
	spi_protocol_t *spi_proto = (spi_protocol_t *) arg;
	char ver_lut[][18] = {
		"default (invalid)",
		"1-wire (old)",
		"2-wire (wake v1)",
		"2-wire (wake v2)",
	};
	spi_proto_version proto_version = spi_proto->settings.proto_version;
	const char *ver_str = proto_version < VER_CNT ? ver_lut[proto_version] : "invalid";
	const char *LOG_TAG = __FUNCTION__;
	LOGI("Started. IRQ lvl: %d, IPC ver: %s", gpio_get_level(spi_proto->settings.spi_settings.irq_pin), ver_str);

	while (true) {
		process_work_items(spi_proto);
		enter_idle(spi_proto);
	}
}
#else
static inline bool isr_forward_enable(spi_protocol_t* spi_proto)
{
	PROTO_ENTER_CRITICAL(&spi_proto->spin_lock);
	uint32_t value = EventGroupSync(spi_proto->tx_eg, ISR_FORWARD, IRQ_EVENT, 0);
	PROTO_EXIT_CRITICAL(&spi_proto->spin_lock);
	if (value & IRQ_EVENT)
	{
		EventGroupClearBits(spi_proto->tx_eg, ISR_FORWARD);
		return false;
	}

	return true;
}
static void enter_idle(spi_protocol_t* spi_proto) {
	if (spi_proto->state.is_idle)
		return;

	spi_device_settings_t* spi_settings = &spi_proto->settings.spi_settings;
	bool rx_is_null = send_null(spi_proto);
	if (spi_proto->settings.proto_version == VER_NO_WAKE)
	{
		wait_for_irq(spi_proto->tx_eg);

		if (rx_is_null && irq_get_level(spi_settings)) {
			irq_set_falling_edge(spi_settings);
			EventGroupClearBits(spi_proto->tx_eg, IRQ_STATUS);
			if (irq_get_level(spi_settings) && isr_forward_enable(spi_proto))
			{
				spi_proto->state.is_idle = true;
				return;
			}
		}

		spi_settings->isr_handler.isr_callback(spi_settings->isr_handler.ctx);
	}
	else
	{
		if (!rx_is_null || spi_proto->state.dsp_state < FW_READY)
			wait_for_irq(spi_proto->tx_eg);
		else if (isr_forward_enable(spi_proto))
		{
			spi_proto->state.is_idle = true;
			return;
		}
	}
}

static void exit_idle(spi_protocol_t* spi_proto) {

	if (spi_proto->settings.proto_version == VER_NO_WAKE)
	{
		EventGroupClearBits(spi_proto->tx_eg, ISR_FORWARD);
		spi_proto->state.is_idle = false;
		irq_set_rising_edge(&spi_proto->settings.spi_settings);
		if (QueueMessagesWaiting(spi_proto->work_queue))
			return;

		send_null(spi_proto);
		wait_for_irq(spi_proto->tx_eg);
	}
	else
	{
		if (QueueMessagesWaiting(spi_proto->work_queue))
		{
			EventGroupClearBits(spi_proto->tx_eg, ISR_FORWARD);
			if (spi_proto->state.is_idle)
			{
				/*
				 * When exitting idle state, driver needs to check whether interrupt line is high.
				 * If not, wake line should be rised and driver ought to wait patinently for dsp to
				 * rise interrupot line as an acknowladement.
				 */
				if (!(EventGroupGetBits(spi_proto->tx_eg) & IRQ_EVENT))
				{
					uint32_t wake_pin = spi_proto->settings.spi_settings.wake_pin;
					gpio_set_level(wake_pin, 1);
					if (!takeEvent(spi_proto->tx_eg, IRQ_EVENT, 100))
						LOGE("irq after wake timeout");
					gpio_set_level(wake_pin, 0);
				}
				else
				{
					/*
					 * If dsp did already rise interrupt line, just clear interrupt flag.
					 */
					EventGroupClearBits(spi_proto->tx_eg, IRQ_EVENT);
				}
				spi_proto->state.is_idle = false;
			}
		}
		else
		{
			if(spi_proto->state.is_idle && isr_forward_enable(spi_proto))
				spi_proto->state.is_idle = true;
			else
			{
				spi_proto->state.is_idle = false;
				send_null(spi_proto);
				wait_for_irq(spi_proto->tx_eg);
			}
		}
	}
}

void schedule(spi_protocol_t *spi_proto)
{
	exit_idle(spi_proto);
	process_work_items(spi_proto);
	enter_idle(spi_proto);
}
#endif

#ifdef RX_TASK
static void rx_task(void *arg) {
	spi_protocol_t *spi_proto = (spi_protocol_t *) arg;
	ipc_msg *msg;
	while (true) {
		if (QueueReceive(spi_proto->rx_task_queue, &msg, MAX_DELAY))
			ipc_msg_dispatch(spi_proto, msg);
	}
}
#endif

size_t add_ipc_storage_pool(spi_protocol_t* spi_proto) {
	size_t i;
	size_t ipc_size = get_ipc_size(&spi_proto->settings);
	uint8_t* ptr = spi_proto->ipc_mem_pool;
	for (i = 0; i < POOL_SIZE; i++) {
		ipc_msg* msg = (ipc_msg*)(ptr + (ipc_size * i));
		if (!msg)
			break;
		if (add_ipc_storage(spi_proto, msg)) {
			break;
		}
	}
	return i;
}

size_t rm_ipc_storage_pool(spi_protocol_t* spi_proto, const size_t count) {
	size_t i = 0;
	ipc_msg* msg;
	while (QueueReceive(spi_proto->ipc_storage_pool, &msg, (uint32_t )0)) {
		i++;
	}
	return i;
}

proto_err spi_protocol_init(spi_protocol_t* spi_proto, spi_proto_settings *settings) {

	if(!spi_proto)
	{
		LOGE("Spi proto nullptr");
		return PROTO_NULL_POINTER;
	}
	else
		memset(spi_proto, 0, sizeof(spi_protocol_t));

	proto_err ret_val = PROTO_SUCCESS;
	if (settings->spi_chunk_size > IPC_MODE)
	{
		LOGE("SPI chunk size %d has to be smaller/equal to packet size %d", settings->spi_chunk_size, IPC_MODE);
		return PROTO_INVALID_CONFIGURATION;
	}

#ifndef TX_TASK
	if (!settings->spi_settings.isr_handler.isr_callback)
	{
		LOGE("ISR callback handler is required for this configuration ");
		return PROTO_INVALID_CONFIGURATION;
	}
	spi_proto->state.is_idle = false;
#endif

	spi_proto->state.ipc_mode = SUE_IPC_SIZE_64B;
	memcpy(&spi_proto->settings, settings, sizeof(spi_proto_settings));
#ifdef TX_TASK
	spi_proto->ipc_msg_sem = CreateSemaphore();
	spi_proto->spi_mutex = CreateMutex();
	spi_proto->ipc_transfer_mtx = CreateMutex();
	spi_proto->send_ipc_mtx = CreateMutex();
	spi_proto->boot_sem = CreateSemaphore();
#else
	spi_proto->spin_lock = (SPIN_LOCK_T)CreateSpinkLock();
#endif
	spi_proto->tx_eg = CreateEventGroup();

	if (spi_proto->tx_eg == NULL) {
		LOGE("error in event\n");
	}

	ret_val = peripherals_init(&spi_proto->spi_device, &spi_proto->settings.spi_settings, spi_proto->tx_eg);
	if (ret_val)
	{
		LOGE("Failed to init peripherals");
		return ret_val;
	}
	irq_set_rising_edge(&spi_proto->settings.spi_settings);
	spi_proto->ipc_storage_pool = QueueCreate(ipc_msg*, POOL_SIZE);
	size_t chunksAdded = add_ipc_storage_pool(spi_proto);
	if (POOL_SIZE > chunksAdded) {
		LOGE("Adding storage chunk %zu/%u of ipc_storage_pool failed", chunksAdded, POOL_SIZE);
		size_t chunksRemoved = rm_ipc_storage_pool(spi_proto, chunksAdded);
		RET_ON_ERROR_LOG(chunksRemoved != chunksAdded, PROTO_OUT_OF_MEMORY, "Couldn't remove all storage from the ipc_storage_pool");
	}


	spi_proto->work_queue = QueueCreate(work_item, POOL_SIZE);

#ifdef RX_TASK
	spi_proto->rx_task_queue = QueueCreate(ipc_msg*, POOL_SIZE / 2 + 1);
	if (NULL == spi_proto->rx_task_queue) {
		LOGE("Error in Rx task Q Creation!");
	}

	if (!CreateTask((void*)rx_task, "rx_task", 1 * 2048, spi_proto, 15, &spi_proto->rx_task, 0)) {
		LOGE("error in rx task creation\n");
	}
#endif

	spi_proto_handler_add(spi_proto, (spiproto_callback_t) boot_handler, spi_proto, 0xffff);
	return ret_val;
}

static proto_err dev_dettach(spi_protocol_t* spi_proto) {
	proto_err ret = PROTO_SUCCESS;
	// Task Delete
#ifdef TX_TASK
	TaskDelete(spi_proto->work_item_task);
#endif
#ifdef RX_TASK
	QueueDelete(spi_proto->rx_task_queue);
	TaskDelete(spi_proto->rx_task);
#endif

	LOGI("All Tasks Destroyed");
#ifdef TX_TASK
	SemaphoreDelete(spi_proto->ipc_msg_sem);
	SemaphoreDelete(spi_proto->boot_sem);
	MutexDelete(spi_proto->ipc_transfer_mtx);
	MutexDelete(spi_proto->send_ipc_mtx);
	MutexDelete(spi_proto->spi_mutex);
#endif

	EventGroupDelete(spi_proto->tx_eg);
	ret = peripherals_close(&spi_proto->spi_device);
	gpio_set_level(spi_proto->settings.spi_settings.irq_pin, 0);

	return ret;
}

proto_err spi_protocol_deinit(spi_protocol_t* spi_proto) {
	proto_err ret_val = PROTO_SUCCESS;
	ret_val = dev_dettach(spi_proto);
	return ret_val;
}

proto_err spi_proto_handler_add(spi_protocol_t* spi_proto, spiproto_callback_t callback_handler, void* args, uint16_t notif_type) {
	int c;
	for (c = 0; c < MAX_CALLBACK_ALLLOCATED; c++)
		if (!(spi_proto->spi_proto_cb_func[c].handler.owner)) {
			LOGD("Adding SPI protocol handler %d/%d (@%p)...", c, MAX_CALLBACK_ALLLOCATED, callback_handler);
			spi_proto->spi_proto_cb_func[c].handler.owner = args;
			spi_proto->spi_proto_cb_func[c].handler.callback = callback_handler;
			if (notif_type == 0xffff)
				spi_proto->spi_proto_cb_func[c].all = true;
			else
				spi_proto->spi_proto_cb_func[c].type = notif_type;
			return PROTO_SUCCESS;
		}
	LOGE("Registering SPI protocol handler %d/%d (@%p) failed", c, MAX_CALLBACK_ALLLOCATED, callback_handler);
	return PROTO_HANDLER_ADD_REMOVE_FAILURE;
}

proto_err spi_proto_handler_remove(spi_protocol_t* spi_proto, spiproto_callback_t callback_handler, void* args) {
	int c;
	for (c = 0; c < MAX_CALLBACK_ALLLOCATED; c++)
		if (spi_proto->spi_proto_cb_func[c].handler.owner == args
				&& spi_proto->spi_proto_cb_func[c].handler.callback == callback_handler) {
			LOGD("Removing SPI protocol handler %d/%d (@%p)...", c, MAX_CALLBACK_ALLLOCATED, callback_handler);
			memset(&spi_proto->spi_proto_cb_func[c], 0, sizeof(notif_handler_t));
			return PROTO_SUCCESS;
		}
	LOGE("Removing SPI protocol handler %d/%d (@%p) failed", c, MAX_CALLBACK_ALLLOCATED, callback_handler);
	return PROTO_HANDLER_ADD_REMOVE_FAILURE;
}

proto_err spi_transfer(spi_protocol_t* spi_proto, uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t len) {
	proto_err res = PROTO_SUCCESS;
	uint16_t loop, i, len1, off, size;
	size = len;

	swap_buffer(tx_buffer, len);
	uint32_t spi_chunk_size = spi_proto->settings.spi_chunk_size;
	loop = (len + spi_chunk_size - 1) / spi_chunk_size; /* Ceil (len / MAX_SPI_BUFF_LEN) */

	off = 0;
	MutexTake(spi_proto->spi_mutex, MAX_DELAY);
	/* Split the IPC Tx/Rx buffer into spi transactions of MAX_SPI_BUFF_LEN */
	for (i = 0; i < loop; i++) {
		len1 = len > spi_chunk_size ? spi_chunk_size : len;
		res = spi_device_transfer(&spi_proto->spi_device, tx_buffer + off, rx_buffer + off, len1);
		if (res)
		{
			LOGE("ERRRRRRR: spi Txn err ret = %d\n", res);
			MutexGive(spi_proto->spi_mutex);
			return res;
		}

		off = off + len1;
		len = len - len1;
		if (len == 0)
			break;
	}
	MutexGive(spi_proto->spi_mutex);
	swap_buffer(rx_buffer, size);
	return res;
}

void ipc_clear(ipc_msg* msg)
{
	memset(msg->buf, 0, IPC_MODE);
}

size_t get_ipc_size(spi_proto_settings* settings)
{
	return sizeof(ipc_msg) + IPC_MODE;
}

ipc_msg* get_ipc_storage(spi_protocol_t* spi_proto) {
	ipc_msg* msg;
	if( !QueueReceive(spi_proto->ipc_storage_pool, &msg, 1000) ) {
		LOGE("Couldn't receive ipc_msg from pool");
		LOGE("Aborting");
		*(int *)0 = 0;
		return NULL;
	}
	return msg;
}

proto_err add_ipc_storage(spi_protocol_t* spi_proto, ipc_msg* msg) {
	size_t ipc_size = get_ipc_size(&spi_proto->settings);
	memset(msg, 0, ipc_size);
	if (QueueSend(spi_proto->ipc_storage_pool, &msg, 0))
		return PROTO_SUCCESS;

	LOGE("Couldn't enqueue storage at %p to the ipc_storage_pool", msg);
	return PROTO_OUT_OF_MEMORY;
}

static bool transfer_done_callback(spi_protocol_t* spi_proto, ipc_msg *msg) {
	bool res = true;
	if(spi_proto->state.sync_msg.custom_callback)
	{
		spiproto_callback_t callback = spi_proto->state.sync_msg.custom_callback->callback;
		void* self = spi_proto->state.sync_msg.custom_callback->owner;
		res = callback(self, msg);
	}
#ifdef TX_TASK
	SemaphoreGive(spi_proto->ipc_msg_sem);
#endif
	return res;
}

void ipc_push(spi_protocol_t* spi_proto, ipc_msg* request) {
	ipc_wrapper req_wrapper = { 0 };
	if (!request) {
		req_wrapper.msg = get_ipc_storage(spi_proto);
		req_wrapper.directly_from_pool = true;
	} else {
		req_wrapper.msg = request;
		req_wrapper.directly_from_pool = false;
		if (req_wrapper.msg->len ==  0)
			LOGE("Ipc length (ipc_msg->len) should be non zero 0x%x 0x%x. May cause protocolar problems",
					request->buf[0], request->buf[1]);
	}

	ipc_wrapper *res_wrapper = &spi_proto->state.response;

	while (req_wrapper.msg) {

		if (!res_wrapper->msg) {
			res_wrapper->msg = get_ipc_storage(spi_proto);
			res_wrapper->directly_from_pool = true;
		}

		if (!req_wrapper.directly_from_pool && !res_wrapper->directly_from_pool)
			LOGD("tx: 0x%x 0x%x ,rx: 0x%x 0x%x ", req_wrapper.msg->buf[0], req_wrapper.msg->buf[1], res_wrapper->msg->buf[0],
					res_wrapper->msg->buf[1]);

		if (spi_proto->state.ipc_mode == SUE_IPC_SIZE_64B && req_wrapper.msg->len > SUE_IPC_SIZE_64B) {
			if (send_mode_chg(spi_proto, &req_wrapper, res_wrapper))
				LOGW("Unsuccessful mode change");
			else
				spi_proto->state.ipc_mode = SUE_IPC_SIZE_4KB;
			wait_for_irq(spi_proto->tx_eg);
			continue;
		}

		bool is_next_4k = is_next_ipc_4k(spi_proto);
		if (is_next_4k)
			SET_FIELD(req_wrapper.msg->buf[1], SUE_R64_W4K, SUE_IPC_MODE_SEL_R4W4_FLD_WIDTH, SUE_IPC_MODE_SEL_R4W4_FLD_OFFSET);

		int32_t err = proto_send(spi_proto, &req_wrapper, res_wrapper, &spi_proto->state.ipc_mode);
		wait_for_irq(spi_proto->tx_eg);
		if (err)
			LOGE("Unsuccessful msg send, err=%d", err);

		if (spi_proto->state.dsp_state == ROM_READY) {
			swap_buffer(req_wrapper.msg->buf, sizeof(uint32_t));
			if (req_wrapper.msg->buf[0] == (ROM_REQUEST_MASK | ROM_FW_LOAD)) {
				if (spi_proto->ctx) {
					load_fw_ctx_t* load_ctx = (load_fw_ctx_t*) spi_proto->ctx;
					uint32_t image_size = load_ctx->fw_loader.image_size;
					ipc_clear(req_wrapper.msg);
					uint8_t *buffer = (uint8_t *) req_wrapper.msg->buf;
					while (image_size > 0) {
						uint16_t requested_size =
								spi_proto->settings.spi_chunk_size < image_size ? spi_proto->settings.spi_chunk_size : image_size;
						uint16_t read_size = load_ctx->fw_loader.read_fw(load_ctx->fw_loader.src, buffer, requested_size);
						if (read_size != requested_size) {
							LOGE("Cannot load fw image chunk, requested: %d read: %d", requested_size, read_size);
							break;
						}
						sha256_update(&load_ctx->sha_ctx, buffer, requested_size);
						spi_transfer(spi_proto, buffer, buffer, requested_size);
						image_size -= requested_size;
					}
					ipc_clear(req_wrapper.msg);
					wait_for_irq(spi_proto->tx_eg);
				} else
					LOGE("FW Load ipc recognized but fw load context is not initialized");
			}
		}

		if (is_next_4k)
			spi_proto->state.ipc_mode = SUE_IPC_SIZE_4KB;

		if (req_wrapper.directly_from_pool) {
			add_ipc_storage(spi_proto, req_wrapper.msg);
		} else if (res_wrapper->msg && is_dsp_req_mode_change(res_wrapper->msg)) {
			//Request and response has to be repeated but in 4K
			swap_buffer(req_wrapper.msg->buf, req_wrapper.msg->len);
			continue;
		} else {
			res_wrapper->directly_from_pool = req_wrapper.directly_from_pool;
			res_wrapper->msg = req_wrapper.msg;
		}

		req_wrapper.msg = NULL;
		req_wrapper.directly_from_pool = false;
	}
}

void push_WI(spi_protocol_t* spi_proto, void *ctx, work_item_callback callback, uint16_t ipc_length) {
	work_item wi = { .self = ctx, .callback = callback, .ipc_length = ipc_length };
	if (!QueueSend(spi_proto->work_queue, &wi, 0))
		LOGE("Sending to work queue failed");
	else
		EventGroupSetBits(spi_proto->tx_eg, MSG_EVENT);
}

void work_(spi_protocol_t* spi_proto) {
	ipc_push(spi_proto, spi_proto->state.sync_msg.msg);
}
proto_err ipc_msg_transfer_cb(spi_protocol_t* spi_proto, ipc_msg *msg, uint32_t time_out_val, spiproto_cb_func_t* custom)
{
	MutexTake(spi_proto->ipc_transfer_mtx, time_out_val);
	msg->self.callback = (spiproto_callback_t) transfer_done_callback;
	msg->self.owner = spi_proto;
	spi_proto->state.sync_msg.msg = msg;
	if (custom)
		spi_proto->state.sync_msg.custom_callback = custom;

	push_WI(spi_proto, spi_proto, (work_item_callback)work_, msg->len);
#ifdef TX_TASK
		bool res = SemaphoreTake(spi_proto->ipc_msg_sem, time_out_val);
		if (!res) {
			LOGE("Message timeout within %d: 0x%x 0x%x", time_out_val, msg->buf[0], msg->buf[1]);
			MutexGive(spi_proto->ipc_transfer_mtx);
			return PROTO_IPC_TIMEOUT;
		}
#else
		 schedule(spi_proto);
#endif
	spi_proto->state.sync_msg.custom_callback = NULL;
	MutexGive(spi_proto->ipc_transfer_mtx);
	return msg->status ? PROTO_IPC_ERR : PROTO_SUCCESS;
}

proto_err ipc_msg_transfer(spi_protocol_t* spi_proto, ipc_msg *msg, uint32_t time_out_val)
{
	return ipc_msg_transfer_cb(spi_proto, msg, time_out_val, NULL);
}

proto_err send_ipc(spi_protocol_t* spi_proto, uint32_t primary, uint32_t extension, uint32_t data_length, const void *data) {
	MutexTake(spi_proto->send_ipc_mtx, SUE_SEND_IPC_TIMEOUT);
	ipc_msg *msg = get_ipc_storage(spi_proto);
	if(!msg)
	{
		MutexGive(spi_proto->send_ipc_mtx);
		return PROTO_OUT_OF_MEMORY;
	}
	struct ipc_hdr header = { .pri = primary, .ext = extension };
	uint32_t header_size = sizeof(header);
	msg->len = header_size + data_length;

	if (msg->len > IPC_MODE) {
		LOGE("Too big to fit into a single transaction: %d ", msg->len);
		add_ipc_storage(spi_proto, msg);
		MutexGive(spi_proto->send_ipc_mtx);
		return PROTO_IGNORE_REQUEST;
	}

	memcpy(msg->buf, &header, header_size);
	if (data_length > 0)
		memcpy((uint8_t *) msg->buf + header_size, data, data_length);

	proto_err res = ipc_msg_transfer(spi_proto, msg, SUE_SEND_IPC_TIMEOUT);
	add_ipc_storage(spi_proto, msg);
	MutexGive(spi_proto->send_ipc_mtx);
	return res;
}

proto_err send_large_cfg_set(spi_protocol_t* spi_proto, uint32_t module, uint32_t instance, uint32_t param_id, uint32_t data_length,
		void *data) {
	uint32_t pri = create_module_msg_hdr_pri(module, instance, MOD_MSG_T_LARGE_CONFIG_SET);
	uint32_t ext = create_lrg_msg_hdr_ext(data_length, param_id, 1, 1);
	return send_ipc(spi_proto, pri, ext, data_length, data);
}

proto_err spi_protocol_destroy(spi_protocol_t* spi_proto) {
	return PROTO_SUCCESS;
}

static inline void send_raw(spi_protocol_t* spi_proto, ipc_msg* msg) {
	uint8_t *buf = (uint8_t *)msg->buf;
	spi_transfer(spi_proto, buf, buf, SUE_IPC_SIZE_64B);
	wait_for_irq(spi_proto->tx_eg);
}

static bool boot_handler(spi_protocol_t* spi_proto, ipc_msg* msg) {
	bool res = true;
	uint32_t pri = ((struct ipc_hdr*) msg->buf)->pri;
	uint32_t type = GET_FIELD(pri, MSG_REPLY_TYPE_FLD_WIDTH, MSG_REPLY_TYPE_FLD_OFFSET);

	if (type == GLB_MSG_T_ROM_CONTROL) {
		uint16_t rom_ctrl_cmd = GET_FIELD(pri, SUE_IPC_ROM_CTRL_CMD_FLD_WIDTH, SUE_IPC_ROM_CTRL_CMD_FLD_OFFSET);
		switch (rom_ctrl_cmd) {
		case SUE_ROM_CTRL_ROM_READY: {
			spi_proto->state.dsp_state = ROM_READY;
#ifdef TX_TASK
			SemaphoreGive(spi_proto->boot_sem);
#endif
			LOGI ("Rom ready received");
			break;
		}
		default:
			res = false;
		}
	} else if (type == GLB_MSG_T_NOTIFICATION) {
		uint32_t notif_type = GET_FIELD(pri, MSG_NOTIF_NOTIF_TYPE_FLD_WIDTH, MSG_NOTIF_NOTIF_TYPE_FLD_OFFSET);
		switch (notif_type) {
		case NOTIF_T_FW_READY: {
			spi_proto->state.dsp_state = FW_READY;
#ifdef TX_TASK
			SemaphoreGive( spi_proto->boot_sem);
#endif
			LOGI ("FW ready received");
			break;
		}
		case NOTIF_T_DFU_READY:
			spi_proto->state.dsp_state = DFU_READY;
#ifdef TX_TASK
			SemaphoreGive( spi_proto->boot_sem);
#endif
			LOGI ("DFU ready received");
			break;
		default:
			res = false;
		}
	} else
		res = false;
	if (res)
		add_ipc_storage(spi_proto, msg);
	return res;
}

void reset_platform(spi_protocol_t* spi_proto, uint32_t boot_mode) {
	LOGI("Resetting DSP (power down)...");
	gpio_set_level(spi_proto->settings.spi_settings.wake_pin, boot_mode == BOOT_SPI_SLAVE ? 1 : 0);
	gpio_set_level(spi_proto->settings.spi_settings.reset_pin, 0);
	TaskDelay(100);
	LOGI("Resetting DSP (power up)...");
	gpio_set_level(spi_proto->settings.spi_settings.reset_pin, 1);
	LOGI("waiting for irq");
	wait_for_irq(spi_proto->tx_eg);
	gpio_set_level(spi_proto->settings.spi_settings.wake_pin, 0);
#ifdef TX_TASK
	if (!CreateTask((void*)work_item_task, "work_item_task", 3072, spi_proto, 15, &spi_proto->work_item_task, 0)) {
			LOGE("error in proto task creation\n");
		}
#else
	ipc_msg* msg = get_ipc_storage(spi_proto);
	int c, retry = 10;
	for(c=1; c <= retry ;++c)
	{
		send_raw(spi_proto, msg);
		if (boot_handler(spi_proto, msg))
			break;

		TaskDelay(50);
		if (c==retry)
			add_ipc_storage(spi_proto, msg);
	}

#endif
#ifdef TX_TASK
	if (!SemaphoreTake(spi_proto->boot_sem, WAIT_FOR_ROM_FW_READY) && !(spi_proto->state.dsp_state & (ROM_READY | FW_READY)))
		LOGE("Dsp in not responsive");
#else
	if (spi_proto->state.dsp_state != FW_READY && spi_proto->state.dsp_state != ROM_READY)
		LOGE("Dsp in not responsive");
	enter_idle(spi_proto);
#endif
}

void change_spi_speed(spi_protocol_t* spi_proto, uint32_t clock) {
	MutexTake(spi_proto->spi_mutex, MAX_DELAY);
	spi_device_set_clock(&spi_proto->spi_device, clock);
	MutexGive(spi_proto->spi_mutex);
}

proto_err fw_load(spi_protocol_t* spi_proto, const fw_loader_t* const fw_loader)
{
	proto_err res = PROTO_SUCCESS;

	if (spi_proto->state.dsp_state == FW_READY) {
		LOGE("FW already loaded");
		return PROTO_IGNORE_REQUEST;
	}

	load_fw_ctx_t fw_load_ctx = { 0 };
	spi_proto->ctx  = &fw_load_ctx;
	memcpy(&fw_load_ctx.fw_loader, fw_loader, sizeof(fw_loader_t));
	SHA256_T* sha_ctx = &fw_load_ctx.sha_ctx;
	sha256_init(sha_ctx);

	ipc_msg* msg = get_ipc_storage(spi_proto);
	rom_load_fw_ipc(msg, fw_load_ctx.fw_loader.address, fw_load_ctx.fw_loader.image_size, 0);
	res = ipc_msg_transfer(spi_proto, msg, SUE_LOAD_FW_TIMEOUT);
	if(res)
	{
		spi_proto->ctx = NULL;
		return res;
	}

	uint8_t sha_buffer [32] = { 0 };
	sha256_finish(&fw_load_ctx.sha_ctx, sha_buffer);
	if (memcmp(msg->buf + 5, sha_buffer, 32) != 0)
	{
		LOGE("Received sha256 is different as expected - content mismatch");
		add_ipc_storage(spi_proto, msg);
		spi_proto->ctx = NULL;
		return  PROTO_SHA_MISMATCH;
	}

	ipc_clear(msg);
	rom_exec_ipc(msg, fw_load_ctx.fw_loader.address);

	spiproto_cb_func_t cb;
	cb.callback = (spiproto_callback_t)boot_handler;
	cb.owner = spi_proto;

	ipc_msg_transfer_cb(spi_proto, msg, SUE_SEND_IPC_TIMEOUT, &cb);

#ifdef TX_TASK
	if (!SemaphoreTake( spi_proto->boot_sem, WAIT_FOR_ROM_FW_READY))
#else
	if (spi_proto->state.dsp_state != FW_READY)
#endif
	{
		LOGE("Fw ready not received");
		res = PROTO_DSP_NOT_RESPONSIVE;
	}
	spi_proto->ctx = NULL;
	return res;
}

