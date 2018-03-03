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
#include "spi_gtw/spi_source.h"

#define GET_DATA_HEADER_SIZE 3 * sizeof(uint32_t)
#define MAX_SMALL_BUF_SIZE 64 - GET_DATA_HEADER_SIZE
#define MAX_BYTES_IN_PACKET IPC_MODE - GET_DATA_HEADER_SIZE
static const char *LOG_TAG = "AudDrv-SPISrc";

static bool spi_source_on_response(spi_obj_handle_t* spi_obj, ipc_msg* msg);

static void run_callback(callback_info_t *callback_arr, dsp_event_t callback_id, void *param)
{
	callback_info_t *cbinfo = &callback_arr[callback_id];
	if (cbinfo->callback) {
		cbinfo->callback(cbinfo->user_param, callback_id, param);
	}
}

static bool is_space_available(spi_obj_handle_t* spi_obj)
{
	if(spi_obj->transfer_buf.size - spi_obj->transfer_buf.used_size == 0)
	{
		if (spi_obj->state == SPI_OBJ_STATE_READ_WRITE || spi_obj->state == SPI_OBJ_STATE_START_DONE)
			run_callback(spi_obj->event_callbacks, DSP_EVENT_REQUEST_COMPLETED, (void*)&spi_obj->transfer_buf);

		spi_obj->state = SPI_OBJ_STATE_BUFFER_PROCESSED;
		return false;
	}
	return true;
}

static void pull_data(spi_obj_handle_t* spi_obj)
{
	MutexTake(spi_obj->cs_mutex, MAX_DELAY);
	int free_bytes = spi_obj->transfer_buf.size - spi_obj->transfer_buf.used_size;
	int read_bytes = free_bytes > MAX_BYTES_IN_PACKET ? MAX_BYTES_IN_PACKET : free_bytes;


	if (spi_obj->state == SPI_OBJ_STATE_BUFFER_PROCESSED)
		spi_obj->state = SPI_OBJ_STATE_READ_WRITE;
	spi_obj->no_buffer_on_notif = false;
	MutexGive(spi_obj->cs_mutex);

	ipc_msg *msg = get_ipc_storage(spi_obj->spi_proto);
	msg->buf[0] = msg_hdr_pri(IPC_GTW_CMD_GET_DATA, 0, GLB_MSG_T_IPC_GTW_CMD, MSG_TGT_FW_GEN_MSG);
	//LOGW("packet: %d, free %d, size %d, used %d, read %d", spi_obj->spi_proto->settings.packet_size, free_bytes, spi_obj->transfer_buf.size, spi_obj->transfer_buf.used_size, read_bytes);
	msg->buf[1] = (read_bytes > MAX_SMALL_BUF_SIZE ? SUE_R64_W4K << 30 : 0) | (read_bytes+4);
	msg->buf[2] = spi_obj->gtw_id;
	msg->len = 3 * sizeof(uint32_t);
	msg->self.callback = (spiproto_callback_t)spi_source_on_response;
	msg->self.owner = spi_obj;

	ipc_push(spi_obj->spi_proto, msg);
}

static bool spi_source_on_notif(spi_obj_handle_t* spi_obj, ipc_msg* msg)
{
	if(msg->buf[3] == spi_obj->gtw_id)
	{
		MutexTake(spi_obj->cs_mutex, MAX_DELAY);
		if (spi_obj->trigger == SUE_STREAM_TRIGGER_ON_NOTIFICATION)
		{
			if(is_space_available(spi_obj))
			{
				spi_obj->trigger = SUE_STREAM_TRIGGER_ON_RESPONSE;
				push_WI(spi_obj->spi_proto, spi_obj, (work_item_callback)pull_data, IPC_MODE);
			}
			else
				spi_obj->no_buffer_on_notif = true;
		}

		MutexGive(spi_obj->cs_mutex);
		add_ipc_storage(spi_obj->spi_proto, msg);
		return true;
	}
	return false;
}

static bool spi_source_on_response(spi_obj_handle_t* spi_obj, ipc_msg* msg)
{
	uint32_t gtw_data_left = msg->buf[2] & 0xFFFF;
	uint32_t received_data_size = (msg->buf[1] & 0xFFFF) - 4;

	MutexTake(spi_obj->cs_mutex, MAX_DELAY);
	if (received_data_size > 0)
	{
		int free_bytes = spi_obj->transfer_buf.size - spi_obj->transfer_buf.used_size;
		if (free_bytes < received_data_size)
		{
			run_callback(spi_obj->event_callbacks, DSP_EVENT_OVERRUN, NULL);
			spi_obj->trigger = SUE_STREAM_TRIGGER_ON_NOTIFICATION;
			MutexGive(spi_obj->cs_mutex);
			return false;
		}
		memcpy(spi_obj->transfer_buf.ptr + spi_obj->transfer_buf.used_size, (uint8_t*) msg->buf + sizeof(struct ipc_hdr) + sizeof(uint32_t), received_data_size);
		spi_obj->transfer_buf.used_size += received_data_size;

		LOGD("bytes written %d", received_data_size);
	}

	bool is_space = is_space_available(spi_obj);
	if (gtw_data_left < MAX_BYTES_IN_PACKET)
	{
		spi_obj->trigger = SUE_STREAM_TRIGGER_ON_NOTIFICATION;
		MutexGive(spi_obj->cs_mutex);
		return false;
	}
	if(is_space)
		push_WI(spi_obj->spi_proto, spi_obj, (work_item_callback)pull_data, IPC_MODE);

	MutexGive(spi_obj->cs_mutex);
	add_ipc_storage(spi_obj->spi_proto, msg);
	return true;
}

proto_err spi_source_deinit(spi_obj_handle_t* spi_obj)
{
	if(spi_obj->state != SPI_OBJ_STATE_DESTROYED
			&& spi_obj->state != SPI_OBJ_STATE_STOP_DONE
			&& spi_obj->state != SPI_OBJ_STATE_UNSET)
		return PROTO_GTW_INVALID_STATE;

#ifdef TX_TASK
	MutexDelete(spi_obj->cs_mutex);
#endif
	return PROTO_SUCCESS;
}

proto_err spi_source_init(spi_obj_handle_t* spi_obj, spi_protocol_t* spi_proto)
{
	if(!spi_obj)
	{
		LOGE("Spi obj nullptr");
		return PROTO_NULL_POINTER;
	}
	else
		memset(spi_obj, 0, sizeof(spi_obj_handle_t));

	spi_obj->spi_proto = spi_proto;
#ifdef TX_TASK
	spi_obj->cs_mutex = CreateMutex();
#endif
	spi_obj->state = SPI_OBJ_STATE_UNSET;
	memset(spi_obj->event_callbacks, 0, sizeof(spi_obj->event_callbacks));

	return PROTO_SUCCESS;
}

proto_err spi_source_read(spi_obj_handle_t* spi_obj, uint8_t* dst_addr, uint32_t req_size, uint32_t** read)
{
	MutexTake(spi_obj->cs_mutex, MAX_DELAY);
	if (spi_obj->state != SPI_OBJ_STATE_BUFFER_PROCESSED)
	{
		MutexGive(spi_obj->cs_mutex);
		LOGW("Source hasn't been started or is still filling data buffer. Current state: %d.", spi_obj->state);
		return PROTO_GTW_INVALID_STATE;
	}

	spi_obj->transfer_buf.ptr = dst_addr;
	spi_obj->transfer_buf.size = req_size;
	spi_obj->transfer_buf.used_size = 0;
	*read = &spi_obj->transfer_buf.used_size;

	if (spi_obj->no_buffer_on_notif || spi_obj->trigger == SUE_STREAM_TRIGGER_ON_RESPONSE)
	{
		spi_obj->trigger = SUE_STREAM_TRIGGER_ON_RESPONSE;
		push_WI(spi_obj->spi_proto, spi_obj, (work_item_callback)pull_data, IPC_MODE);
	}

	if(spi_obj->state != SPI_OBJ_STATE_STOP)
		spi_obj->state = SPI_OBJ_STATE_READ_WRITE;
	MutexGive(spi_obj->cs_mutex);

	return PROTO_SUCCESS;
}

proto_err spi_source_start(spi_obj_handle_t* spi_obj)
{
	if(spi_obj->state != SPI_OBJ_STATE_UNSET
			&& spi_obj->state != SPI_OBJ_STATE_BUFFER_PROCESSED
			&& spi_obj->state != SPI_OBJ_STATE_STOP_DONE)
		return PROTO_GTW_INVALID_STATE;

	// arm for data notification
	spi_obj->trigger = SUE_STREAM_TRIGGER_ON_NOTIFICATION;
	spi_proto_handler_add(spi_obj->spi_proto, (spiproto_callback_t)spi_source_on_notif, spi_obj, NOTIF_T_RESOURCE_EVENT);

	spi_obj->state = SPI_OBJ_STATE_START_DONE;

	return PROTO_SUCCESS;
}

proto_err spi_source_stop(spi_obj_handle_t* spi_obj)
{
	if(spi_obj->state != SPI_OBJ_STATE_UNSET
			&& spi_obj->state != SPI_OBJ_STATE_BUFFER_PROCESSED
			&& spi_obj->state != SPI_OBJ_STATE_START_DONE)
	{
		LOGE("Status %d ",spi_obj->state);
		return PROTO_GTW_INVALID_STATE;
	}


	spi_proto_handler_remove(spi_obj->spi_proto, (spiproto_callback_t)spi_source_on_notif, spi_obj);

	spi_obj->state = SPI_OBJ_STATE_STOP_DONE;

	return PROTO_SUCCESS;
}

proto_err spi_source_seteventcallback(spi_obj_handle_t* spi_obj, void *user_param,
	dsp_event_t event_type, event_callback_t callback)
{
	proto_err ret_val = PROTO_SUCCESS;
	LOGD("WOV: entered spi_source_set_wov ");
	if(callback != NULL) {
		if (event_type >= DSP_EVENT_UNDERRUN && event_type < DSP_EVENTS_COUNT)
		{
			spi_obj->event_callbacks[event_type].user_param = user_param;
			spi_obj->event_callbacks[event_type].callback = callback;
		}else
		{
			LOGE("Unsupported event %d",event_type);
			ret_val = PROTO_INVALID_PARAM;
		}
	}
	return ret_val;
}

