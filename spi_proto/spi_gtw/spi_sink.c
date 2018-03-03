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
#include "spi_gtw/spi_sink.h"

#define SET_DATA_HEADER_SIZE 3 * sizeof(uint32_t)
static const char *LOG_TAG = "AudDrv-SPISink";
static bool spi_sink_on_response(spi_obj_handle_t* spi_obj, ipc_msg* msg);


static bool is_data_available(spi_obj_handle_t* spi_obj)
{
	if (spi_obj->transfer_buf.size - spi_obj->transfer_buf.used_size == 0)
	{
		if (spi_obj->state == SPI_OBJ_STATE_READ_WRITE || spi_obj->state == SPI_OBJ_STATE_START_DONE
				|| spi_obj->state == SPI_OBJ_STATE_DRAINING_DONE)
			if(spi_obj->event_callbacks[DSP_EVENT_REQUEST_COMPLETED].callback != NULL)
					spi_obj->event_callbacks[DSP_EVENT_REQUEST_COMPLETED].callback(spi_obj->event_callbacks[DSP_EVENT_REQUEST_COMPLETED].user_param, DSP_EVENT_REQUEST_COMPLETED, (void*)&spi_obj->transfer_buf);

		spi_obj->state = SPI_OBJ_STATE_BUFFER_PROCESSED;
		return false;
	}
	return  true;
}

static void push_data(spi_obj_handle_t* spi_obj)
{
	ipc_msg* msg =(ipc_msg*) get_ipc_storage(spi_obj->spi_proto);
	MutexTake(spi_obj->cs_mutex, MAX_DELAY);
	SemaphoreTake(spi_obj->stop_mutex, 0);
	if(spi_obj->state >= SPI_OBJ_STATE_STOP) 	{
		add_ipc_storage(spi_obj->spi_proto, msg);
		MutexGive(spi_obj->cs_mutex);
		return;
	}
	int max_bytes_in_packet = IPC_MODE - SET_DATA_HEADER_SIZE;
	int free_bytes = spi_obj->transfer_buf.size - spi_obj->transfer_buf.used_size;
	int read_bytes = free_bytes > max_bytes_in_packet ? max_bytes_in_packet : free_bytes;

	memcpy((uint8_t*) msg->buf + sizeof(struct ipc_hdr) +sizeof(uint32_t), spi_obj->transfer_buf.ptr + spi_obj->transfer_buf.used_size, read_bytes);

#if AUDIO_DETECT_XRUN
	if (read_bytes < IPC_MODE)
		if(spi_obj->event_callbacks[DSP_EVENT_UNDERRUN].callback != NULL)
			spi_obj->event_callbacks[DSP_EVENT_UNDERRUN].callback(spi_obj->event_callbacks[DSP_EVENT_UNDERRUN].user_param, DSP_EVENT_UNDERRUN, NULL);
#endif
	if (spi_obj->state == SPI_OBJ_STATE_BUFFER_PROCESSED) {
		spi_obj->state = SPI_OBJ_STATE_READ_WRITE;
	}
	spi_obj->no_buffer_on_notif = false;
	MutexGive(spi_obj->cs_mutex);
	msg->buf[0] = msg_hdr_pri(IPC_GTW_CMD_SET_DATA, 0, GLB_MSG_T_IPC_GTW_CMD, MSG_TGT_FW_GEN_MSG);
	msg->buf[1] = read_bytes;
	msg->buf[2] = spi_obj->gtw_id;
	msg->len = SET_DATA_HEADER_SIZE + read_bytes;
	msg->self.callback = (spiproto_callback_t)spi_sink_on_response;
	msg->self.owner = spi_obj;

	ipc_push(spi_obj->spi_proto, msg);
}



static bool spi_sink_on_notif(spi_obj_handle_t* spi_obj, ipc_msg* msg)
{
	if(msg->buf[3] == spi_obj->gtw_id)
	{
		MutexTake(spi_obj->cs_mutex, MAX_DELAY);
		if (spi_obj->trigger == SUE_STREAM_TRIGGER_ON_NOTIFICATION)
		{
			if(is_data_available(spi_obj))
			{
				spi_obj->trigger = SUE_STREAM_TRIGGER_ON_RESPONSE;
				push_WI(spi_obj->spi_proto, spi_obj, (work_item_callback)push_data, IPC_MODE);
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

static bool spi_sink_on_response(spi_obj_handle_t* spi_obj, ipc_msg* msg)
{
	uint16_t data_written = msg->buf[2] >> 16;
	uint16_t data_left = msg->buf[2] & 0xFFFF;
	MutexTake(spi_obj->cs_mutex, MAX_DELAY);
	if (data_written > 0)
		spi_obj->transfer_buf.used_size += data_written;

	bool is_data = is_data_available(spi_obj);
	if (data_left < IPC_MODE)
	{
		spi_obj->trigger = SUE_STREAM_TRIGGER_ON_NOTIFICATION;
		MutexGive(spi_obj->cs_mutex);
		return false;
	}

	MutexGive(spi_obj->cs_mutex);
	if (is_data)
		push_WI(spi_obj->spi_proto, spi_obj, (work_item_callback)push_data, IPC_MODE);
	add_ipc_storage(spi_obj->spi_proto, msg);
	return true;
}


static bool spi_sink_overrun_handler(spi_obj_handle_t* spi_obj, ipc_msg* msg)
{
	uint32_t resource_type = msg->buf[2];
	uint32_t resource_id = msg->buf[3];
	uint32_t gtw_id = msg->buf[6];
	if( (resource_type == RESOURCE_T_PIPELINE) && (gtw_id == 0 || gtw_id == spi_obj->gtw_id)) {
		LOGW("GTW OVERRUN DETECTED! pipeline id: %#x, node id: %#x", resource_id, gtw_id);
		add_ipc_storage(spi_obj->spi_proto, msg);
		{ //critical section
			MutexTake(spi_obj->cs_mutex, MAX_DELAY);
			if(spi_obj->state == SPI_OBJ_STATE_STOP || spi_obj->state == SPI_OBJ_STATE_BUFFER_PROCESSED)
				SemaphoreGive(spi_obj->stop_mutex);
			if(spi_obj->state != SPI_OBJ_STATE_STOP_DONE)
				spi_obj->state = SPI_OBJ_STATE_DRAINING_DONE;
			MutexGive(spi_obj->cs_mutex);
		}
		return true;
	}
	return false;
}

proto_err spi_sink_deinit(spi_obj_handle_t* spi_obj) {
	if(spi_obj->state != SPI_OBJ_STATE_DESTROYED && spi_obj->state != SPI_OBJ_STATE_STOP_DONE && spi_obj->state != SPI_OBJ_STATE_UNSET) {
		return PROTO_GTW_INVALID_STATE;
	}

	SemaphoreDelete(spi_obj->stop_mutex);
#ifdef TX_TASK
	MutexDelete(spi_obj->cs_mutex);
#endif

	return PROTO_SUCCESS;
}

proto_err spi_sink_init(spi_obj_handle_t* spi_obj, spi_protocol_t* spi_proto) {

	if(!spi_obj)
	{
		LOGE("Spi obj nullptr");
		return PROTO_NULL_POINTER;
	}
	else
		memset(spi_obj, 0, sizeof(spi_obj_handle_t));

	spi_obj->spi_proto = spi_proto;
	spi_obj->stop_mutex = CreateSemaphore();
	#ifdef TX_TASK
		spi_obj->cs_mutex = CreateMutex();
	#endif
	spi_obj->state = SPI_OBJ_STATE_UNSET;

	if( spi_obj->state != SPI_OBJ_STATE_UNSET
		&& spi_obj->state != SPI_OBJ_STATE_DESTROYED
		&& spi_obj->state != SPI_OBJ_STATE_STOP_DONE )
		return PROTO_GTW_INVALID_STATE;

	spi_proto_handler_add(spi_obj->spi_proto, (spiproto_callback_t)spi_sink_on_notif, spi_obj, NOTIF_T_RESOURCE_EVENT);
	spi_proto_handler_add(spi_obj->spi_proto, (spiproto_callback_t)spi_sink_overrun_handler, spi_obj, NOTIF_T_RESOURCE_EVENT);

	return PROTO_SUCCESS;
}

proto_err spi_sink_write(spi_obj_handle_t* spi_obj, uint8_t* src_addr, uint32_t size, uint32_t** written)
{
	MutexTake(spi_obj->cs_mutex, MAX_DELAY);
	if (spi_obj->state != SPI_OBJ_STATE_BUFFER_PROCESSED && spi_obj->state != SPI_OBJ_STATE_DRAINING_DONE)
	{
		LOGW("Sink hasn't been started nor drained. Current state: %d.", spi_obj->state);
		MutexGive(spi_obj->cs_mutex);
		return PROTO_GTW_INVALID_STATE;
	}

	spi_obj->transfer_buf.ptr = src_addr;
	spi_obj->transfer_buf.size = size;
	spi_obj->transfer_buf.used_size = 0;
	*written = &spi_obj->transfer_buf.used_size;


	LOGD("Spi Sink : reqd size to write %d",size);
	if (spi_obj->no_buffer_on_notif || spi_obj->trigger == SUE_STREAM_TRIGGER_ON_RESPONSE)
	{
		spi_obj->trigger = SUE_STREAM_TRIGGER_ON_RESPONSE;
		push_WI(spi_obj->spi_proto, spi_obj, (work_item_callback)push_data, IPC_MODE);
	}

	if(spi_obj->state != SPI_OBJ_STATE_STOP)
		spi_obj->state = SPI_OBJ_STATE_READ_WRITE;
	LOGD("spi_sink_write exit %d", size);

	MutexGive(spi_obj->cs_mutex);
	return PROTO_SUCCESS;
}

proto_err spi_sink_start(spi_obj_handle_t* spi_obj){
	spi_obj->trigger = SUE_STREAM_TRIGGER_ON_NOTIFICATION;

	if(spi_obj->state != SPI_OBJ_STATE_UNSET && spi_obj->state != SPI_OBJ_STATE_BUFFER_PROCESSED && spi_obj->state != SPI_OBJ_STATE_STOP_DONE)
		return PROTO_GTW_INVALID_STATE;

	spi_obj->state = SPI_OBJ_STATE_START_DONE;
	return PROTO_SUCCESS;
}

proto_err spi_sink_stop(spi_obj_handle_t* spi_obj) {
	if(spi_obj->state != SPI_OBJ_STATE_BUFFER_PROCESSED
		&& spi_obj->state != SPI_OBJ_STATE_UNSET)
		return PROTO_GTW_INVALID_STATE;

	proto_err status = PROTO_SUCCESS;
	{ //critical section
		MutexTake(spi_obj->cs_mutex, MAX_DELAY);
		spi_obj->state = SPI_OBJ_STATE_STOP;
		gtw_audio_format_t* format = &spi_obj->audio_format;
		//We assume that max buffer size is in use
		uint32_t wait_time = ((12 * 4096) / ((format->sampl_freq/1000)*(format->bit_depth/8)*format->num_channels));
		MutexGive(spi_obj->cs_mutex); //need to exit critical section before calling any functions
		if (!SemaphoreTake(spi_obj->stop_mutex, wait_time))
			status = PROTO_GTW_TIMEOUT;

	}
	if( PROTO_SUCCESS == status )
			spi_obj->state = SPI_OBJ_STATE_STOP_DONE;

	spi_proto_handler_remove(spi_obj->spi_proto, (spiproto_callback_t)spi_sink_on_notif, spi_obj);
	spi_proto_handler_remove(spi_obj->spi_proto, (spiproto_callback_t)spi_sink_overrun_handler, spi_obj);

	return status;
}

proto_err spi_sink_seteventcallback(spi_obj_handle_t* spi_obj, void *user_param,
	dsp_event_t event_type, event_callback_t callback)
{
	proto_err ret_val = PROTO_SUCCESS;

	if(callback != NULL) {
		if (event_type >= DSP_EVENT_UNDERRUN && event_type <= DSP_EVENT_FATAL_ERROR)
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
