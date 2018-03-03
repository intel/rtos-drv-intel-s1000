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
#include "dsp_driver.h"
#include "mfg_driver.h"
#include "crb_defs.h"
#include "spi_gtw/spi_source.h"
#include "spi_gtw/spi_sink.h"
#include "spi_protocol.h"
#include "ipc_msg.h"

#define SOURCE_GTW 0x1400
#define SINK_GTW 0x1500

typedef struct {
	spi_protocol_t spi_proto;
	evt_cb_ops_t* cb;
	spi_obj_handle_t source;
	spi_obj_handle_t sink;
	dsp_capture_state_t c_state;
} dsp_dev_t;

static dsp_dev_t dsp_dev = {0};

static const char *LOG_TAG = "dsp_driver";

void copy_response_data(ipc_msg* msg, buf_desc_t* bd);

static proto_err set_retention_delay(dsp_dev_t *dsp_dev) {
	const uint32_t delay_data[] = {
		IPC_SC_DELAY_ADDR1, IPC_SC_DELAY_VALUE1,
		IPC_SC_DELAY_ADDR2, IPC_SC_DELAY_VALUE2,
		IPC_SC_DELAY_ADDR3, IPC_SC_DELAY_VALUE3,
		IPC_SC_DELAY_ADDR4, IPC_SC_DELAY_VALUE4,
	};
	proto_err is_error = send_ipc(&dsp_dev->spi_proto, IPC_REQUEST_MASK|IPC_ROM_CTRL_MEM_WR, 8, 8, delay_data);
	/* Set retention delay. */
	if (is_error)
		LOGE("Setting Retention Delay failed");
	 else
		LOGI("Retention Delay set");

	return is_error;
}

proto_err dsp_driver_init(irq_handler_t isr, uint32_t boot_mode)
{
	uint32_t spi_chunk_size = IPC_MODE > MAX_SPI_BUFF_LEN ? MAX_SPI_BUFF_LEN : IPC_MODE;
	spi_proto_settings settings = {
			.spi_settings = {
					.miso_pin = SPI_MISO,
					.mosi_pin = SPI_MOSI,
					.clock_pin = SPI_CLK,
					.cs_pin = SPI_CS_HW,
					.irq_pin = GPIO_IRQ,
					.reset_pin = GPIO_RST,
					.itnitial_clock = 9000000,
					.wake_pin = GPIO_WAKE,
#ifndef TX_TASK
					.isr_handler = isr,
#endif
			},
			.spi_chunk_size = spi_chunk_size,
			.proto_version = VER_WAKE_1,
	};

	spi_protocol_t* proto =  &dsp_dev.spi_proto;
	proto_err ret = spi_protocol_init(proto, &settings);
	RET_ON_ERROR(ret);

	reset_platform(proto, boot_mode);
	dsp_state_t dsp_state = proto->state.dsp_state;
	if (dsp_state == UNKNOWN)
		return PROTO_IPC_ERR;

	if (dsp_state == ROM_READY)
		set_retention_delay(&dsp_dev);

	return PROTO_SUCCESS;
}

proto_err dsp_driver_deinit()
{
	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_TOPOLOGY_STOP, 0, NULL);
	if(is_error)
		LOGE("Failed to stop topology");

	if (dsp_dev.cb->notif_callbacks[NOTIF_WOV_DETECTED].callback)
	{
		is_error = unregister_notif_event(&dsp_dev.spi_proto, NOTIF_WOV_DETECTED);
		RET_ON_PROTO_ERROR_LOG(is_error, "Failed to unregister wov notification");
	}
	return PROTO_SUCCESS;
}

proto_err dsp_driver_configure(dsp_dialect_t dialect, evt_cb_ops_t* cb_ops)
{
	dsp_dev.cb = cb_ops;
	if (IPC_MODE == SUE_IPC_SIZE_64B)
	{
		uint32_t mode = IPC64_EXCLUSIVE;
		proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_IPC_MODE, sizeof(mode), &mode);
		RET_ON_PROTO_ERROR_LOG(is_error, "could not set ipc64 mode");
	}
	if (dialect != DIALECT_NOT_SPECIFIED)
	{
		//select config blob
		uint32_t idx = dialect;
		proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_CONFIG_SELECT, sizeof(idx), &idx);
		RET_ON_PROTO_ERROR_LOG(is_error, "selecting config from flash failed");
	}
	return PROTO_SUCCESS;
}

proto_err dsp_driver_start()
{
	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_TOPOLOGY_START, 0, NULL);
	RET_ON_PROTO_ERROR_LOG(is_error, "Topology start failed");

	dsp_dev.c_state = DSP_CAPTURE_STATE_IDLE;

	is_error = spi_source_init(&dsp_dev.source, &dsp_dev.spi_proto);
	dsp_dev.source.gtw_id = SOURCE_GTW;
	RET_ON_PROTO_ERROR_LOG(is_error, "spi_source_setup failed");

	if (dsp_dev.cb->notif_callbacks[NOTIF_WOV_DETECTED].callback)
		is_error = register_notif_event(&dsp_dev.spi_proto, NOTIF_WOV_DETECTED, dsp_dev.cb->notif_callbacks[NOTIF_WOV_DETECTED]);
	RET_ON_PROTO_ERROR_LOG(is_error, "register_notif_event failed");

	if (dsp_dev.cb->gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED].callback)
		is_error = spi_source_seteventcallback(&dsp_dev.source, dsp_dev.cb->gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED].user_param, DSP_EVENT_REQUEST_COMPLETED, dsp_dev.cb->gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED].callback);
	RET_ON_PROTO_ERROR_LOG(is_error, "spi_source_seteventcallback failed");

	return PROTO_SUCCESS;
}

proto_err dsp_driver_mfg_start()
{
	proto_err is_error = PROTO_SUCCESS;

	if (dsp_dev.cb->notif_callbacks[NOTIF_MFG_TEST_END].callback)
                is_error = register_notif_event(&dsp_dev.spi_proto, NOTIF_MFG_TEST_END, dsp_dev.cb->notif_callbacks[NOTIF_MFG_TEST_END]);
	RET_ON_PROTO_ERROR_LOG(is_error, "register_notif_event failed");

	return PROTO_SUCCESS;
}

proto_err dsp_driver_read_data(unsigned int stream_id, void *buffer, unsigned int req_size, uint32_t **read)
{
	return spi_source_read(&dsp_dev.source, (uint8_t*)buffer, req_size, read);
}

proto_err dsp_driver_write_data(unsigned int stream_id, void *buffer, unsigned int size, uint32_t **written)
{
	return spi_sink_write(&dsp_dev.sink, (uint8_t*)buffer, size, written);
}

proto_err dsp_driver_schedule()
{
#ifndef TX_TASK
	schedule(&dsp_dev.spi_proto);
#endif
	return PROTO_SUCCESS;
}

proto_err dsp_driver_set_state(dsp_capture_state_t capture_state)
{
	proto_err is_error;
	uint32_t state = capture_state;

	switch(state)
	{
	case DSP_CAPTURE_STATE_IDLE:
		if (dsp_dev.c_state != DSP_CAPTURE_STATE_IDLE)
		{
			is_error = spi_source_stop(&dsp_dev.source);
			RET_ON_PROTO_ERROR_LOG(is_error, "Failed to stop capture stream");
		}
		break;
	case DSP_CAPTURE_STATE_WOV:
	case DSP_CAPTURE_STATE_STREAM:
		if (dsp_dev.c_state == DSP_CAPTURE_STATE_IDLE)
		{
			is_error = spi_source_start(&dsp_dev.source);
			RET_ON_PROTO_ERROR_LOG(is_error, "spi_source_start failed");
		}
		break;
	case DSP_CAPTURE_STATE_RESET:
		break;
	default:
		RET_ON_PROTO_ERROR_LOG(true, "Invalid capture state %d", capture_state);
		break;
	}

	is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_CAPTURE_SET_STATE, sizeof(state), &state);
	RET_ON_PROTO_ERROR_LOG(is_error, "Setting captute state %d failed", capture_state);

	dsp_dev.c_state = capture_state;

	return PROTO_SUCCESS;
}

proto_err dsp_driver_start_stream(unsigned int stream_id, gtw_audio_format_t *audio_format, callback_info_t gtw_event_callbacks[])
{
	if (stream_id != REFERENCE_STREAM_ID)
	{
		proto_err is_error = spi_sink_init(&dsp_dev.sink, &dsp_dev.spi_proto);
		dsp_dev.sink.gtw_id = SINK_GTW;
		dsp_dev.sink.audio_format = *audio_format;
		RET_ON_PROTO_ERROR_LOG(is_error, "spi_sink_setup failed");

		if (gtw_event_callbacks && gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED].callback)
			spi_source_seteventcallback(&dsp_dev.sink, gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED].user_param, DSP_EVENT_REQUEST_COMPLETED, gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED].callback);

		is_error = spi_sink_start(&dsp_dev.sink);
		RET_ON_PROTO_ERROR_LOG(is_error, "spi_sink_start failed");
	}

	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_STREAM_START, sizeof(stream_id), &stream_id);
	RET_ON_PROTO_ERROR_LOG(is_error, "Start stream id %d failed", stream_id);

	return PROTO_SUCCESS;
}

proto_err dsp_driver_stop_stream(uint32_t stream_id)
{
	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_STREAM_STOP, sizeof(stream_id), &stream_id);
	RET_ON_PROTO_ERROR_LOG(is_error, "Stop stream id %d failed", stream_id);

	if (stream_id != REFERENCE_STREAM_ID)
	{
		is_error = spi_sink_stop(&dsp_dev.sink);
		RET_ON_PROTO_ERROR_LOG(is_error, "spi_sink_stop failed");
	}

	return PROTO_SUCCESS;
}

typedef struct
{
	read_img_chunk_cb read_img_cb;
	void *read_img_ctx;
	uint32_t *remaining_size;
	uint32_t *cumul_sent_size;
	uint32_t cmd;
	proto_err res;
	bool done;
} img_loader_t;

static bool img_load_on_response(img_loader_t *ctx, ipc_msg* msg);

static void push_ipc(img_loader_t* ctx) {
	ipc_msg *msg = get_ipc_storage(&dsp_dev.spi_proto);
	uint16_t max_packet_size = IPC_MODE - sizeof(struct mega_cfg_ipc_hdr);
	uint16_t requested_size = max_packet_size < *ctx->remaining_size ? max_packet_size : *ctx->remaining_size;
	uint8_t *ptr = (uint8_t*)msg->buf + sizeof(struct mega_cfg_ipc_hdr);
	uint16_t read_size = ctx->read_img_cb(ctx->read_img_ctx, ptr, requested_size);

	if (read_size != requested_size)
	{
		LOGE("Cannot load image chunk, requested: %d read: %d", requested_size, read_size);
		ctx->res = PROTO_IPC_ERR;
		return;
	}
	sue_mega_cfg_set_msg(msg, SUE_MGMT_SRVC_MOD_ID, 0, ctx->cmd,
			ctx->remaining_size, ctx->cumul_sent_size, ptr - *ctx->cumul_sent_size,
			IPC_MODE);
	msg->self.callback = (spiproto_callback_t) img_load_on_response;
	msg->self.owner = ctx;
	ipc_push(&dsp_dev.spi_proto, msg);
}

static bool img_load_on_response(img_loader_t *ctx, ipc_msg* msg)
{
	if (*ctx->remaining_size == 0)
	{
		ctx->done = true;
		ctx->res = PROTO_SUCCESS;
	}
	else
	{
		push_WI(&dsp_dev.spi_proto, ctx, (work_item_callback)push_ipc, IPC_MODE);
	}

	if (msg->status)
	{
		ctx->done = true;
		ctx->res = PROTO_IPC_ERR;
	}

	add_ipc_storage(&dsp_dev.spi_proto, msg);
	return true;
}

static proto_err img_load(img_loader_t *img_loader)
{
	int i = 0;

	push_WI(&dsp_dev.spi_proto, img_loader, (work_item_callback)push_ipc, IPC_MODE);

	while(!img_loader->done)
	{
		dsp_driver_schedule();
		if (i++ > SUE_BOOT_TIMEOUT)
		{
			LOGE("Timeout while loading image.");
			return PROTO_IPC_ERR;
		}
		TaskDelay(1);
	}
	return img_loader->res;
}

proto_err dsp_driver_load_img(load_function_t function, read_img_chunk_cb cb, void* ctx, uint32_t img_size, void* user_param)
{
	proto_err err = PROTO_SUCCESS;
	fw_loader_t fw_loader;
	img_loader_t img_loader;
	uint32_t remaining = img_size;
	uint32_t cumul_sent_size = 0;
	int i = 0;
	ipc_msg *msg;
	switch(function)
	{
		case LOAD_FUNC_BOOTSTRAP:
			fw_loader = (fw_loader_t){.image_size = img_size, .address = IPC_SC_IMG_ADDR, .read_fw = cb, .src = ctx };
			err = fw_load(&dsp_dev.spi_proto, &fw_loader);
			RET_ON_PROTO_ERROR_LOG(err, "Cannot load fw img");
			break;

		case LOAD_FUNC_CFG_LOAD:
		case LOAD_FUNC_CFG_STORE:
			img_loader = (img_loader_t){.read_img_cb = cb, .read_img_ctx = ctx, .remaining_size = &remaining, .cumul_sent_size = &cumul_sent_size, .cmd = SUE_CMD_CONFIG_LOAD, .res = PROTO_SUCCESS, .done = false};
			err = img_load(&img_loader);
			RET_ON_PROTO_ERROR_LOG(err, "Cannot load config blob");

			if (LOAD_FUNC_CFG_STORE == function)
			{
				msg = get_ipc_storage(&dsp_dev.spi_proto);
				uint32_t cfg_idx = *(uint32_t*)user_param;
				sue_large_cfg_set_msg(msg, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_CONFIG_SAVE, sizeof(cfg_idx), &cfg_idx);
				err = ipc_msg_transfer(&dsp_dev.spi_proto, msg, SUE_STREAM_OP_TIMEOUT);
				add_ipc_storage(&dsp_dev.spi_proto, msg);
				RET_ON_PROTO_ERROR_LOG(err, "Error saving config blob data on flash");
			}
			break;

		case LOAD_FUNC_DFU:
			//start DFU
			err = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_FIRMWARE_UPDATE_START, 0, NULL);
			RET_ON_PROTO_ERROR_LOG(err, "Cannot start DFU.");

			//wait for DFU_READY
			LOGI("Waiting for DFU READY. Scheduling...");
			while (dsp_dev.spi_proto.state.dsp_state != DFU_READY)
			{
				dsp_driver_schedule();
				if (i++ > SUE_BOOT_TIMEOUT)
				{
					LOGE("Did not get DFU READY.");
					return PROTO_IPC_ERR;
				}
				TaskDelay(1);
			}

			//set fw content
			img_loader = (img_loader_t){.read_img_cb = cb, .read_img_ctx = ctx, .remaining_size = &remaining, .cumul_sent_size = &cumul_sent_size, .cmd = 0x2, .res = PROTO_SUCCESS, .done = false};
			err = img_load(&img_loader);
			RET_ON_PROTO_ERROR_LOG(err, "Cannot complete DFU");

			//wait for fw to send FW_READY
			LOGI("Waiting for FW READY. Scheduling...");
			i = 0;
			while (dsp_dev.spi_proto.state.dsp_state != FW_READY)
			{
				dsp_driver_schedule();
				if (i++ > SUE_BOOT_TIMEOUT)
				{
					LOGE("Did not get FW READY.");
					return PROTO_IPC_ERR;
				}
				TaskDelay(1);
			}
			break;

		default:
			err = PROTO_INVALID_PARAM;
	}

	return err;
}

proto_err dsp_driver_external_playback_start()
{
	uint32_t state = SUE_EXT_PLAYBACK_STATE_STARTED;
	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_EXTERNAL_PB_NOTIF, sizeof(state), &state);
	RET_ON_PROTO_ERROR_LOG(is_error, "External playback start failed");
	return PROTO_SUCCESS;
}

proto_err dsp_driver_external_playback_stop()
{
	uint32_t state = SUE_EXT_PLAYBACK_STATE_STOPPED;
	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_EXTERNAL_PB_NOTIF, sizeof(state), &state);
	RET_ON_PROTO_ERROR_LOG(is_error, "External playback stop failed");
	return PROTO_SUCCESS;
}

#ifndef TX_TASK
bool dsp_driver_is_protocol_idle()
{
	return dsp_dev.spi_proto.state.is_idle;
}
#endif

proto_err dsp_driver_mfg_run_test(uint32_t test, uint32_t step)
{
    mfg_test_ctl_data_t payload = {
        .command = MFG_COMMAND_TEST_START,
        .test_id = test,
        .test_step = step,
    };
	proto_err is_error = send_large_cfg_set(&dsp_dev.spi_proto, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_MFG_TEST_CONTROL, sizeof(payload), &payload);
	RET_ON_PROTO_ERROR_LOG(is_error, "Test %d, step %d failed", test, step);
	return PROTO_SUCCESS;
}

proto_err dsp_driver_mfg_test_result(uint32_t test, uint32_t step, buf_desc_t* bd)
{
	proto_err is_error = PROTO_SUCCESS;
    mfg_test_ctl_data_t payload = {
        .command = MFG_COMMAND_TEST_STOP,
        .test_id = test,
        .test_step = step,
    };
	ipc_msg* msg = get_ipc_storage(&dsp_dev.spi_proto);
	sue_large_cfg_set_msg(msg, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_MFG_TEST_CONTROL, sizeof(payload), &payload);
	is_error = ipc_msg_transfer(&dsp_dev.spi_proto, msg, SUE_STREAM_OP_TIMEOUT);
	if (bd)
		copy_response_data(msg, bd);
	add_ipc_storage(&dsp_dev.spi_proto, msg);
	RET_ON_PROTO_ERROR_LOG(is_error, "Mfg test failed");
	return PROTO_SUCCESS;
}

proto_err dsp_driver_get_fw_version(buf_desc_t* bd)
{
	proto_err is_error = PROTO_SUCCESS;
	ipc_msg* msg = get_ipc_storage(&dsp_dev.spi_proto);
	sue_large_cfg_set_msg(msg, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_FW_VERSION, 0, NULL);
	is_error = ipc_msg_transfer(&dsp_dev.spi_proto, msg, SUE_STREAM_OP_TIMEOUT);
	if (bd)
		copy_response_data(msg, bd);
	add_ipc_storage(&dsp_dev.spi_proto, msg);
	RET_ON_PROTO_ERROR_LOG(is_error, "Get FW Version failed");
	return PROTO_SUCCESS;
}

proto_err dsp_driver_mic_mute(bool enabled)
{
	proto_err is_error;
	uint32_t payload = (uint32_t) enabled;
	ipc_msg* msg = get_ipc_storage(&dsp_dev.spi_proto);
	sue_large_cfg_set_msg(msg, SUE_MGMT_SRVC_MOD_ID, 0, SUE_CMD_MIC_MUTE_CONTROL, sizeof(payload), &payload);
	is_error = ipc_msg_transfer(&dsp_dev.spi_proto, msg, SUE_STREAM_OP_TIMEOUT);
	if (is_error && msg->buf[2] == ADSP_INVALID_SEQUENCE)
		LOGE("Invalid sequence: choosen mic mute state is already set.");
	add_ipc_storage(&dsp_dev.spi_proto, msg);
	RET_ON_PROTO_ERROR_LOG(is_error, "Mic mute control failed.");
	return PROTO_SUCCESS;
}

void copy_response_data(ipc_msg* msg, buf_desc_t* bd)
{
	uint32_t max_payload_size = IPC_MODE - sizeof(struct ipc_hdr);
	uint32_t copy_size = bd->data_size > max_payload_size ? max_payload_size : bd->data_size;
	memcpy(bd->data, &msg->buf[2], copy_size);
	bd->valid_data_size = msg->buf[1] & 0xFFFFF;
}
