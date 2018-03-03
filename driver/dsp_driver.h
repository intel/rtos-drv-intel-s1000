/*  BSD LICENSE
*
*  Copyright(c) 2019 Intel Corporation.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in
*      the documentation and/or other materials provided with the
*      distribution.
*    * Neither the name of Intel Corporation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef DSP_DRIVER_H
#define DSP_DRIVER_H


#include "spi_gtw/spi_common.h"
#include "spi_lib_error.h"
#include "spi_settings.h"
#include "helpers/notif_events.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define REFERENCE_STREAM_ID 5

typedef struct buf_desc_ {
    uint32_t data_size;
    uint32_t valid_data_size;
    uint32_t data[0];
}buf_desc_t;

typedef struct dsp_evt_cb_tlv_
{
	uint32_t	type;
	uint32_t    length;
	uint8_t     value[0];
} dsp_evt_cb_tlv_t;

typedef struct evt_cb_ops_
{
	notif_callback_info_t notif_callbacks[NOTIF_SIZE];
	callback_info_t gtw_event_callbacks[DSP_EVENTS_COUNT];
} evt_cb_ops_t;

typedef enum _dsp_dialect {
	DIALECT_DE_DE = 0,
	DIALECT_EN_AU,
	DIALECT_EN_CA,
	DIALECT_EN_GB,
	DIALECT_EN_IN,
	DIALECT_EN_US,
	DIALECT_JA_JP,
	DIALECT_ZH_CN,
	DIALECT_NOT_SPECIFIED,
} dsp_dialect_t;

typedef enum dsp_capture_state_
{
    DSP_CAPTURE_STATE_IDLE = 0,
    DSP_CAPTURE_STATE_WOV,
    DSP_CAPTURE_STATE_STREAM,
    DSP_CAPTURE_STATE_WOV_STREAM,
    DSP_CAPTURE_STATE_RESET,
    DSP_CAPTURE_STATE_COUNT,
} dsp_capture_state_t;

typedef struct dsp_fw_version
{
	uint16_t major_version;
	uint16_t minor_version;
	uint16_t hotfix_version;
	uint16_t build_version;
} dsp_fw_version_t;

/**
 * initializes the driver, performs dsp reset
 */
proto_err dsp_driver_init(irq_handler_t isr, uint32_t boot_mode);
/**
 * Deinitializes the driver
 */
proto_err dsp_driver_deinit();
/**
 * sets language dialect, sets wov, ipc error, xrun, buffer completion, dsp wake callbacks
 */
proto_err dsp_driver_configure(dsp_dialect_t dialect, evt_cb_ops_t *cb_ops); //enum to be defined for dialect

/**
 * sends TOPOLOGY_START message
 */
proto_err dsp_driver_start();

/**
 * register MFG callback function
 */
proto_err dsp_driver_mfg_start();

/**
 * non-blocking, sets buffer chunk to be filled with captured audio data
 */
proto_err dsp_driver_read_data(unsigned int stream_id, void *buffer, unsigned int req_size, uint32_t **read);

/**
 * non-blocking, sets audio data buffer chunk to be used for playback
 */
proto_err dsp_driver_write_data(unsigned int stream_id, void *buffer, unsigned int size, uint32_t **written);

/**
 * execute scheduled non-blocking driver actions
 */
proto_err dsp_driver_schedule();

/**
 * set capture pipeline to one of supported states:
 *      DSP_CAPTURE_STATE_WOV - arms for keyphrase recognition,
 *      DSP_CAPTURE_STATE_STREAM - starts capturing data without need for keyphrase detection,
 *      DSP_CAPTURE_STATE_IDLE - stops capture stream
 */
proto_err dsp_driver_set_state(dsp_capture_state_t capture_state);

/**
 * start audio stream (playback, external reference)
 */
proto_err dsp_driver_start_stream(unsigned int stream_id, gtw_audio_format_t *audio_format, callback_info_t gtw_event_callbacks[]);
/**
 * stop audio stream
 */
proto_err dsp_driver_stop_stream(unsigned int stream_id);
/**
 * notifies dsp about external playback start
 */
proto_err dsp_driver_external_playback_start();
/**
 * notifies dsp about external playback stop
 */
proto_err dsp_driver_external_playback_stop();
/**
 * performs mfg fw test - restricted only to manufacturing fw image
 */
proto_err dsp_driver_mfg_run_test(uint32_t test, uint32_t step);
/**
 * reads previous mfg fw test results - restricted only to manufacturing fw image
 */
proto_err dsp_driver_mfg_test_result(uint32_t test, uint32_t step, buf_desc_t* bd);
/**
 * reads fw version
 */
proto_err dsp_driver_get_fw_version(buf_desc_t* bd);
/**
 * controls mic mute state
 * param enabled: true - mic mute, false - mic unmute
 */
proto_err dsp_driver_mic_mute(bool enabled);

typedef enum _load_function {
	LOAD_FUNC_BOOTSTRAP, //loading fw through spi-s
	LOAD_FUNC_DFU,
	LOAD_FUNC_CFG_LOAD,
	LOAD_FUNC_CFG_STORE,
	LOAD_FUNC_COUNT
} load_function_t;

typedef uint16_t (*read_img_chunk_cb)(void*, uint8_t*, uint16_t);

/**
 * loads fw img through spi-s, sets read data chunk callback function used while loading
 */
proto_err dsp_driver_load_img(load_function_t function, read_img_chunk_cb cb, void* ctx, uint32_t img_size, void* user_param);

/**
 * returns IPC protocol state: true if in idle state, false otherwise
 */
#ifndef TX_TASK
bool dsp_driver_is_protocol_idle();
#endif

#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif // DSP_DRIVER_H
