#include "notif_events.h"

#include <string.h>

static const char *LOG_TAG = "AudDrv-helper";

#define ASR_DATA_LENGTH 268
#define VAD_DATA_LENGTH 16
#define WOV_DATA_MAX_LENGTH 56
#define MFG_DATA_MAX_LENGTH 268

#define ASR_EVNT_ARGS_SIZE ((sizeof(dsp_evt_cb_hlv_t)) + ASR_DATA_LENGTH)
#define VAD_EVNT_ARGS_SIZE ((sizeof(dsp_evt_cb_hlv_t)) + VAD_DATA_LENGTH)
#define WOV_EVNT_ARGS_SIZE ((sizeof(dsp_evt_cb_hlv_t)) + WOV_DATA_MAX_LENGTH)
#define MFG_EVNT_ARGS_SIZE ((sizeof(dsp_evt_cb_hlv_t)) + MFG_DATA_MAX_LENGTH)

typedef struct {
	spi_protocol_t* spi_proto;
	notif_callback_info_t info;
} notif_handle_wrapper_t;

static notif_handle_wrapper_t handle_wrappers[4] = { 0 };

bool wov_handler(notif_handle_wrapper_t* handle_wrapper, ipc_msg* msg) {
	LOGI("WOV: Phrase detected");
	add_ipc_storage(handle_wrapper->spi_proto, msg);
	uint8_t buf [WOV_EVNT_ARGS_SIZE] = { 0 };
	handle_wrapper->info.callback(handle_wrapper->info.user_param, NOTIF_WOV_DETECTED, (dsp_evt_cb_hlv_t*)buf);
	return true;
}

bool vad_handler(notif_handle_wrapper_t* handle_wrapper, ipc_msg* msg) {
	LOGI("VAD: detected");
	add_ipc_storage(handle_wrapper->spi_proto, msg);
	uint8_t buf [VAD_EVNT_ARGS_SIZE] = { 0 };
	handle_wrapper->info.callback(handle_wrapper->info.user_param, NOTIF_VAD_DETECTED, (dsp_evt_cb_hlv_t*)buf);
	return true;
}

bool asr_handler(notif_handle_wrapper_t* handle_wrapper, ipc_msg* msg) {
	LOGI("ASR: entered spi_source_asr");
	add_ipc_storage(handle_wrapper->spi_proto, msg);
	uint8_t buf [ASR_EVNT_ARGS_SIZE] = { 0 };
	handle_wrapper->info.callback(handle_wrapper->info.user_param, NOTIF_ASR_DETECTED, (dsp_evt_cb_hlv_t*)buf);
	return true;
}

bool mfg_handler(notif_handle_wrapper_t* handle_wrapper, ipc_msg* msg) {
	LOGI("MFG: test ended");
	add_ipc_storage(handle_wrapper->spi_proto, msg);
	uint8_t buf [MFG_EVNT_ARGS_SIZE] = { 0 };
	handle_wrapper->info.callback(handle_wrapper->info.user_param, NOTIF_MFG_TEST_END, (dsp_evt_cb_hlv_t*)buf);
	return true;
}

static spiproto_callback_t event_to_spi_proto_func(notif_events_t event_type) {

	switch (event_type) {
	case NOTIF_WOV_DETECTED:
		return (spiproto_callback_t) wov_handler;
	case NOTIF_VAD_DETECTED:
		return (spiproto_callback_t) vad_handler;
	case NOTIF_ASR_DETECTED:
		return (spiproto_callback_t) asr_handler;
	case NOTIF_MFG_TEST_END:
		return (spiproto_callback_t) mfg_handler;
	default:
		return NULL;
	}
	LOGE("Other notifs are not implemented");
}
proto_err register_notif_event(spi_protocol_t* spi_proto, notif_events_t event_type, notif_callback_info_t info) {

	spiproto_callback_t on_notif = event_to_spi_proto_func(event_type);
	if (!on_notif)
		return PROTO_IGNORE_REQUEST;

	uint16_t notif_type;
	switch (event_type) {
	case NOTIF_WOV_DETECTED:
		notif_type = NOTIF_T_PHRASE_DETECTED;
		break;
	case NOTIF_VAD_DETECTED:
		notif_type = NOTIF_T_VAD_DETECTED;
		break;
	case NOTIF_ASR_DETECTED:
		notif_type = NOTIF_T_ASR_RESULT;
		break;
	case NOTIF_MFG_TEST_END:
		notif_type = NOTIF_T_MODULE_NOTIFICATION;
		break;
	default:
		LOGE("Other notifs are not implemented");
		return PROTO_IGNORE_REQUEST;
	}

	if (handle_wrappers[event_type].info.callback) {
		LOGE("Already registered for event type %d", event_type);
		return PROTO_IGNORE_REQUEST;
	}

	handle_wrappers[event_type].info = info;
	handle_wrappers[event_type].spi_proto = spi_proto;
	proto_err res = spi_proto_handler_add(spi_proto, on_notif, &handle_wrappers[event_type], notif_type);

	if (res)
		memset(&handle_wrappers[event_type], 0, sizeof(notif_handle_wrapper_t));

	return res;
}

proto_err unregister_notif_event(spi_protocol_t* spi_proto, notif_events_t event_type) {
	proto_err res = PROTO_SUCCESS;
	if (event_type >= NOTIF_SIZE) {
		LOGE("No matching event type");
		return PROTO_IGNORE_REQUEST;
	}

	spiproto_callback_t on_notif = event_to_spi_proto_func(event_type);
	res = spi_proto_handler_remove(spi_proto, on_notif, &handle_wrappers[event_type]);
	if (!res)
		memset(&handle_wrappers[event_type], 0, sizeof(notif_handle_wrapper_t));

	return res;
}
