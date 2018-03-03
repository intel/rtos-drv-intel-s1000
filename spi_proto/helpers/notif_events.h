#include "spi_protocol.h"
#include "pal_os_specific.h"

typedef enum {
	NOTIF_WOV_DETECTED, NOTIF_VAD_DETECTED, NOTIF_ASR_DETECTED, NOTIF_MFG_TEST_END, NOTIF_SIZE
} notif_events_t;

typedef struct dsp_evt_cb_hlv_ {
	/*! Header. Interpretation may vary depending on the event */
	uint32_t header;
	/*! Number of valid bytes in the value array that follows */
	uint32_t length;
	/*! event callback data */
	uint8_t value[0];
} dsp_evt_cb_hlv_t;

typedef void (*notif_callback_t)(const void *user_param, notif_events_t event_type, dsp_evt_cb_hlv_t *event_args);
typedef struct {
	void *user_param;
	notif_callback_t callback;
} notif_callback_info_t;

/*!
 * \brief Defines the call back function pointer structure
 */

proto_err register_notif_event(spi_protocol_t* spi_proto, notif_events_t event_type, notif_callback_info_t info);
proto_err unregister_notif_event(spi_protocol_t* spi_proto, notif_events_t event_type);
