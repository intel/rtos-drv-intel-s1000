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

#ifndef IPC_MSG_H
#define IPC_MSG_H

#include "pal_os_specific.h"

#define SUE_IPC_MODE_SEL_R4W4_FLD_OFFSET    30
#define SUE_IPC_MODE_SEL_R4W4_FLD_WIDTH     2
#define SUE_IPC_RESPONSE_SUCCESS			0x0
#define SUE_ROM_CTRL_LOAD               0x02
#define SUE_ROM_CTRL_MODE_SELECT        0x04
#define SUE_ROM_CTRL_ROM_READY          0x20
#define SUE_ROM_CTRL_MEMORY_READ        0x10
#define SUE_ROM_CTRL_MEMORY_WRITE       0x11
#define SUE_ROM_CTRL_MEMORY_BLOCK_WRITE      0x12
#define SUE_ROM_CTRL_EXEC               0x13
#define SUE_ROM_CTRL_WAIT               0x14
#define IPC_ROM_CTRL_FW_LD      (0x02)
#define IPC_ROM_CTRL_MEM_RD     (0x10)
#define IPC_ROM_CTRL_MEM_WR     (0x11)
#define IPC_ROM_CTRL_READY      (0x20)
#define SUE_IPC_ROM_CTRL_CMD_FLD_OFFSET     0
#define SUE_IPC_ROM_CTRL_CMD_FLD_WIDTH      9

enum sue_ipc_mode {
	SUE_R64_W64 = 0x00, SUE_R64_W4K = 0x01, SUE_R4K_W64 = 0x02, SUE_R4K_W4K = 0x03,
};

enum sue_ipc_size {
	SUE_IPC_SIZE_64B = 64, SUE_IPC_SIZE_4KB = 4096
};

enum msg_tgt {
	MSG_TGT_FW_GEN_MSG = 0, MSG_TGT_MODULE_MSG = 1
};

enum msg_rsp {
	MSG_RSP_REQUEST = 0, /* Request */
	MSG_RSP_NOTIFICATION = 0, /* Notification */
	MSG_RSP_REPLY = 1 /* Reply */
};

enum glb_msg_type {
	GLB_MSG_T_ROM_CONTROL = 1,
	GLB_MSG_T_IPC_GTW_CMD = 2,
	GLB_MSG_T_PERF_MEAS_CMD = 13,
	GLB_MSG_T_CHAIN_DMA = 14,
	GLB_MSG_T_LOAD_MULTI_MODULES = 15,
	GLB_MSG_T_UNLOAD_MULT_MODULES = 16,
	GLB_MSG_T_CREATE_PIPELINE = 17,
	GLB_MSG_T_DELETE_PIPELINE = 18,
	GLB_MSG_T_SET_PIPELINE_STATE = 19,
	GLB_MSG_T_GET_PIPELINE_STATE = 20,
	GLB_MSG_T_GET_PIPELINE_CTX_SIZE = 21,
	GLB_MSG_T_SAVE_PIPELINE = 22,
	GLB_MSG_T_RESTORE_PIPELINE = 23,
	GLB_MSG_T_LOAD_LIBRARY = 24,
	GLB_MSG_T_NOTIFICATION = 27
};

enum mod_msg_type {
	MOD_MSG_T_INIT_INSTANCE = 0,
	MOD_MSG_T_MOD_CONFIG_GET = 1,
	MOD_MSG_T_MOD_CONFIG_SET = 2,
	MOD_MSG_T_LARGE_CONFIG_GET = 3,
	MOD_MSG_T_LARGE_CONFIG_SET = 4,
	MOD_MSG_T_BIND = 5,
	MOD_MSG_T_UNBIND = 6,
	MOD_MSG_T_SET_DX = 7,
	MOD_MSG_T_SET_D0IX = 8,
	MOD_MSG_T_ENT_MOD_RESTORE = 9,
	MOD_MSG_T_EXT_MOD_RESTORE = 10,
	MOD_MSG_T_DELETE_INSTANCE = 11
};

enum notif_type {
	NOTIF_T_PHRASE_DETECTED = 4,
	NOTIF_T_RESOURCE_EVENT = 5,
	NOTIF_T_LOG_BUFFER_STATUS = 6,
	NOTIF_T_TIMESTAMP_CAPTURED = 7,
	NOTIF_T_FW_READY = 8,
	NOTIF_T_EXCEPTION_CAUGHT = 10,
	NOTIF_T_MODULE_NOTIFICATION = 12,
	NOTIF_T_DFU_READY = 13,
	NOTIF_T_ASR_RESULT = 14,
	NOTIF_T_VAD_DETECTED = 15,
};

enum resource_event_type {
	RESOURCE_EVENT_T_BUDGET_VIOLATION = 0, RESOURCE_EVENT_T_MIXER_UNDERRUN_DETECTED = 1,
	//RESOURCE_EVENT_T_STREAM_DATA_SEGMENT = 2,
	RESOURCE_EVENT_T_PROCESS_DATA_ERROR = 3,
	//RESOURCE_EVENT_T_STACK_OVERFLOW = 4,
	//RESOURCE_EVENT_T_BUFFERING_MODE_CHANGED = 5,
	RESOURCE_EVENT_T_GTW_UNDERRUN_DETECTED = 6,
	RESOURCE_EVENT_T_GTW_OVERRUN_DETECTED = 7,
	//RESOURCE_EVENT_T_EDF_DOMAIN_UNSTABLE = 8,
	//RESOURCE_EVENT_T_RSVD_9 = 9,
	RESOURCE_EVENT_T_GTW_HIGH_THRESHOLD = 10,
	RESOURCE_EVENT_T_GTW_LOW_THRESHOLD = 11,
	RESOURCE_EVENT_T_INVALID_RESORUCE_EVENT_TYPE,
};

enum resource_type {
	RESOURCE_T_MODULE_INSTANCE = 0,
	RESOURCE_T_PIPELINE = 1,
	RESOURCE_T_GATEWAY = 2,
	RESOURCE_T_EDF_TASK = 3,
	RESOURCE_T_INVALID_RESOURCE_TYPE,
};
enum ipc_gtw_cmd {
	IPC_GTW_CMD_GET_DATA = 1, IPC_GTW_CMD_SET_DATA = 2,
};
/*
 struct global_message_request {
 uint32_t rsvd0      : 24;
 uint32_t type       : 5;
 uint32_t rsp        : 1;
 uint32_t msg_tgt    : 1;
 uint32_t _hw_rsvd_0 : 1;

 uint32_t rsvd1      : 30;
 uint32_t _hw_rsvd_2 : 2;
 };
 */

/*
 #define GLB_MSG_REQ_RSVD0_FLD_OFFSET	0
 #define GLB_MSG_REQ_RSVD0_FLD_WIDTH		24
 #define GLB_MSG_REQ_TYPE_FLD_OFFSET		24
 #define GLB_MSG_REQ_TYPE_FLD_WIDTH		5
 #define GLB_MSG_REQ_RSP_FLD_OFFSET		29
 #define GLB_MSG_REQ_RSP_FLD_WIDTH		1
 #define GLB_MSG_REQ_TGT_FLD_OFFSET		30
 #define GLB_MSG_REQ_TGT_FLD_WIDTH		1
 #define GLB_MSG_REQ_HWRSVD0_FLD_OFFSET	31
 #define GLB_MSG_REQ_HWRSVD0_FLD_WIDTH	1
 */

/*
 struct module_message_request {
 uint32_t module_id      : 16;
 uint32_t instance_id    : 8;
 uint32_t type           : 5;
 uint32_t rsp            : 1;
 uint32_t msg_tgt        : 1;
 uint32_t _hw_rsvd_0     : 1;

 uint32_t rsvd1          : 30;
 uint32_t _hw_rsvd_2     : 2;
 };
 */

#define MOD_MSG_REQ_MOD_ID_FLD_OFFSET	0
#define MOD_MSG_REQ_MOD_ID_FLD_WIDTH	16
#define MOD_MSG_REQ_INST_ID_FLD_OFFSET	16
#define MOD_MSG_REQ_INST_ID_FLD_WIDTH	8
#define MOD_MSG_REQ_TYPE_FLD_OFFSET		24
#define MOD_MSG_REQ_TYPE_FLD_WIDTH		5
#define MOD_MSG_REQ_RSP_FLD_OFFSET		29
#define MOD_MSG_REQ_RSP_FLD_WIDTH		1
#define MOD_MSG_REQ_TGT_FLD_OFFSET		30
#define MOD_MSG_REQ_TGT_FLD_WIDTH		1
#define MOD_MSG_REQ_HWRSVD0_FLD_OFFSET	31
#define MOD_MSG_REQ_HWRSVD0_FLD_WIDTH	1

/*
 struct message_reply {
 uint32_t status     : 24;
 uint32_t type       : 5;
 uint32_t rsp        : 1;
 uint32_t msg_tgt    : 1;
 uint32_t _hw_rsvd_0 : 1;

 uint32_t rsvd1      : 30;
 uint32_t _hw_rsvd_2 : 2;
 };
 */

#define MSG_REPLY_STATUS_FLD_OFFSET		0
#define MSG_REPLY_STATUS_FLD_WIDTH		24
#define MSG_REPLY_TYPE_FLD_OFFSET		24
#define MSG_REPLY_TYPE_FLD_WIDTH		5
#define MSG_REPLY_RSP_FLD_OFFSET		29
#define MSG_REPLY_RSP_FLD_WIDTH			1
#define MSG_REPLY_TGT_FLD_OFFSET		30
#define MSG_REPLY_TGT_FLD_WIDTH			1
#define MSG_REPLY_HWRSVD0_FLD_OFFSET	31
#define MSG_REPLY_HWRSVD0_FLD_WIDTH		1

/*
 struct notification {
 uint32_t rsvd2          : 16;
 uint32_t notif_type     : 8;
 uint32_t type           : 5;
 uint32_t rsp            : 1;
 uint32_t msg_tgt        : 1;
 uint32_t _hw_rsvd_0     : 1;

 uint32_t rsvd1          : 30;
 uint32_t _hw_rsvd_2     : 2;
 };
 */

#define MSG_NOTIF_RSVD2_FLD_OFFSET		0
#define MSG_NOTIF_RSVD2_FLD_WIDTH		16
#define MSG_NOTIF_NOTIF_TYPE_FLD_OFFSET	16
#define MSG_NOTIF_NOTIF_TYPE_FLD_WIDTH	8
#define MSG_NOTIF_TYPE_FLD_OFFSET		24
#define MSG_NOTIF_TYPE_FLD_WIDTH		5
#define MSG_NOTIF_RSP_FLD_OFFSET		29
#define MSG_NOTIF_RSP_FLD_WIDTH			1
#define MSG_NOTIF_TGT_FLD_OFFSET		30
#define MSG_NOTIF_TGT_FLD_WIDTH			1
#define MSG_NOTIF_HWRSVD0_FLD_OFFSET	31
#define MSG_NOTIF_HWRSVD0_FLD_WIDTH		1

#define MSG_REQ_BUSY_FLD_OFFSET         31
#define MSG_REQ_BUSY_FLD_WIDTH          1
/*
 Large Config set header extension fields
 uint32_t data_off_size  : 20;
 uint32_t large_param_id : 8;
 uint32_t final_block    : 1;
 uint32_t init_block     : 1;
 uint32_t _hw_rsvd_2     : 2;

 */

#define LRG_CFG_DATA_OFF_SIZE_FLD_OFFSET 0
#define LRG_CFG_DATA_OFF_SIZE_FLD_WIDTH	 20
#define LRG_CFG_PARAM_ID_FLD_OFFSET		 20
#define LRG_CFG_PARAM_ID_FLD_WIDTH		 8
#define LRG_CFG_FIN_BLK_FLD_OFFSET		 28
#define LRG_CFG_FIN_BLK_FLD_WIDTH		 1
#define LRG_CFG_INIT_BLK_FLD_OFFSET		 29
#define LRG_CFG_INIT_BLK_FLD_WIDTH		 1
#define LRG_CFG_HWRSVD2_FLD_OFFSET		 30
#define LRG_CFG_HWRSVD2_FLD_WIDTH		 2

/* IPC gateway Message */
/*
 uint32_t cmd            : 24;   < takes one of IPCGWCMD_... values
 uint32_t type           : 5;    < Global::IPCGATEWAY_CMD
 uint32_t rsp            : 1;    < Msg::MSG_REQUEST
 uint32_t msg_tgt        : 1;    < Msg::FW_GEN_MSG
 uint32_t _reserved_1    : 1;

 uint32_t data_size      : 30;   < in bytes (size of data.payload[])
 uint32_t _reserved_2    : 2;
 */
#define IPC_GTW_MSG_REQ_CMD_FLD_OFFSET		0
#define IPC_GTW_MSG_REQ_CMD_FLD_WIDTH		24	
#define IPC_GTW_MSG_REQ_DATA_SZ_FLD_OFFSET	0
#define IPC_GTW_MSG_REQ_DATA_SZ_FLD_WIDTH 	30	

/* Module init instance message extension */
/*
 uint32_t param_block_size   : 16;
 uint32_t ppl_instance_id    : 8;
 uint32_t core_id            : 4;
 uint32_t proc_domain        : 1;
 uint32_t rsvd1              : 1;
 uint32_t _hw_rsvd_2         : 2;
 */
#define MOD_INIT_INST_BLK_SZ_FLD_OFFSET        0
#define MOD_INIT_INST_BLK_SZ_FLD_WIDTH        16
#define MOD_INIT_INST_PPL_INST_ID_FLD_OFFSET  16
#define MOD_INIT_INST_PPL_INST_ID_FLD_WIDTH    8
#define MOD_INIT_INST_CORE_ID_FLD_OFFSET      24
#define MOD_INIT_INST_CORE_ID_FLD_WIDTH        4
#define MOD_INIT_INST_PROC_DOMAIN_FLD_OFFSET  28
#define MOD_INIT_INST_PROC_DOMAIN_FLD_WIDTH    1
#define MOD_INIT_INST_RSVD1_FLD_OFFSET        29
#define MOD_INIT_INST_RSVD1_FLD_WIDTH          1
#define MOD_INIT_INST_HW_RSVD_2_FLD_OFFSET    30
#define MOD_INIT_INST_HW_RSVD_2_FLD_WIDTH      2

#define SET_FIELD(reg, val, width, offset) \
reg = (((reg) & (~(((1 << (width)) - 1) << (offset)))) | ((val) & ((1 << (width)) - 1)) << (offset));
#define GET_FIELD(reg,  width, offset) \
((((reg) >> (offset)) & ((1 << (width)) - 1)));

struct ipc_hdr {
	uint32_t pri;
	uint32_t ext;
};

struct mega_cfg_ipc_hdr {
	uint32_t pri;
	uint32_t ext;
	uint32_t offset;
};

struct ipc_msg_;
typedef struct ipc_msg_ ipc_msg;

typedef bool (*spiproto_callback_t)(void *, ipc_msg*);
typedef struct {
	spiproto_callback_t callback;
	void *owner;
} spiproto_cb_func_t;

struct ipc_msg_ {
	uint16_t status;
	spiproto_cb_func_t self;
	uint32_t len;
	uint32_t buf[];
};

void sue_large_cfg_set_msg(ipc_msg* msg, uint32_t mod_id, uint32_t inst_id, uint32_t param_id, uint32_t param_size,
		void *param_data);
void sue_large_cfg_get_msg(ipc_msg *msg, uint32_t mod_id, uint32_t inst_id, uint32_t param_id, uint32_t param_size,
		void *param_data);
int sue_mega_cfg_set_msg(ipc_msg *msg, uint32_t mod_id, uint32_t inst_id, uint32_t param_id, uint32_t *remaining,
		uint32_t *cumul_sent_size, const uint8_t *data_start, uint32_t packet_size);
uint32_t msg_hdr_pri(uint32_t mod_id, uint32_t inst_id, uint32_t type, uint32_t msg_target);
uint32_t create_module_msg_hdr_pri(uint32_t mod_id, uint32_t inst_id, uint32_t type);
uint32_t create_module_init_inst_msg_hdr_ext(uint32_t block_sz, uint32_t ppl_inst_id, uint8_t core_id, uint8_t proc_domain);
void create_module_init_inst_msg_hdr(struct ipc_hdr *hdr, uint32_t mod_id, uint32_t inst_id, uint32_t type, uint32_t block_sz,
		uint32_t ppl_inst_id, uint8_t core_id, uint8_t proc_domain);
uint32_t create_lrg_msg_hdr_ext(uint32_t data_off_size, uint32_t param_id, uint32_t fin_blk, uint32_t init_blk);

#endif
