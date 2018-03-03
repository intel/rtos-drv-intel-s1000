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

#include <stdint.h>
#include <string.h>
#include "ipc_msg.h"


uint32_t msg_hdr_pri(uint32_t mod_id, uint32_t inst_id, uint32_t type, uint32_t msg_target) {
	uint32_t pri = 0;
	SET_FIELD(pri, mod_id, MOD_MSG_REQ_MOD_ID_FLD_WIDTH, MOD_MSG_REQ_MOD_ID_FLD_OFFSET)
	SET_FIELD(pri, inst_id, MOD_MSG_REQ_INST_ID_FLD_WIDTH, MOD_MSG_REQ_INST_ID_FLD_OFFSET)
	SET_FIELD(pri, type, MOD_MSG_REQ_TYPE_FLD_WIDTH, MOD_MSG_REQ_TYPE_FLD_OFFSET)
	SET_FIELD(pri, MSG_RSP_REQUEST, MOD_MSG_REQ_RSP_FLD_WIDTH, MOD_MSG_REQ_RSP_FLD_OFFSET)
	SET_FIELD(pri, msg_target, MOD_MSG_REQ_TGT_FLD_WIDTH, MOD_MSG_REQ_TGT_FLD_OFFSET)
	SET_FIELD(pri, 1, MSG_REQ_BUSY_FLD_WIDTH, MSG_REQ_BUSY_FLD_OFFSET)
	return pri;
}

uint32_t create_module_msg_hdr_pri(uint32_t mod_id, uint32_t inst_id, uint32_t type) {
	return msg_hdr_pri(mod_id, inst_id, type, MSG_TGT_MODULE_MSG);;
}

uint32_t create_lrg_msg_hdr_ext(uint32_t data_off_size, uint32_t param_id, uint32_t fin_blk, uint32_t init_blk) {
	uint32_t ext = 0;

	SET_FIELD(ext, data_off_size, LRG_CFG_DATA_OFF_SIZE_FLD_WIDTH, LRG_CFG_DATA_OFF_SIZE_FLD_OFFSET)
	SET_FIELD(ext, param_id, LRG_CFG_PARAM_ID_FLD_WIDTH, LRG_CFG_PARAM_ID_FLD_OFFSET)
	SET_FIELD(ext, fin_blk, LRG_CFG_FIN_BLK_FLD_WIDTH, LRG_CFG_FIN_BLK_FLD_OFFSET)
	SET_FIELD(ext, init_blk, LRG_CFG_INIT_BLK_FLD_WIDTH, LRG_CFG_INIT_BLK_FLD_OFFSET)

	return ext;
}

void sue_large_cfg_get_msg(ipc_msg *msg, uint32_t mod_id, uint32_t inst_id, uint32_t param_id, uint32_t param_size,
		void *param_data) {
	struct ipc_hdr *header = (struct ipc_hdr *) msg->buf;
	header->pri = create_module_msg_hdr_pri(mod_id, inst_id, MOD_MSG_T_LARGE_CONFIG_GET);
	header->ext = create_lrg_msg_hdr_ext(param_size, param_id, 1, 1);
	if (param_size > 0)
		memcpy((uint8_t *) msg->buf + sizeof(struct ipc_hdr), param_data, param_size);
	msg->len = param_size + sizeof(struct ipc_hdr);
}

void sue_large_cfg_set_msg(ipc_msg* msg, uint32_t mod_id, uint32_t inst_id, uint32_t param_id, uint32_t param_size,
		void *param_data) {
	struct ipc_hdr *header = (struct ipc_hdr *) msg->buf;
	header->pri = create_module_msg_hdr_pri(mod_id, inst_id, MOD_MSG_T_LARGE_CONFIG_SET);
	header->ext = create_lrg_msg_hdr_ext(param_size, param_id, 1, 1);
	if (param_size > 0)
		memcpy((uint8_t *) msg->buf + sizeof(struct ipc_hdr), param_data, param_size);

	msg->len = param_size + sizeof(struct ipc_hdr);
}

//#define ENABLE_CONFIG_BLOB_DATA_DUMP
#ifdef ENABLE_CONFIG_BLOB_DATA_DUMP
#define LAST_BLOB_EXTENSION_HEADER 0x91028e14 /* Define proper Last blob Extension header according to config blob */
#endif
int sue_mega_cfg_set_msg(ipc_msg *msg, uint32_t mod_id, uint32_t inst_id, uint32_t param_id, uint32_t *remaining,
		uint32_t *cumul_sent_size, const uint8_t *data_start, uint32_t packet_size) {
	uint8_t *tx_buf = (uint8_t *) msg->buf;
	struct mega_cfg_ipc_hdr *mcfg_hdr = (struct mega_cfg_ipc_hdr *) tx_buf;
	uint32_t pay_load_size;
	uint32_t cumul_sent_size_var = *cumul_sent_size;
	const uint8_t *data = data_start + cumul_sent_size_var;
	uint32_t remaining_data = *remaining;
	uint32_t max_payload_size = packet_size - sizeof(*mcfg_hdr);
	int32_t ipc_len = SUE_IPC_SIZE_64B;

	mcfg_hdr->pri = create_module_msg_hdr_pri(mod_id, inst_id, MOD_MSG_T_LARGE_CONFIG_SET);

	if (cumul_sent_size_var == 0) {
		/* This is the first block; Set Init blk =1; */
		mcfg_hdr->ext = create_lrg_msg_hdr_ext(cumul_sent_size_var, param_id, 0, 1);
		/* Set the reserved field just to conform to the logs from FW scripts.*/
		mcfg_hdr->offset = remaining_data;
	} else {
		/* Assume it is an intermediate block. Decide later if it is final block */
		mcfg_hdr->ext = create_lrg_msg_hdr_ext(cumul_sent_size_var, param_id, 0, 0);
		/* Set the reserved field just to conform to the logs from FW scripts.*/
		mcfg_hdr->offset = cumul_sent_size_var;
	}

	if (remaining_data <= max_payload_size) {
		/* This is the final block; Set Final blk =1; */
		pay_load_size = remaining_data;
		SET_FIELD(mcfg_hdr->ext, 1, LRG_CFG_FIN_BLK_FLD_WIDTH, LRG_CFG_FIN_BLK_FLD_OFFSET)
	} else {
		pay_load_size = max_payload_size;
	}

	memcpy(tx_buf + sizeof(*mcfg_hdr), data, pay_load_size);

	if (pay_load_size + sizeof(*mcfg_hdr) > SUE_IPC_SIZE_64B)
	{
		ipc_len = SUE_IPC_SIZE_4KB;
		SET_FIELD(mcfg_hdr->ext, 2, LRG_CFG_HWRSVD2_FLD_WIDTH, LRG_CFG_HWRSVD2_FLD_OFFSET)
	}

	*remaining = remaining_data - pay_load_size;
	*cumul_sent_size = cumul_sent_size_var + pay_load_size;

	msg->len = ipc_len;

	return 0;
}

uint32_t create_module_init_inst_msg_hdr_ext(uint32_t block_sz, uint32_t ppl_inst_id, uint8_t core_id, uint8_t proc_domain) {
	uint32_t ext = 0;
	SET_FIELD(ext, block_sz, MOD_INIT_INST_BLK_SZ_FLD_WIDTH, MOD_INIT_INST_BLK_SZ_FLD_OFFSET)
	SET_FIELD(ext, ppl_inst_id, MOD_INIT_INST_PPL_INST_ID_FLD_WIDTH, MOD_INIT_INST_PPL_INST_ID_FLD_OFFSET)
	SET_FIELD(ext, core_id, MOD_INIT_INST_CORE_ID_FLD_WIDTH, MOD_INIT_INST_CORE_ID_FLD_OFFSET)
	SET_FIELD(ext, proc_domain, MOD_INIT_INST_PROC_DOMAIN_FLD_WIDTH, MOD_INIT_INST_PROC_DOMAIN_FLD_OFFSET)
	return ext;
}

void create_module_init_inst_msg_hdr(struct ipc_hdr *hdr, uint32_t mod_id, uint32_t inst_id, uint32_t type, uint32_t block_sz,
		uint32_t ppl_inst_id, uint8_t core_id, uint8_t proc_domain) {
	hdr->pri = create_module_msg_hdr_pri(mod_id, inst_id, type);
	hdr->ext = create_module_init_inst_msg_hdr_ext(block_sz, ppl_inst_id, core_id, proc_domain);
}
