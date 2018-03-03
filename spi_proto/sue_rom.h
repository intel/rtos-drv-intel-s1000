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

#ifndef SUE_ROM_H
#define SUE_ROM_H

#include "ipc_msg.h"

#define ROM_REQUEST_MASK        (0x81000000)

enum rom_opcodes {
	ROM_FW_LOAD = 2, ROM_CTRL_EXEC = 0x13
};

typedef struct {
	uint32_t header;
	union {
		uint32_t ext;
		struct {
			uint32_t ipc_size :9;
			uint32_t reserved :11;
			uint32_t ClockSelect :5;
			uint32_t NI :1;
			uint32_t NX :1;
			uint32_t NT :1;
			uint32_t NS :1;
			uint32_t N1 :1;
			uint32_t reserved2 :2;
		};
	};
	uint32_t memory_address;
	uint32_t image_offset;
	uint32_t image_size;
	uint32_t image_sha[32];
} fw_load_t;

void rom_load_fw_ipc(ipc_msg* msg, uint32_t memory_adress, uint32_t image_size, uint32_t image_offet) {
	fw_load_t* fw_load = (fw_load_t*)msg->buf;
	fw_load->header = ROM_REQUEST_MASK | ROM_FW_LOAD;
	fw_load->ipc_size = (sizeof(fw_load_t) - sizeof(fw_load->ext) - sizeof(fw_load->header)) / sizeof(uint32_t);
	fw_load->memory_address = memory_adress;
	fw_load->image_offset = image_size;
	fw_load->image_size = image_size;
	msg->len = SUE_IPC_SIZE_64B;
}

void rom_exec_ipc(ipc_msg* msg, uint32_t memory_adress) {
	msg->buf[0] = ROM_REQUEST_MASK | ROM_CTRL_EXEC;
	msg->buf[1] = 1;
	msg->buf[2] = memory_adress;
	msg->len = SUE_IPC_SIZE_64B;
}

#endif
