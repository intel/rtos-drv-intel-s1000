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

#ifndef CRB_DEFS
#define CRB_DEFS

#define ADSP_INVALID_SEQUENCE 144
#define ADSP_DFU_DOWNGRADE_NOT_ALLOWED 3601
/*Pin configuration for Sue*/

#define IPC_REQUEST_MASK        (0x81000000)
#define CLK_SEL_SPI_SLV         (1 << 21)
#define IPC_SC_IMG_ADDR         (0xBE000000)
#define IPC_SC_IMG_INDEX        (0)
#define IPC_SC_DELAY_ADDR1      (0x304628)
#define IPC_SC_DELAY_VALUE1     (0x4F)
#define IPC_SC_DELAY_ADDR2      (0x71d14)
#define IPC_SC_DELAY_VALUE2     (0x00)
#define IPC_SC_DELAY_ADDR3      (0x71d24)
#define IPC_SC_DELAY_VALUE3     (0x00)
#define IPC_SC_DELAY_ADDR4      (0x301000)
#define IPC_SC_DELAY_VALUE4     (0x20020608)
#define IPC_SC_SHA_OFFSET       (20)

#define ETIMEOUT                2 /* Timeout Error */
#define SUE_ROM_RDY_TIMEOUT     1200
#define SUE_BOOT_TIMEOUT        12000
#define SUE_STREAM_OP_TIMEOUT   1800

/* PB (SET DATA) or CAP (GET_DATA) timeout. Typical latencies for these commands are ~5ms.*/
#define SUE_PB_CAP_CMD_TIMEOUT  500

#define SUE_MGMT_SRVC_MOD_ID 0x1000

enum sue_phrase_det_state {
	SUE_WAITING_FOR_PHRASE = 0, SUE_PHRASE_DETECTED = 1,
};

/* Commands(Larg config IPC param ids) for control module */
enum sue_cmds_control_mod {
	SUE_CMD_STREAM_STOP = 1,
	SUE_CMD_STREAM_START = 2,
	SUE_CMD_CAPTURE_STOP = 3,
	SUE_CMD_CAPTURE_START = 4,
	SUE_CMD_TOPOLOGY_SOFT_RESET = 5,
	SUE_CMD_TOPOLOGY_STOP = 8,
	SUE_CMD_TOPOLOGY_START = 9,
	SUE_CMD_FIRMWARE_UPDATE_START = 0x0c,
	SUE_CMD_MIC_MUTE_CONTROL = 0xd,
	SUE_CMD_CONFIG_LOAD = 0x10,
	SUE_CMD_CONFIG_SAVE = 0x11,
	SUE_CMD_CONFIG_LIST = 0x12,
	SUE_CMD_CONFIG_SET_DEFAULT = 0x13,
	SUE_CMD_CONFIG_SELECT = 0x13, // number reused
	SUE_CMD_CAPTURE_SET_STATE = 0x14,
	SUE_CMD_IPC_MODE = 0x50,
	SUE_CMD_FW_VERSION = 0x51,
	SUE_CMD_EXTERNAL_PB_NOTIF = 0x60,
	SUE_CMD_MFG_TEST_CONTROL = 0x61,
};

enum ipc_exclusive_mode {
	IPC4K_ALLOWED = 0,
	IPC64_EXCLUSIVE = 1,
};
enum base_fw_runtime_params_ids {
	BASE_FW_FIRMWARE_DMA_CONTROL = 5, BASE_FW_FIRMWARE_CONFIG = 7,
};

/* Commands(Larg config IPC param ids) for Probe module */
enum sue_cmds_probe_mod {
	SUE_CMD_PROBE_INJECTION_DMA_ATTACH = 1,
	SUE_CMD_PROBE_INJECTION_DMA_DETACH = 2,
	SUE_CMD_PROBE_POINT_CONNECT = 3,
	SUE_CMD_PROBE_POINT_DISCONNECT = 4,
	SUE_CMD_PROBE_GTW_INIT = 5,
};

enum sue_fw_load_status {
	SUE_UNINIT = 0,
	SUE_ROM_READY_DONE = 1,
	SUE_RET_DEL_SET_DONE = 2,
	SUE_FW_READY_DONE = 3,
	SUE_CONFIG_LOAD_DONE = 4,
	SUE_FW_TOPOLOGY_START_DONE = 5,
	SUE_FW_TOPOLOGY_STOP_DONE = 6,
};

enum sue_external_playback_state {
	SUE_EXT_PLAYBACK_STATE_UNDEFINED = 0,
	SUE_EXT_PLAYBACK_STATE_STARTED = 1,
	SUE_EXT_PLAYBACK_STATE_STOPPED = 2,
	SUE_EXT_PLAYBACK_STATE_PAUSED = 3,
	SUE_EXT_PLAYBACK_STATE_COUNT,
};

typedef enum s1000_gpio_cmd {
	S1000_GPIO_INIT = 0x30, S1000_GPIO_GET = 0x31, S1000_GPIO_SET = 0x32,
} s1000_gpio_cmd_t;

typedef enum s1000_i2c_cmd {
	S1000_I2C_WRITE_CHK = 0x20, S1000_I2C_READ = 0x21, S1000_I2C_WRITE = 0x22, S1000_I2C_SPEED_SET = 0x27,
} s1000_i2c_cmd_t;

#endif

