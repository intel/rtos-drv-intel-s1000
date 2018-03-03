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

#ifndef _MFG_DRIVER_H
#define _MFG_DRIVER_H

#define MFG_COMMAND_TEST_START 1
#define MFG_COMMAND_TEST_STOP 2
#define INTEL_MFG_CHANNELS_CNT 2

#define MFG_TEST_MIC_THD 1
#define MFG_TEST_REF_THD 2
#define MFG_TEST_MIC_RMS 3
#define MFG_TEST_MIC_REF_DELAY 5

typedef struct mfg_test_ctl_data_ {
	uint32_t command;
	uint32_t test_id;
	uint32_t test_step;
} mfg_test_ctl_data_t;

struct _IntelMfgRmsResponse
{
	float rms1;
	float rms2;
	float total_rms;
};

struct IntelMfgRmsResponse
{
	bool isDataValid;
	struct _IntelMfgRmsResponse response[INTEL_MFG_CHANNELS_CNT];
};

struct _IntelMfgThdResponse
{
	float amplitude_1kHz;
	float frequency_with_max_amplitude;
	float max_amplitude;
	float thd;
};

struct IntelMfgThdResponse
{
	bool isDataValid;
	struct _IntelMfgThdResponse response[INTEL_MFG_CHANNELS_CNT];
};

#endif /* _MFG_DRIVER_H */
