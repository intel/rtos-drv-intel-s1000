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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dsp_driver.h"
#include "mfg_driver.h"
#include "pal_os_specific.h"

#ifdef FREERTOS
#include "esp_partition.h"
#include "esp_vfs_fat.h"
#else
#define PATHLENGTH 256
#endif

#define BOOT_SPI_S
const char *LOG_TAG = "mfg-app";

static SEMAPHORE_T isrSem = NULL;
static SEMAPHORE_T mfgSem = NULL;

void mfg_test_mic_rms();
void mfg_test_mic_thd();
void mfg_test_ref_thd();
void mfg_test_mic_ref_delay();

#ifdef FREERTOS
void concatstr(char * const dst, const size_t dstSize, const char *const src){
	if( dstSize > 0 ) {
		size_t free = dstSize - strlen(dst) -1 ;
		strncat(dst, src, free);
		dst[dstSize-1] = '\0';
	}
}

void copystr(char * const dst, const size_t dstSize, const char *const src){
	if( dstSize > 0 ) {
		size_t free = dstSize -1 ;
		strncpy(dst, src, free);
		dst[dstSize-1] = '\0';
	}
}

// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition:
#define MNT_POINT "/spiflash"
// Maximum lenght of fully-qualified file name, i.e.: "/12345678.123/12345678.123"
#define PATHLENGTH (2*(1+8+1+3)+1)

void makeFullPath(char * const fullName, const size_t size, const char * const dirName, const char * const fName){
	copystr(fullName, size, dirName);
	if( '/' != dirName[strlen(dirName)-1] )
		concatstr(fullName, size, "/");
	concatstr(fullName, size, fName);
}

esp_err_t mount_fatfs()
{
	// To mount device we need name of device partition, define MNT_POINT
	// and allow format partition in case if it is new one and was not formated before
	const esp_vfs_fat_mount_config_t mount_config = {
			.max_files = 4,
			.format_if_mount_failed = true
	};
	return esp_vfs_fat_spiflash_mount(MNT_POINT, "storage", &mount_config, &s_wl_handle);
}
#endif

#define WAIT_WHILE_IDLE(TIMEOUT) (dsp_driver_is_protocol_idle() && !SemaphoreTake(isrSem, TIMEOUT))

static void isr_callback(void* ctx)
{
#if !defined(TX_TASK) && defined(FREERTOS)
	BaseType_t xHigherPriorityTaskWoken = false;
	xSemaphoreGiveFromISR(isrSem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken)
		portYIELD_FROM_ISR();
#endif
}

static void mfg_callback_function(const void *user_param,dsp_event_t event_type,dsp_evt_cb_hlv_t *event_args){
	LOGI("mfg_callback_function");
	SemaphoreGive(mfgSem);
}

#ifdef BOOT_SPI_S
static uint16_t read_img_chunk_callback(void* ctx, uint8_t* buf, uint16_t size)
{
	FILE* fd = (FILE*) ctx;
	return fread(buf, 1, size, fd);
}
#endif

#ifdef FREERTOS
proto_err init_fs() {
	spi_flash_init();
	int32_t  err = mount_fatfs();
	if( err ){
		LOGE("mount_fatfs failed");
		return err;
	}

	return PROTO_SUCCESS;
}
#endif

void app_main() {
	LOGI("Started");
	isrSem = CreateSemaphore();
	mfgSem = CreateSemaphore();
	proto_err err = PROTO_SUCCESS;
#ifdef FREERTOS
	err = init_fs();
	if (err)
	{
		LOGE("Could not init file system");
		return;
	}
#endif

	irq_handler_t isr = { .isr_callback = isr_callback, .ctx = NULL };
	err = dsp_driver_init(isr, BOOT_SPI_SLAVE);
	if (err)
	{
		LOGE("Could not init dsp driver");
		return;
	}

#ifdef BOOT_SPI_S
#ifdef FREERTOS
	char fpath[PATHLENGTH];
	makeFullPath(fpath, sizeof(fpath), MNT_POINT, "MFG.FW");
#else
	const char *fpath = "crb1438.bin";
#endif
	FILE *ff = fopen(fpath, "rb");
	if (!ff)
	{
		LOGE("Could not open fw img file.");
		return;
	}
	fseek(ff, 0, SEEK_END);
	uint32_t size = ftell(ff);
	rewind(ff);

	LOGI("Load fw");
	err = dsp_driver_load_img(LOAD_FUNC_BOOTSTRAP, read_img_chunk_callback, ff, size, NULL);
	if (err)
	{
		LOGE("Could not bootstrap fw file.");
		return;
	}
	fclose(ff);
#ifdef FREERTOS
        char bpath[PATHLENGTH];
        makeFullPath(bpath, sizeof(bpath), MNT_POINT, "MFGBL.BIN");
#else
        const char *bpath = "enus1438.bin";
#endif
        ff = fopen(bpath, "rb");
        if (!ff)
        {
                LOGE("Could not open config blob file.");
                return;
        }
        fseek(ff, 0, SEEK_END);
        size = ftell(ff);
        rewind(ff);

        err = dsp_driver_load_img(LOAD_FUNC_CFG_LOAD, read_img_chunk_callback, ff, size, NULL);
        if (err)
        {
                LOGE("Could not load config blob file.");
                return;
        }
        fclose(ff);
#endif

	evt_cb_ops_t cb_ops;
	memset(&cb_ops, 0, sizeof(cb_ops));
	cb_ops.notif_callbacks[NOTIF_MFG_TEST_END] = (notif_callback_info_t) {.user_param = NULL, .callback = (notif_callback_t)mfg_callback_function};

	dsp_driver_configure(DIALECT_NOT_SPECIFIED, &cb_ops);

	dsp_driver_mfg_start();

	// buffer for storing test results returned via ipc response
	buf_desc_t* bd;
	uint32_t pld_size = 56;
	bd = (buf_desc_t*)malloc(sizeof(buf_desc_t) + pld_size);
	if(!bd) {
		LOGE("Could not allocate memory");
		return;
	}
	bd->data_size = pld_size;
	bd->valid_data_size = 0;

	mfg_test_mic_rms(bd);
	mfg_test_mic_thd(bd);
	mfg_test_ref_thd(bd);
	mfg_test_mic_ref_delay();

	free(bd);
	SemaphoreDelete(isrSem);
	SemaphoreDelete(mfgSem);
}

void mfg_wait_for_test_end(int timeout)
{
	#ifndef TX_TASK
		do
		{
			if(WAIT_WHILE_IDLE(timeout))
			{
				LOGE("NO ISR for MFG test end notification!");
				return;
			}
			dsp_driver_schedule();

		} while(!SemaphoreTake(mfgSem, 1));
	#else
		if(!SemaphoreTake(mfgSem, timeout))
		{
			LOGE("NO ISR for MFG test end notification!");
			return;
		}
	#endif
}

// measure mics thd factor
// 1khz sine signal played to the mics is required
void mfg_test_mic_thd(buf_desc_t* bd)
{
	LOGI("Running %s", __func__);
	proto_err err;
	bd->data_size = sizeof(struct IntelMfgThdResponse);

	err = dsp_driver_mfg_run_test(MFG_TEST_MIC_THD, 0);
	if (err)
	{
		LOGE("error: starting mic thd test failed");
		return;
	}

	mfg_wait_for_test_end(15000);

	err = dsp_driver_mfg_test_result(MFG_TEST_MIC_THD, 0, bd);
	if (err)
	{
		LOGE("error: getting mic thd test results failed");
		return;
	}

	struct IntelMfgThdResponse* mic_thd = (struct IntelMfgThdResponse*) bd->data;
	if (!mic_thd->isDataValid)
	{
		LOGE("test hasnt't finished yet, increase delay between test_run and test_result");
		return;
	}

	LOGI("Test results:");
	for (int i = 0; i < INTEL_MFG_CHANNELS_CNT; i++)
	{
		struct _IntelMfgThdResponse* ch = (struct _IntelMfgThdResponse*) &mic_thd->response[i];
		LOGI("ch%d: thd %f, amp_1khz: %f, freq_with_max_amp: %f, max_amp: %f",
				i+1, ch->thd, ch->amplitude_1kHz, ch->frequency_with_max_amplitude, ch->max_amplitude);
	}

}

void mfg_test_mic_rms(buf_desc_t* bd)
{
	LOGI("Running %s", __func__);
	proto_err err;
	bd->data_size = sizeof(struct IntelMfgRmsResponse);

	// step 1 - sealed mics
	err = dsp_driver_mfg_run_test(MFG_TEST_MIC_RMS, 1);
	if (err)
	{
		LOGE("error: starting mic rms test failed on step 1");
		return;
	}

	mfg_wait_for_test_end(5000);

	// step 2 - open mics
	err = dsp_driver_mfg_run_test(MFG_TEST_MIC_RMS, 2);
	if (err)
	{
		LOGE("error: starting mic rms test failed on step 2");
		return;
	}

	mfg_wait_for_test_end(5000);

	err = dsp_driver_mfg_test_result(MFG_TEST_MIC_RMS, 0, bd);
	if (err)
	{
		LOGE("error: getting mic rms test results failed");
		return;
	}

	struct IntelMfgRmsResponse* mic_rms = (struct IntelMfgRmsResponse*) bd->data;
	if (!mic_rms->isDataValid)
	{
		LOGE("test hasnt't finished yet, increase delay between test_run and test_result");
		return;
	}

	LOGI("Test results:");
	for (int i = 0; i < INTEL_MFG_CHANNELS_CNT; i++)
	{
		struct _IntelMfgRmsResponse* ch = (struct _IntelMfgRmsResponse*) &mic_rms->response[i];
		LOGI("ch%d: rms_sealed %f, rms_open: %f, delta: %f",
				i+1, ch->rms1, ch->rms2, ch->total_rms);
	}

}

// measure refereence path thd factor
// 1khz sine signal played on reference i2s interface is required
void mfg_test_ref_thd(buf_desc_t* bd)
{
	LOGI("Running %s",__func__);
	proto_err err;
	bd->data_size = sizeof(struct IntelMfgThdResponse);

	err = dsp_driver_mfg_run_test(MFG_TEST_REF_THD, 0);
	if (err)
	{
		LOGE("error: starting ref thd test failed");
		return;
	}

	mfg_wait_for_test_end(15000);

	err = dsp_driver_mfg_test_result(MFG_TEST_REF_THD, 0, bd);
	if (err)
	{
		LOGE("error: getting ref thd test results failed");
		return;
	}

	struct IntelMfgThdResponse* ref_thd = (struct IntelMfgThdResponse*) bd->data;
	if (!ref_thd->isDataValid)
	{
		LOGE("test hasnt't finished yet, increase delay between test_run and test_result");
		return;
	}

	LOGI("Test results:");
	for (int i = 0; i < INTEL_MFG_CHANNELS_CNT; i++)
	{
		struct _IntelMfgThdResponse* ch = (struct _IntelMfgThdResponse*) &ref_thd->response[i];
		LOGI("ch%d: thd %f, amp_1khz: %f, freq_with_max_amp: %f, max_amp: %f",
				i+1, ch->thd, ch->amplitude_1kHz, ch->frequency_with_max_amplitude, ch->max_amplitude);
	}

}

// this test requires connecting external logical analyzer to I2S1 interface
// and manual checking for signal offseting between mic channel and reference channel
// test does not send any results
// adjust test time to your preference
void mfg_test_mic_ref_delay()
{
	LOGI("Running %s", __func__);
	proto_err err;
	// step 1 - enable mclk 19.2MHz
	err = dsp_driver_mfg_run_test(MFG_TEST_MIC_REF_DELAY, 0);
	if (err)
	{
		LOGE("error: starting mic ref delay test failed on step 0");
		return;
	}

	// step 2 - configure I2S1
	err = dsp_driver_mfg_run_test(MFG_TEST_MIC_REF_DELAY, 1);
	if (err)
	{
		LOGE("error: starting mic ref delay test failed on step 1");
		return;
	}

	TaskDelay(60 * 1000);

	err = dsp_driver_mfg_test_result(MFG_TEST_MIC_REF_DELAY, 0, NULL);
	if (err)
	{
		LOGE("error: finishing mic ref delay test failed");
		return;
	}

	LOGI("Test finished");
}

int main()
{
	app_main();
	return 0;
}
