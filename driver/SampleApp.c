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
#include "SampleApp.h"
#include "dsp_driver.h"
#include "pal_os_specific.h"

#ifdef FREERTOS
#include "esp_partition.h"
#include "esp_vfs_fat.h"
#else
#define PATHLENGTH 256
#endif

const char *LOG_TAG = "singlethreadedapp";

static SEMAPHORE_T wovSem = NULL;
static SEMAPHORE_T isrSem = NULL;
static SEMAPHORE_T captureSem = NULL;

#ifdef BOOT_SPI_S
#ifdef DFU
#error "DFU only possible in SPI-M boot mode, undef BOOT_SPI_S"
#endif
#endif

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

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define WAIT_WHILE_IDLE(TIMEOUT) (dsp_driver_is_protocol_idle() && !SemaphoreTake(isrSem, TIMEOUT))

static void IRAM_ATTR isr_callback(void* ctx)
{
#if !defined(TX_TASK) && defined(FREERTOS)
	BaseType_t xHigherPriorityTaskWoken = false;
	xSemaphoreGiveFromISR(isrSem, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken)
		portYIELD_FROM_ISR();
#endif
}

static void wov_callback_function(const void *user_param,dsp_event_t event_type,dsp_evt_cb_hlv_t *event_args){
	LOGI("wov_callback_function");
	SemaphoreGive(wovSem);
}

static void capture_buf_completion_callback(const void *user_param,dsp_event_t event_type,dsp_evt_cb_hlv_t *event_args){
	LOGI("capture_buf_completion_callback");
	SemaphoreGive(captureSem);
}

#if defined(BOOT_SPI_S) || defined(DFU)
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

void app_start() {
	LOGI("Started");
	wovSem = CreateSemaphore();
	isrSem = CreateSemaphore();
	captureSem = CreateSemaphore();
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
	err = dsp_driver_init(isr, BOOT_SPI_MASTER);
	if (err)
	{
		LOGE("Could not init dsp driver");
		return;
	}

#ifdef DFU
#ifdef FREERTOS
	char fpath[PATHLENGTH];
	makeFullPath(fpath, sizeof(fpath), MNT_POINT, "DFU.BIN");
#else
	const char *fpath = "dfu.bin";
#endif
	FILE *ff = fopen(fpath, "rb");
	if (!ff)
	{
		LOGE("Could not open upgrade fw img file.");
		return;
	}
	fseek(ff, 0, SEEK_END);
	uint32_t size = ftell(ff);
	rewind(ff);

	err = dsp_driver_load_img(LOAD_FUNC_DFU, read_img_chunk_callback, ff, size, NULL);
	if (err)
	{
		LOGE("Could not upgrade fw.");
		return;
	}
	fclose(ff);
#endif

#ifdef BOOT_SPI_S
#ifdef FREERTOS
	char fpath[PATHLENGTH];
	makeFullPath(fpath, sizeof(fpath), MNT_POINT, "LP1440.BIN");
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

	err = dsp_driver_load_img(LOAD_FUNC_BOOTSTRAP, read_img_chunk_callback, ff, size, NULL);
	if (err)
	{
		LOGE("Could not bootstrap fw file.");
		return;
	}
	fclose(ff);
#ifdef FREERTOS
	char bpath[PATHLENGTH];
	makeFullPath(bpath, sizeof(bpath), MNT_POINT, "LPBL1440.BIN");
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
	cb_ops.notif_callbacks[NOTIF_WOV_DETECTED] = (notif_callback_info_t) {.user_param = NULL, .callback = (notif_callback_t)wov_callback_function};
	cb_ops.gtw_event_callbacks[DSP_EVENT_REQUEST_COMPLETED] = (callback_info_t) {.user_param = NULL, .callback = (event_callback_t)capture_buf_completion_callback};

#ifdef BOOT_SPI_S
	err = dsp_driver_configure(DIALECT_NOT_SPECIFIED, &cb_ops);
#else
	err = dsp_driver_configure(DIALECT_EN_US, &cb_ops);
#endif
	if (err)
	{
		LOGE("Could not configure dsp driver");
		return;
	}

	err = dsp_driver_start();
	if (err)
	{
		LOGE("Could not start dsp driver");
		return;
	}

	/*
	 * DSP_CAPTURE_STATE_WOV for keyphrase recognition,
	 * DSP_CAPTURE_STATE_STREAM for direct data capture
	 **/
	dsp_capture_state_t capture_state = DSP_CAPTURE_STATE_WOV;

	// get FW version
	buf_desc_t* fw_version_bd;
	fw_version_bd = (buf_desc_t*)malloc(sizeof(buf_desc_t) + sizeof(struct dsp_fw_version));
	if(!fw_version_bd) {
		LOGE("Could not allocate memory");
		return;
	}
	fw_version_bd->data_size = sizeof(struct dsp_fw_version);
	fw_version_bd->valid_data_size = 0;

	dsp_driver_get_fw_version(fw_version_bd);

	struct dsp_fw_version* fw_version = (struct dsp_fw_version*) fw_version_bd->data;
	LOGI("FW VERSION: %u.%u.%u.%u", fw_version->major_version, fw_version->minor_version, fw_version->hotfix_version, fw_version->build_version);
	free(fw_version_bd);

	while (1)
	{
		err = dsp_driver_start_stream(REFERENCE_STREAM_ID, NULL, NULL);
		if (err)
		{
			LOGE("Could not start reference stream");
			return;
		}

		err = dsp_driver_external_playback_start();
		if (err)
		{
			LOGE("Could not notify dsp about playback start");
			return;
		}

		LOGI("Starting capture.");
		err = dsp_driver_set_state(capture_state);
		if (err)
		{
			LOGE("Could not set capture state.");
			return;
		}
		if (capture_state == DSP_CAPTURE_STATE_WOV)
		{
			LOGI("Waiting for keyphrase notification...");
#ifndef TX_TASK
			do
			{
				if (WAIT_WHILE_IDLE(30000))
				{
					LOGE("NO ISR for keyphrase notification!");
					proto_err err = dsp_driver_stop_stream(REFERENCE_STREAM_ID);
					if (err)
						LOGE("Could not stop reference stream");

					err = dsp_driver_external_playback_stop();
					if (err)
						LOGE("Could not notify dsp about playback stop");
					dsp_driver_deinit();
					return;
				}
				dsp_driver_schedule();

			} while (!SemaphoreTake(wovSem, 1));
#else
			if (!SemaphoreTake(wovSem, 30000))
			{
				LOGE("NO ISR for keyphrase notification!");
				proto_err err = dsp_driver_stop_stream(REFERENCE_STREAM_ID);
				if (err)
					LOGE("Could not stop reference stream");

				err = dsp_driver_external_playback_stop();
				if (err)
					LOGE("Could not notify dsp about playback stop");

				dsp_driver_deinit();
				return;
			}
#endif
		}

		LOGI("Waiting for notification from capture stream");
#ifndef TX_TASK
		do
		{
			if (WAIT_WHILE_IDLE(2000))
			{
				LOGE("NO ISR for notification from capture stream !");
				return;
			}
			dsp_driver_schedule();
		} while (!SemaphoreTake(captureSem, 1));
#else
		if (!SemaphoreTake(captureSem, 2000))
		{
			LOGE("NO notification from capture stream !");
			return;
		}
#endif
		LOGI("Got notification from capture stream.");


		uint32_t size = 64;
		char *buff = (char*)malloc(size);
		if (!buff)
		{
			LOGE("Memory allocation failed");
			return;
		}
#ifdef FREERTOS
		char path[PATHLENGTH];
		makeFullPath(path, sizeof(path), MNT_POINT, "CAP.RAW");
#else
		const char *path = "cap.raw";
#endif

		FILE *fd = fopen(path, "wb");
		if (!fd)
		{
			LOGE("Cannot open file.");
			free(buff);
			return;
		}

		uint32_t *read;
		uint32_t written;
		uint32_t fsize = 0;
		while (fsize < 4096 * 10)
		{
			LOGI("Reading data...");
			dsp_driver_read_data(0, buff, size, &read);

#ifndef TX_TASK
			LOGI("Waiting for reading to complete. Scheduling...");
			while (!SemaphoreTake(captureSem, 1))
				dsp_driver_schedule();
#else
			if (!SemaphoreTake(captureSem, 2000))
			{
				LOGE("NO notification from capture stream !");
				return;
			}
#endif
			LOGI("Reading data done, read %d bytes, write content to file.", *read);

			written = fwrite(buff, 1 , *read, fd);
			LOGI("written %d, fsize %d", written, fsize);
			if (written != *read)
			{
				LOGE("File write failed. Wrote %u out of %u!", written, *read);
				break;
			}
			fsize += written;
		}
		fclose(fd);
		free(buff);

		LOGI("Setting capture state: IDLE");
		err = dsp_driver_set_state(DSP_CAPTURE_STATE_IDLE);
		if (err)
		{
			LOGE("Could not set capture state idle.");
			return;
		}

		err = dsp_driver_stop_stream(REFERENCE_STREAM_ID);
		if (err)
		{
			LOGE("Could not stop reference stream");
			return;
		}

		err = dsp_driver_external_playback_stop();
		if (err)
		{
			LOGE("Could not notify dsp about playback stop");
			return;
		}

	}

	LOGI("Exiting.");

	SemaphoreDelete(captureSem);
	SemaphoreDelete(isrSem);
	SemaphoreDelete(wovSem);
}

int main()
{
	app_start();
	return 0;
}
