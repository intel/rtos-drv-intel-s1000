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

#ifndef SPI_LIB_ERROR_H
#define SPI_LIB_ERROR_H

// return error is return value is not success
#define RET_ON_ERROR_LOG(ret_val, error_code,  format, ...) \
	do { \
		if (ret_val) { \
			LOGE("Code returned %d. " format , ret_val,  ##__VA_ARGS__); \
			return error_code; \
		}\
	} while (0)


#define RET_ON_PROTO_ERROR_LOG(error_code,  format, ...) \
	RET_ON_ERROR_LOG(error_code, error_code, format, ##__VA_ARGS__)

#define RET_ON_ERROR(ret_val) \
	do { \
		if (ret_val) { \
			return ret_val; \
		}\
	} while (0)

typedef int32_t proto_err;
typedef enum spi_error_codes_t
{
	PROTO_SUCCESS,
	PROTO_NULL_POINTER,
	PROTO_OUT_OF_MEMORY,
	PROTO_SPI_CONFIGURE_FAILED,
	PROTO_SPI_TRANSFER_FAILED,
	PROTO_GPIO_CONFIGURE_FAILED,
	PROTO_SHA_MISMATCH,
	PROTO_DSP_NOT_RESPONSIVE,
	PROTO_HANDLER_ADD_REMOVE_FAILURE,
	PROTO_IGNORE_REQUEST,
	PROTO_INVALID_CONFIGURATION,
	PROTO_IPC_ERR,
	PROTO_IPC_TIMEOUT,
	PROTO_INVALID_PARAM,
	PROTO_GTW_INVALID_STATE,
	PROTO_GTW_TIMEOUT,
}spi_error_codes;

#endif
