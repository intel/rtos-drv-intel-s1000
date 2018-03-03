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
#ifndef SPISINK_H
#define SPISINK_H

#include "spi_gtw/spi_common.h"
#include "spi_lib_error.h"

#if defined(__cplusplus)
extern "C" {
#endif

	proto_err spi_sink_init(spi_obj_handle_t* spi_obj, spi_protocol_t* spi_proto);
	proto_err spi_sink_deinit(spi_obj_handle_t* spi_obj);
    proto_err spi_sink_write(spi_obj_handle_t* spi_obj, uint8_t* src_addr, uint32_t size, uint32_t** written);
    proto_err spi_sink_start(spi_obj_handle_t* spi_obj);
    proto_err spi_sink_stop(spi_obj_handle_t* spi_obj);
    proto_err spi_sink_seteventcallback(spi_obj_handle_t* spi_obj, void *user_param,
		dsp_event_t event_type, event_callback_t callback);

#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif

