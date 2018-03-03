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
#ifndef SPICOMMON_H
#define SPICOMMON_H

#include "pal_os_specific.h"
#include "spi_protocol.h"

#if defined(__cplusplus)
extern "C" {
#endif


	typedef enum
	{
		SUE_STREAM_TRIGGER_ON_NOTIFICATION,
		SUE_STREAM_TRIGGER_ON_RESPONSE
	} trigger_t;

	typedef enum {
		IPC_EVENT_DMA_ERROR,
		IPC_EVENT_LOW_THRESHOLD,     /*!< SPI low threshold reached*/
		IPC_EVENT_HIGH_THRESHOLD,     /*!< SPI high threshold reached*/
		IPC_EVENT_MAX,         /*!< SPI max event index*/
	} ipc_event_type_t;

	enum spi_obj_states
	{
		SPI_OBJ_STATE_UNSET		  		= 0,
		SPI_OBJ_STATE_START_DONE	    = 1,
		SPI_OBJ_STATE_READ_WRITE		= 2,
		SPI_OBJ_STATE_DRAINING_DONE	    = 3,
		SPI_OBJ_STATE_BUFFER_PROCESSED	= 4,
		SPI_OBJ_STATE_STOP	  	    	= 5,
		SPI_OBJ_STATE_STOP_DONE	    	= 6,
		SPI_OBJ_STATE_DESTROYED			= 7,
	};

	typedef struct spi_transfer_buf
	{
		uint8_t* ptr;
		uint32_t size;
		uint32_t used_size;
	} spi_transfer_buf_t;

	typedef enum spi_dir_ {
		spi_dir_out_, spi_dir_in_
	} spi_dir_t;

	/*!\brief Specifies various audio events returned by the
	*           stream.
	*/
	typedef enum _dsp_event {
	    /*! The audio event to client in case of underrun */
	    DSP_EVENT_UNDERRUN = 0,
	    /*! The audio event to client in case of overrun */
		DSP_EVENT_OVERRUN,
		/*! The audio event to client in case of asynchronous request completion */
		DSP_EVENT_REQUEST_COMPLETED,
	    /*! The audio event signaling fatal error in processing */
		DSP_EVENT_FATAL_ERROR,
	    /*! The audio event count */
		DSP_EVENTS_COUNT
	} dsp_event_t;

	/*!
	 * \brief Defines the call back function pointer structure
	*/
	typedef void(*event_callback_t)(const void *user_param, dsp_event_t event_type, void *args);

	/*!
	 * \brief Defines the call back info which consists of user context and callback function
	*/
	typedef struct callback_info {
		void *user_param;
		event_callback_t callback;
	} callback_info_t;

	typedef struct {
		uint8_t num_channels;
		uint8_t bit_depth;
		uint8_t valid_bit_depth;
		uint32_t sampl_freq;

	} gtw_audio_format_t;

	typedef	struct spi_obj_handle
	{
		spi_transfer_buf_t    	transfer_buf;
#ifdef TX_TASK
		MUTEX_T             	cs_mutex;
#endif
		SEMAPHORE_T    	        stop_mutex;
		spi_protocol_t*         spi_proto;
		callback_info_t     	event_callbacks[DSP_EVENTS_COUNT];
		gtw_audio_format_t 	    audio_format;
		uint16_t        	    gtw_id;
		enum spi_obj_states 	state;
		trigger_t trigger;
		bool no_buffer_on_notif;
	} spi_obj_handle_t;


#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif

