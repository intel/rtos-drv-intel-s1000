
# Sue Creek RTOS driver
MCU/RTOS driver for Sue Creek speech and audio processor. This driver implements the APIs to applications and the underlying control mechanisms for managing audio capture, playback, wakeword notification, etc.
This driver has been tested on both FreeRTOS and Linux, it is possible to port it to other RT operating systems.

# Contents:
- [Package contents](#package-contents)
- [Protocol integration guide](#sue-creek-protocol-library-integration-guide)  
   - [SampleApp scenario](#sampleapp-scenario)
- [RTOS driver API reference](#sue-creek-rtos-driver-api)
   - [Driver API](#driver-api)
   - [Flow diagrams](#flow-diagrams)

## Package contents

### driver/

Audio driver for Intel S1000 (Sue Creek). 
Contains test application for testing the various audio features supported by the driver and illustrating the use of APIs exposed by the driver to the application.

### spi_proto/

Core library implementing IPC protocol over SPI interface. Its primary role is to encapsulate host-to-DSP IPC communication. It is intended to be used with an MCU with an SPI master interface running an RTOS or as a Linux user space shared object with dependencies on C++ std library (thread, deque).


## Sue Creek protocol library integration guide

1.	Define types and create empty functions required by `pal_os_specific.h` and `peripherals_os_specific.h`:
    * Create subdirectory specific for your platform. See Linux or FreeRTOS examples.
    * Create new implementation of pal and peripherials according to api defined for FreeRTOS and Linux.  

    Purpose of this step is to create complete set of required files. Source code content will be completed in following steps. As a result source code won’t be functional but it should compile.

2.	Implement platform specific version of logging functions (LOGE, LOGW, LOGI, LOGD).  
See: logging macros prototype in `Linux/pal.h`

3.	Define pin numbers in peripherals.h  
    ```c
    #define SPI_MISO (0)
    #define SPI_MOSI (0)
    #define SPI_CLK (0)
    #define SPI_CS_HW (0)
    #define GPIO_IRQ (0)
    #define GPIO_RST (0)
    #define GPIO_WAKE (0)
    ```

4.	Implement 
    ```c
    int peripherals_init(spi_device_t* spi_dev, spi_device_settings_t* settings, group_event_t* event)
    ```
    in `peripherals.c`
    * Configure gpios (level and edge) for irq, rst and wake pins –pin numbers available in settings parameter
    * Configure isr for irq pin. See example in `FreeRTOS/peripherals.c#gpio_isr_handler`
    * Configure spi device

5.	Create queue and eventgroup @see: `pal.h`
    * As queue consists only of two elements and does not need to support multithreading it may be just a simple structure as below. `QUEUE_HANDLE_T` is used in `spi_porotocol_t` structure for ipcs and workitems:
    ```c
    Struct queue
    {
        void* ptr1;
        void* ptr2;
    }
    ```
    * Create eventgroup. Refer to FreeRTOS or Linux implementation. For single threaded mode two bits are required: `IRQ_EVENT` and `ISR_FORWARD`. 
      * `ISR_FORWARD` bit is set in `EVENT_GROUP_T` from `spi_protocol.c#enter_idle/exit_idle`. `ISR_FORWARD` means that dsp went idle. Detection of rising edge while `ISR_FORWARD` is set means that FW has notification pending and application should take action.
      * `IRQ_EVENT` – is set from isr and read explicitly by `EventGroupWaitBits` call.

6.	At this stage it should be possible to successfully start application and wake FW with key phrase. 
    * As a first approach eventgroup or isr may be replaced with a simple delay.  
    See `spi_protocol.c#reset_platfrom`: `wait_for_irq` can be replaced by `task_delay()`.  
    If `SampleApp.c#isrSem` is never set make sure that isr is correctly configured for gpio irq pin.


### SampleApp scenario:
1. Loads audio FW using SPI-S interface or boots from SPI-M flash - `dsp_driver_init`
2. Loads FW configuration file through SPI-S interface or selects configuration from SueCreek's flash, according to specified dialect index - `dsp_driver_configure`
3. Initializes driver's audio gateway - `dsp_driver_start`
4. Starts SueCreek's reference I2S stream, required for Acoustic Echo Cancellation - `dsp_driver_start_stream`
5. Notifies SueCreek about start of playback, it needs to be called just before actual playback begins - `dsp_driver_external_playback_start`
6. Sets capture pipeline state, requested state is either WOV or STREAM as a result capture endpoint (spi_source) is prepared for data capture - `dsp_driver_set_state(DSP_CAPTURE_STATE_WOV)`
7. In a loop:
   1. Reads audio data - `dsp_driver_read_data`
   2. Stores audio data in a file
8. Sets capture pipeline state to IDLE and stops data capture - `dsp_driver_set_state(DSP_CAPTURE_STATE_IDLE)`
9. Stops SueCreek's reference I2S stream - `dsp_driver_stop_stream`
10. Notifies SueCreek about stop of playback, it needs to be called just after actual playback stops - `dsp_driver_external_playback_stop`

## Sue Creek RTOS Driver API

### Driver Api:

|**Method**                      |**Params**                 |**Description**                                                                |
|--------------------------------|---------------------------|-------------------------------------------------------------------------------|
|`dsp_driver_init`               | (irq_handler_t, uint32_t)  |initializes the driver, performs dsp reset                                     |
|`dsp_driver_deinit`             |                           |deinitializes the driver, stops topology, unregisters notification             |
|`dsp_driver_configure`          | (dsp_dialect_t, evt_cb_ops_t*)         |sets language dialect, sets wov, ipc error, xrun, buffer completion callbacks  |
|`dsp_driver_start`              |                           |sends TOPOLOGY_START message                                                   |
|`dsp_driver_mfg_start`          |                           |registers to MFG_TEST_END notification                                         |
|`dsp_driver_read_data`          | (uint, void *, uint, uint32_t **) |non-blocking, sets buffer chunk to be filled with captured audio data          |
|`dsp_driver_write_data`         | (uint, void *, uint, uint32_t **) |non-blocking, sets audio data buffer chunk to be used for playback             |
|`dsp_driver_schedule`           |                           |execute scheduled non-blocking actions (read_audio_data, write_audio_data)     |
|`dsp_driver_set_state`          | (dsp_capture_state_t)           |sets capture state                                                             |
|`dsp_driver_start_stream`       | (uint, gtw_audio_format_t *, callback_info_t)  |start fw stream (playback, external reference)           |
|`dsp_driver_stop_stream`        | (uint32_t)               | stops fw stream                                                               |
|`dsp_driver_load_img`           | (load_function_t, read_img_chunk_cb, void*, uint32_t, void*)      | initialize loading binary(fw, dfu, config load, config save) |
|`dsp_driver_voice_stream_start` |                           |starts voice stream                                                            |
|`dsp_driver_voice_stream_stop`  |                           |stops voice stream                                                             |
|`dsp_driver_external_playback_start`|                       |starts external playback                                                       |
|`dsp_driver_external_playback_stop` |                       |stops external playback                                                        |
|`dsp_driver_mfg_run_test`       | (uint32_t, uint32_t)              |runs step X in test Y                                                          |
|`dsp_driver_mfg_test_result`    | (uint32_t, uint32_t, buf_desc_t*)          |reads test step result into bd                                                 |
|`dsp_driver_get_fw_version`     | (buf_desc_t*)                      |reads FW version                                                               |
|`dsp_driver_mic_mute`           | (bool)                 |toggles mic mute                                                               |

