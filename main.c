/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bsec_integration.h"
#include "math.h"



#define TWI_INSTANCE_ID   0
#define BME680_ADDR       0x77U
#define MAX44009_ADDR			0x4AU
#define MAX44009_EN_PIN		3
#define MAX44009_INT_PIN	19
#define BME680_EN_PIN			4
#define MEIN_TIMER_INTERVAL         APP_TIMER_TICKS(60000)              // 20s

static ble_advdata_t advdata;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
APP_TIMER_DEF(m_sleep_timer_id);
uint8_t max44009_data[8]	= {0};
uint8_t	sleep_timer_state	= 0;
uint64_t	system_time			=	0;
uint16_t	bme680_temp_offset	= 0;
uint32_t	bme680_press				= 0;
uint32_t	bme680_humi					= 0;
uint16_t	bme680_quali				= 0;
uint32_t	max44009_illu				= 0;
uint8_t		ble_batt						= 0;
uint16_t	feuchte			= 0;


void twi_init (void);
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint8_t data_len);
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint8_t data_len);
static void sleep_timer_timeout_handler(void * p_context);
static void application_timers_start(uint32_t time);
void max44009_read(void);
uint32_t calc_lux(uint8_t *data);
void sleep(uint32_t t_ms);
int64_t get_timestamp_us(void);
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity, float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status);


////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "11223344556677889900"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           1                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                160//64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      5 //BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_sleep_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sleep_timer_timeout_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    //ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &srdata);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t     err_code;
    ble_lbs_init_t init;

    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;
				
				case BLE_GAP_EVT_SCAN_REQ_REPORT:
						__ASM("nop");
						break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
#if (__FPU_USED == 1)
__set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
(void) __get_FPSCR();
NVIC_ClearPendingIRQ(FPU_IRQn);
#endif
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
		uint32_t  err_code;
    int64_t time_stamp = 0;
//    int64_t time_stamp_trigger = 0;
    int64_t time_stamp_interval_ms = 0;    
    bsec_input_t bsec_inputs[5];    
    uint8_t num_bsec_inputs = 0;    
    bsec_bme_settings_t sensor_settings;    
    bsec_library_return_t bsec_status = BSEC_OK;
		uint8_t		bit_container[11]	=	{0};
		char			str_buff[25]			=	{0};
		ble_gap_conn_sec_mode_t sec_mode;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    // Initialize.
    timers_init();
    log_init();
		twi_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Blinky example started.");
    bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 0.0f, bus_write, bus_read, sleep);

    // Enter main loop.
    for (;;)
    {
        time_stamp = get_timestamp_us() * 1000;
        bsec_status = bsec_sensor_control(time_stamp, &sensor_settings);
        bme680_bsec_trigger_measurement(&sensor_settings, sleep);
        num_bsec_inputs = 0;
        bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);
        bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);
        time_stamp_interval_ms = (sensor_settings.next_call - get_timestamp_us() * 1000) / 1000000;
				
				max44009_read();											// Ergebnis in max44009_data[]
				max44009_illu			= calc_lux(max44009_data);
			
				bit_container[0]	= (uint8_t)(bme680_temp_offset >> 3);
				bit_container[1]	= (uint8_t)(((bme680_temp_offset & 0x07) << 5) | ((bme680_humi >> 5) & 0x1F));
				bit_container[2]	= (uint8_t)(((bme680_humi & 0x1F) << 3) | ((bme680_press >> 11) & 0x07));
				bit_container[3]	= (uint8_t)((bme680_press >> 3) & 0xFF);
				bit_container[4]	= (uint8_t)(((bme680_press & 0x07) << 5) | ((bme680_quali >> 4) & 0x1F));
				bit_container[5]	= (uint8_t)(((bme680_quali & 0x0F) << 4) | ((max44009_illu >> 14) & 0x0F));
				bit_container[6]	= (uint8_t)((max44009_illu >> 6) & 0xFF);
				bit_container[7]	= (uint8_t)(((max44009_illu & 0x3f) << 2) | ((feuchte >> 14) & 0x03));
				bit_container[8]	= (uint8_t)((feuchte >> 6) & 0xFF);
				bit_container[9]	= (uint8_t)(((feuchte & 0x3F) << 2) | (ble_batt & 0x03));

				sprintf(str_buff, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					bit_container[0],bit_container[1],bit_container[2],bit_container[3],bit_container[4],
					bit_container[5],bit_container[6],bit_container[7],bit_container[8],bit_container[9]);

				err_code = sd_ble_gap_device_name_set(&sec_mode, str_buff, strlen(str_buff));
				APP_ERROR_CHECK(err_code);

				err_code = ble_advdata_set(&advdata, NULL);
				APP_ERROR_CHECK(err_code);
				advertising_start();			
			
				while ( NRF_LOG_PROCESS() != false ) {}	// debug infos senden
					
        if (time_stamp_interval_ms > 0)
        {
            sleep((uint32_t)time_stamp_interval_ms);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void sleep_timer_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);
		sleep_timer_state	= 0;
}

uint32_t calc_lux(uint8_t *data)
{
		float 	lux_calc	= 0;
		uint8_t	exponent	= 0;
		uint8_t	mantissa	= 0;
	  
		exponent = (data[0] & 0xF0) >> 4;
    mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);
    lux_calc = pow(2, exponent) * mantissa * 0.045;
	
		return (uint32_t)lux_calc;
}



void max44009_read(void)
{	
	uint8_t register1	= 0x03;
	uint8_t register2 = 0x04;
	
		nrf_drv_twi_tx(&m_twi, 0x4A, &register1, 1, true);
		nrf_drv_twi_rx(&m_twi, 0x4A, &max44009_data[0], 1);
		nrf_drv_twi_tx(&m_twi, 0x4A, &register2, 1, true);
		nrf_drv_twi_rx(&m_twi, 0x4A, &max44009_data[1], 1);
}

static void application_timers_start(uint32_t time)
{
    ret_code_t err_code;
		sleep_timer_state	=	1;
    // Start application timers.
    err_code = app_timer_start(m_sleep_timer_id,  APP_TIMER_TICKS(time), NULL);
    APP_ERROR_CHECK(err_code);
}

void sleep(uint32_t t_ms)
{
	system_time	+=	t_ms;
	application_timers_start(t_ms);
	while ( sleep_timer_state == 1 )
	{	
		power_manage();	
	}
}

int64_t get_timestamp_us(void)
{
    int64_t system_current_time = 0;
		system_current_time	= (int64_t)system_time * 1000;
    return system_current_time;
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_bme680_config = {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

//		err_code = nrf_drv_twi_init(&m_twi, &twi_bme680_config, twi_handler, NULL);		// non blocking
    err_code = nrf_drv_twi_init(&m_twi, &twi_bme680_config, NULL, NULL);					// blocking
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint8_t data_len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

		rslt = nrf_drv_twi_tx(&m_twi, BME680_ADDR, &reg_addr, 1, true);
		rslt = nrf_drv_twi_rx(&m_twi, BME680_ADDR, reg_data_ptr, data_len);

    return rslt;
}

int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint8_t data_len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    uint8_t	send_tmp[200]	= {0};
		send_tmp[0] 	= reg_addr;
		memcpy(send_tmp +1, reg_data_ptr, data_len);

		rslt = nrf_drv_twi_tx(&m_twi, BME680_ADDR, send_tmp, data_len +1, false);
	
    return rslt;
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status)
{
	int16_t	temp = (uint16_t)(temperature * 10);
		if ( temp <= -500 )	{ temp = -500;}		
		if ( temp >= 700 )	{ temp = 700;}		
		bme680_temp_offset	= (uint16_t)(temp + 500);						// offset um ins positive zu kommen
		bme680_press				= (uint32_t)(pressure / 10);
		bme680_humi					= (uint32_t)(humidity * 10);
		bme680_quali				= (uint16_t)iaq;
		
		NRF_LOG_INFO("************************************");
		NRF_LOG_INFO("bme680_temp_offset:%d",bme680_temp_offset);
		NRF_LOG_INFO("bme680_press:%d",bme680_press);
		NRF_LOG_INFO("bme680_humi:%d",bme680_humi);
		NRF_LOG_INFO("bme680_quali:%d",bme680_quali);
		NRF_LOG_INFO("************************************");
		NRF_LOG_INFO("timestamp:%u",timestamp);
		NRF_LOG_INFO("iaq:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(iaq));
		NRF_LOG_INFO("iaq_accuracy:%d",iaq_accuracy);
		NRF_LOG_INFO("temperature:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(temperature));
		NRF_LOG_INFO("humidity:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(humidity));
		NRF_LOG_INFO("pressure:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(pressure));
		NRF_LOG_INFO("raw_temperature:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(raw_temperature));
		NRF_LOG_INFO("raw_humidity:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(raw_humidity));
		NRF_LOG_INFO("gas:"NRF_LOG_FLOAT_MARKER"", NRF_LOG_FLOAT(gas));
		NRF_LOG_INFO("bsec_status:%d",bsec_status);
		NRF_LOG_INFO("************************************");
}
