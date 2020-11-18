/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "string.h"

#include "nrf_delay.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

//中文显示蓝牙名称，编码为           URL编码
uint8_t DEVICE_NAME[]={0xe8,0xa5,0xbf,//西
                       0xe5,0x8c,0xbb,//医
                       0xe4,0xbb,0xbf,//仿
                       0xe7,0x9c,0x9f,//真
                       0x53,0x50,//SP
                       0x2d,0x30,0x30,0x30,0x30,//-0000
                       0x00};
//#define DEVICE_NAME                     "TCM-Acupuncture-0001"                      /**< Name of device. Will be included in the advertising data. */
//厂商名称
#define MANUFACTURER_NAME               "TCML of Tellyes Scientific Inc."           /**< Manufacturer. Will be passed to Device Information Service. */
//设备名称
#define MODEL_NUM_NAME                  "Western_Medicine_Simulation_SP"
//硬件版本
#define HW_REV                          "1.0"

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
//设置为0，表示始终广播
#define APP_ADV_DURATION                0//18000                                    /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                     /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */                   

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(12, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(16, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(800, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                     /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                2048                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                2048                                         /**< UART RX buffer size. */

static uint8_t position_data = 0;
static uint8_t Recev_Buffer[512];
static uint16_t Buffer_Offset = 0;

BLE_BAS_DEF(m_bas);
BLE_NUS_DEF(version_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_NUS_DEF(sound_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_USER_VERSION_SERVICE,       BLE_UUID_TYPE_BLE},
    {BLE_UUID_USER_SOUND_SERVICE,         BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


void ioset_connected(void);
void ioset_disconnected(void);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
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


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
//电池电量服务更新函数
uint8_t  battery_level = 0x00;
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t    level;

    level = battery_level; 

    err_code = ble_bas_battery_level_update(&m_bas, level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
//16进制数转    ASCII码
void HexToAscii(uint8_t * pHex, unsigned char * pAscii, int nLen)
{
    unsigned char Nibble[2];

    for (int i = 0; i < nLen; i++)
    {
        Nibble[0] = (pHex[i] & 0xF0) >> 4;
        Nibble[1] = pHex[i] & 0x0F;
        for (int j = 0; j < 2; j++)
        {
            if (Nibble[j] < 10)
                Nibble[j] += 0x30;
            else
            {
                if (Nibble[j] < 16)
                    Nibble[j] = Nibble[j] - 10 + 'A';
            }
            *pAscii++ = Nibble[j];
        }
    }
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_addr_t mac_addr;
    
    unsigned char res[12];

    sd_ble_gap_addr_get(&mac_addr);
    HexToAscii(&mac_addr.addr,res,strlen(mac_addr.addr));
    NRF_LOG_INFO("MAC:%x-%x-%x-%x-%x-%x",mac_addr.addr[0],mac_addr.addr[1],mac_addr.addr[2],mac_addr.addr[3],mac_addr.addr[4],mac_addr.addr[5]);
 //   NRF_LOG_INFO("MAC2:%c%c%c%c%c%c\n",res[0],res[1],res[2],res[3],res[4],res[5]);
 //   NRF_LOG_INFO("MAC3:%s\n",res);
    uint8_t send_data[]={"MacAddr:123456789abc\r\n"};
    for(int i=0;i<12;i+=2)
    {
      send_data[8+i] = res[11-i-1];
      send_data[8+i+1] = res[11-i]; 
    }

    for (uint32_t i = 0; i < sizeof(send_data); i++)
    {
        app_uart_put(send_data[i]);
    }

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    DEVICE_NAME[15]=res[2];
    DEVICE_NAME[16]=res[3];
    DEVICE_NAME[17]=res[0];
    DEVICE_NAME[18]=res[1];
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
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


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    
        //UpdateVersion
        if((p_evt->params.rx_data.p_data[0]=='U')&&(p_evt->params.rx_data.p_data[1]=='p'))
        {
            uint8_t send_data[]={"UpdateVersion\r\n"};

            for (uint32_t i = 0; i < sizeof(send_data); i++)
            {
                app_uart_put(send_data[i]);
            }
        }
        //SoundName
        else if((p_evt->params.rx_data.p_data[0]=='S')&&(p_evt->params.rx_data.p_data[1]=='o'))
        {
            uint8_t send_data[64]={"SoundName:"};
            uint8_t i=0;

            while(i<64)
            {
                send_data[10+i] = p_evt->params.rx_data.p_data[10+i];
                if(p_evt->params.rx_data.p_data[10+i]=='\n') 
                {
                  break;
                }
                else
                {
                  i++;
                }
            }

            for (uint8_t j = 0; j < (i+11); j++)
            {
                app_uart_put(send_data[j]);
            }
        }   
    }
}


/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Command NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_version_init(&version_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Sensor NUS.
    err_code = ble_nus_sound_init(&sound_nus, &nus_init);
    APP_ERROR_CHECK(err_code);


    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 0;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUM_NAME);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HW_REV);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, &DEVICE_NAME[15]);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
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
    uint32_t               err_code;
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


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    uint8_t BTS[]="BTe\r\n";//e为可选状态
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            ioset_connected();
            BTS[2]='C';
            for(int i=0;i<sizeof(BTS);i++)
            {
              app_uart_put(BTS[i]);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            ioset_disconnected();
            BTS[2]='D';
            for(int i=0;i<sizeof(BTS);i++)
            {
              app_uart_put(BTS[i]);
            }
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("DLE update request.");
            ble_gap_data_length_params_t dle_param;
            //0 means auto select DLE
            memset(&dle_param, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dle_param, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

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

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if((data_array[index - 1] == 0x0A) && (data_array[index - 2] == 0x0D))
            {
                uint16_t length = index;
                memcpy(&Recev_Buffer[Buffer_Offset],&data_array[0],index);
                
                if((Recev_Buffer[0]=='V')&&(Recev_Buffer[1]=='e'))
                {
                    ble_nus_version_send(&version_nus, Recev_Buffer, &length, m_conn_handle);
                }
                else if((Recev_Buffer[0]=='P')&&(Recev_Buffer[1]=='o'))
                {
                    ble_nus_position_send(&sound_nus, Recev_Buffer, &length, m_conn_handle);
                }
                
                memset(Recev_Buffer,0,sizeof(Recev_Buffer));
                Buffer_Offset = 0;
                index = 0;
            }
            break;

        case APP_UART_FIFO_ERROR:
            app_uart_flush();
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t                   err_code;
    ble_advertising_init_t     init;
    ble_advdata_manuf_data_t   manuf_data_response;
    uint8_t data_response[]    = "LLYES";
    int8_t tx_power = 4;
    
//    ble_advdata_manuf_data_t manuf_data;
//    memset(&manuf_data, 0, sizeof(manuf_data));
//    uint8_t data[]                = "TELLYES";
//    manuf_data.company_identifier = 0x0000;
//    manuf_data.data.p_data        = data;
//    manuf_data.data.size          = sizeof(data);

    memset(&init, 0, sizeof(init));

    init.advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance    = true;
    init.advdata.flags                 = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
//    init.advdata.p_manuf_specific_data = &manuf_data;
//    init.advdata.p_tx_power_level      = &tx_power;
    
    memset(&manuf_data_response, 0, sizeof(manuf_data_response));
    
    manuf_data_response.company_identifier     = 0x5445;
    manuf_data_response.data.p_data            = data_response;
    manuf_data_response.data.size              = strlen(data_response);

    init.srdata.name_type               = BLE_ADVDATA_NO_NAME;
    init.srdata.p_manuf_specific_data   = &manuf_data_response;
    init.srdata.p_tx_power_level        = &tx_power;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
    APP_ERROR_CHECK(err_code);
}


void throughput_test()
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = true;
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}


void pa_lna_init(uint32_t gpio_pa_pin, uint32_t gpio_lna_pin)
{
    ble_opt_t opt;
    uint32_t gpiote_ch = NULL;
    ret_code_t err_code;        

    memset(&opt, 0, sizeof(ble_opt_t));
    
    err_code = nrf_drv_gpiote_init();
    if(err_code != NRF_ERROR_INVALID_STATE)
        APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_init();
    if(err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED)
        APP_ERROR_CHECK(err_code);
    
    nrf_ppi_channel_t ppi_set_ch;
    nrf_ppi_channel_t ppi_clr_ch;
    
    err_code = nrf_drv_ppi_channel_alloc(&ppi_set_ch);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_alloc(&ppi_clr_ch);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
    
    if((gpio_pa_pin == NULL) && (gpio_lna_pin == NULL))
    {
        err_code = NRF_ERROR_INVALID_PARAM;
        APP_ERROR_CHECK(err_code);
    }    

    if(gpio_pa_pin != NULL)
    {
        if(gpiote_ch == NULL)
        {
            err_code = nrf_drv_gpiote_out_init(gpio_pa_pin, &config);
            APP_ERROR_CHECK(err_code);
            
            gpiote_ch = nrf_drv_gpiote_out_task_addr_get(gpio_pa_pin); 
        }
        
        // PA config
        opt.common_opt.pa_lna.pa_cfg.active_high = 1;   // Set the pin to be active high
        opt.common_opt.pa_lna.pa_cfg.enable      = 1;   // Enable toggling
        opt.common_opt.pa_lna.pa_cfg.gpio_pin    = gpio_pa_pin; // The GPIO pin to toggle tx  
    }
    
    if(gpio_lna_pin != NULL)
    {
        if(gpiote_ch == NULL)
        {
            err_code = nrf_drv_gpiote_out_init(gpio_lna_pin, &config);
            APP_ERROR_CHECK(err_code);        
            
            gpiote_ch = nrf_drv_gpiote_out_task_addr_get(gpio_lna_pin); 
        }
        
        // LNA config
        opt.common_opt.pa_lna.lna_cfg.active_high  = 1; // Set the pin to be active high
        opt.common_opt.pa_lna.lna_cfg.enable       = 1; // Enable toggling
        opt.common_opt.pa_lna.lna_cfg.gpio_pin     = gpio_lna_pin;  // The GPIO pin to toggle rx
    }

    // Common PA/LNA config
    opt.common_opt.pa_lna.gpiote_ch_id  = (gpiote_ch - NRF_GPIOTE_BASE) >> 2;   // GPIOTE channel used for radio pin toggling
    opt.common_opt.pa_lna.ppi_ch_id_clr = ppi_clr_ch;   // PPI channel used for radio pin clearing
    opt.common_opt.pa_lna.ppi_ch_id_set = ppi_set_ch;   // PPI channel used for radio pin setting
    
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &opt);
    APP_ERROR_CHECK(err_code);    
}

void connected_flag_gpio_init(void)
{
  nrf_gpio_cfg_output(21);
  nrf_gpio_pin_set(21);
  nrf_gpio_cfg_output(19);
  nrf_gpio_pin_clear(19);
}

void ioset_connected(void)
{
  nrf_gpio_pin_clear(21);
  nrf_gpio_pin_set(19);
}

void ioset_disconnected(void)
{
  nrf_gpio_pin_set(21);
  nrf_gpio_pin_clear(19);
}


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    connected_flag_gpio_init();
    nrf_delay_ms(100);
    uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    throughput_test();

    // Init GPIO's to control PA and/or LNA, must be done while radio is inactive.
    pa_lna_init(23,22);

    // Start execution.
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    application_timers_start();
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
