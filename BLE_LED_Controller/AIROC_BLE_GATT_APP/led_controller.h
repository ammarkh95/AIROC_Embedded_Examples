/** @file
*
* LE Vendor Specific Device
*
* This file provides definitions and function prototypes for LED Controller
* device
*
*/
#ifndef _LED_CONTROLLER_H_
#define _LED_CONTROLLER_H_

#include "wiced_hal_nvram.h"
#include "wiced_gki.h"

extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];
extern const wiced_transport_cfg_t transport_cfg;
void led_controller_gatt_init();
extern const uint8_t led_controller_gatt_database[];
extern size_t led_controller_gatt_database_size;
/******************************************************************************
 *                                Constants
 ******************************************************************************/
#ifndef DEV_NAME
#define DEV_NAME "BLE_LED_CONTROLLER"
#endif
#define LED_CONTROLLER_MAX_NUM_CLIENTS 1
#define led_controller_GATTS_MAX_CONN     1
#define APP_ADV_NAME wiced_bt_cfg_settings.device_name
/* LED Controller Status Update Timer  */
#define LED_CONTROLLER_STATUS_UPDATE_TIMEOUT_IN_SECONDS                 30
#define LED_CONTROLLER_VS_ID                      WICED_NVRAM_VSID_START
#define LED_CONTROLLER_LOCAL_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 1 )
#define LED_CONTROLLER_PAIRED_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 2 )
/* AIROC LED 1 to indicate connection success */
#define LED_1_GPIO (uint32_t)*platform_led[WICED_PLATFORM_LED_1].gpio
/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_HSENS_GATT_SERVICE = 0x1, // service handle

    HANDLE_HSENS_GAP_SERVICE = 0x14, // service handle
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, // characteristic handl
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL, // char value handle

        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, // characteristic handl
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,// char value handle


    HANDLE_HSENS_SERVICE = 0x28,
        HANDLE_HSENS_SERVICE_CHAR_LED_STATUS, // characteristic handl
        HANDLE_HSENS_SERVICE_CHAR_LED_STATUS_VAL, // char value handle

        HANDLE_HSENS_SERVICE_CHAR_LED_BLINK_PERIOD, // characteristic handl
        HANDLE_HSENS_SERVICE_CHAR_LED_BLINK_PERIOD_VAL, //long  char value handl......

    HANDLE_HSENS_DEV_INFO_SERVICE = 0x40,
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, // characteristic handle
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,// char value handle

        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, // characteristic handl
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,// char value handle

        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, // characteristic handl
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,// char value handle
    // Client Configuration
    HDLD_CURRENT_TIME_SERVICE_CURRENT_TIME_CLIENT_CONFIGURATION,
}led_controller_db_tags;
/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    BD_ADDR   remote_addr;              // remote peer device address
    uint16_t  conn_id;                  // connection ID referenced by the stack
    uint16_t  peer_mtu;                 // peer MTU
    uint8_t   flag_stay_connected;      // stay connected or disconnect after all messages are sent
} led_controller_state_t;

#pragma pack(1)
/* Host information saved in NVRAM */
typedef PACKED struct
{
    BD_ADDR  bdaddr;                                /* BD address of the bonded host */
    uint8_t   led_status;                     /* configuration of LED blink status */
    uint8_t   led_blink_period_sec;                     /* configuration of LED blink period in seconds */

} host_info_t;
#pragma pack()

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;
#endif // _LED_CONTROLLER_H_
