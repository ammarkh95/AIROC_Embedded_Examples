/** @file
*
* LE Vendor Specific Device
*
* During initialization the app registers with LE stack to receive various
* notifications including bonding complete, connection status change and
* peer write. When device is successfully bonded, application saves
* peer's Bluetooth Device address to the NVRAM. Bonded device can also
* write in to client configuration descriptor of the notification
* characteristic. That is also saved in the NVRAM. The app exposes LED
* control service that controls the state of LEDs attached to an I2C ESP32
* slave device. When the characterstic is set to 1, the CYW20835
* (as I2C master) will trigger LED_ON I2C command to the ESP32 to turn ON
* the LEDs, the opposite will happen when the characterstic recives the value zero
*
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - NVRAM read/write operation
*  - Sending data to the client
*  - Processing write requests from the client
*  - using I2C peripheral 
*/
#include <string.h>
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_i2c.h"
#include "wiced_result.h"
#include "wiced_bt_stack.h"
#include "wiced_transport.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "cycfg_pins.h"
#include "led_controller_uuid.h"
#include "led_controller.h"
#include "led_i2c_slave.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/*
 * This is the GATT database for the LED Controller application.  It defines
 * services, characteristics and descriptors supported. Each attribute in the database has a handle,
 * (characteristic has two, one for characteristic itself, another for the value).
 * The handles are used by the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it. All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t led_controller_gatt_database[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    // Declare mandatory GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

    // Declare mandatory GAP service characteristic: Dev Name
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Declare mandatory GAP service characteristic: Appearance
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Declare proprietary LED Service with 128 byte UUID
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_LED_SERVICE ),

    // Declare characteristic LED Configuration
    // The configuration consists of 1 byte which indicates target state of the LED
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_LED_STATUS, HANDLE_HSENS_SERVICE_CHAR_LED_STATUS_VAL,
            UUID_LED_CHARACTERISTIC_ON_OFF_CONFIG, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ ),

    // The configuration consists of 1 byte which indicates blink period of the LED in seconds
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_LED_BLINK_PERIOD, HANDLE_HSENS_SERVICE_CHAR_LED_BLINK_PERIOD_VAL,
            UUID_LED_CHARACTERISTIC_BLINK_PERIOD_CONFIG, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

    // Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
            UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Handle 0x50: characteristic Model Number, handle 0x51 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
            UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Handle 0x52: characteristic System ID, handle 0x53 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
            UUID_CHARACTERISTIC_SYSTEM_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),
};
size_t led_controller_gatt_database_size = sizeof(led_controller_gatt_database);
uint8_t led_controller_device_name[]          = DEV_NAME;                                          //GAP Service characteristic Device Name
uint8_t led_controller_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    led_controller_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    led_controller_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t led_controller_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};
uint8_t write_buffer[2] = {0};
wiced_bt_gpio_numbers_t led_pin;

/* Holds the global state of the led_controller application */
led_controller_state_t led_controller_state;
/* Holds the host info saved in the NVRAM */
host_info_t led_controller_hostinfo;

/* Attribute list of the LED Controller */
attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( led_controller_device_name ),         led_controller_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(led_controller_appearance_name),       led_controller_appearance_name },
    { HANDLE_HSENS_SERVICE_CHAR_LED_STATUS_VAL,              1,                                          &led_controller_hostinfo.led_status },
    { HANDLE_HSENS_SERVICE_CHAR_LED_BLINK_PERIOD_VAL,              1,                                          &led_controller_hostinfo.led_blink_period_sec},
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(led_controller_char_mfr_name_value),   led_controller_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(led_controller_char_model_num_value),  led_controller_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(led_controller_char_system_id_value),  led_controller_char_system_id_value },
};
static wiced_result_t           led_controller_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_gatt_status_t          led_controller_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                     led_controller_set_advertisement_data(void);
static void                     led_controller_smp_bond_result( uint8_t result );
static void                     led_controller_encryption_changed( wiced_result_t result, uint8_t* bd_addr );
static void                     led_controller_application_init( void );
static void                     led_controller_load_keys_for_address_resolution( void );
void                            led_controller_set_led_state(uint8_t led_state );
void                            led_controller_set_led_blink_period(uint8_t led_blink_period_sec);
/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that Bluetooth device is ready.
 */
APPLICATION_START( )
{
    WICED_BT_TRACE("BLE LED Controller Start\n" );
    // Assign AIROC LED 1 GPIO 
    led_pin = LED_1_GPIO;
    wiced_transport_init( &transport_cfg );
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_bt_stack_init(led_controller_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
    // start I2C on dedicated SCL, SDA pins
    WICED_BT_TRACE("Init I2C\n");
    wiced_hal_i2c_init();
    wiced_hal_i2c_select_pads(I2C_SCL, I2C_SDA);

    // read last stored controller info from NVRAM
    wiced_result_t result;
    wiced_hal_read_nvram(LED_CONTROLLER_VS_ID, sizeof(led_controller_hostinfo), (uint8_t*)&led_controller_hostinfo, &result);
    // set initial state, blink period of the LED based on stored data from NVRAM
    led_controller_set_led_state(led_controller_hostinfo.led_status);
    led_controller_set_led_blink_period(led_controller_hostinfo.led_blink_period_sec);
}
/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void led_controller_application_init( void )
{

    WICED_BT_TRACE("LED Controller GATT/ADV Init\n" );

    // init gatt
    led_controller_gatt_init();

    /* Load previous paired keys for address resolution */
    led_controller_load_keys_for_address_resolution();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    led_controller_set_advertisement_data();
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /*
     * Set flag_stay_connected to remain connected
     * Reset flag to 0, to disconnect
     */
    led_controller_state.flag_stay_connected = 1;
}
/**
 * Set the LED status by making I2C write to ESP32
 */
void led_controller_set_led_state(uint8_t led_state)
{
    if(led_state == 0){
        write_buffer[0] = LED_OFF_COMMAND;
        WICED_BT_TRACE("Send LED OFF Command: %0x\n", LED_OFF_COMMAND);
        wiced_hal_i2c_write((uint8_t*)&write_buffer, sizeof(write_buffer), ESP32_I2C_ADDRESS);
    }
    else if (led_state == 1)
    {
        write_buffer[0] = LED_ON_COMMAND;
        WICED_BT_TRACE("Send LED ON Command: %0x\n", LED_ON_COMMAND);
        wiced_hal_i2c_write((uint8_t*)&write_buffer, sizeof(write_buffer), ESP32_I2C_ADDRESS);
    }
    else
    {
        WICED_BT_TRACE("INVALID LED STATE: %d\n", led_state);
    }
}
/**
 * Set the LED blink period by making I2C write to ESP32
 */
void led_controller_set_led_blink_period(uint8_t led_blink_period_sec)
{
    if(led_blink_period_sec != 0)
    {
        write_buffer[0] = LED_SET_BLINK_PERIOD_COMMAND;
        write_buffer[1] = led_blink_period_sec;
        WICED_BT_TRACE("Send LED SET BLINK PERIOD Command: %0x\n", LED_SET_BLINK_PERIOD_COMMAND);
        wiced_hal_i2c_write((uint8_t*)&write_buffer, sizeof(write_buffer), ESP32_I2C_ADDRESS);
    }
    else
    {
        WICED_BT_TRACE("INVALID LED BLINK PERIOD: %d\n", led_blink_period_sec);
    }
}
/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void led_controller_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t led_service_uuid[LEN_UUID_128] = { UUID_LED_SERVICE };

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = led_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (const char *) APP_ADV_NAME);
    adv_elem[num_elem].p_data       = (uint8_t *)APP_ADV_NAME;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}
/*
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */
void led_controller_advertisement_stopped( void )
{
    wiced_result_t result;

    if ( led_controller_state.flag_stay_connected && !led_controller_state.conn_id )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", result );
    }
    else
    {
        WICED_BT_TRACE( "ADV stop\n");
    }

    UNUSED_VARIABLE(result);
}
/*
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */
void led_controller_smp_bond_result( uint8_t result )
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    WICED_BT_TRACE( "led_controller, bond result: %d\n", result );

    /* Bonding success */
    if ( result == WICED_BT_SUCCESS )
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( led_controller_hostinfo.bdaddr, led_controller_state.remote_addr, sizeof( BD_ADDR ) );

        /* Write to NVRAM */
        /* using single VS_ID to support storing only one paired device */
        /* see README for multiple concurrent pairing support */
        written_byte = wiced_hal_write_nvram(LED_CONTROLLER_VS_ID, sizeof(led_controller_hostinfo), (uint8_t*)&led_controller_hostinfo, &status );
        WICED_BT_TRACE("NVRAM write: %d\n", written_byte);
    }

    UNUSED_VARIABLE(written_byte);
}
/*
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 */
void led_controller_encryption_changed( wiced_result_t result, uint8_t* bd_addr )
{
    WICED_BT_TRACE( "encryp change bd ( %B ) res: %d ", led_controller_hostinfo.bdaddr,  result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
    /* using single VS_ID to support storing only one paired device */
    /* see README for multiple concurrent pairing support */
    wiced_hal_read_nvram(LED_CONTROLLER_VS_ID, sizeof(led_controller_hostinfo), (uint8_t*)&led_controller_hostinfo, &result );
}
/*
 * led_controller device and link management callback
 */
wiced_result_t led_controller_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_encryption_status_t    *p_status;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t          *p_mode;
    wiced_bt_device_address_t           id_addr;
    uint8_t                             *p_keys;
    wiced_result_t                      result = WICED_BT_SUCCESS;

    WICED_BT_TRACE("led_controller_management_cback: 0x%x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            led_controller_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE( "Pairing Complete: %d ", p_info->reason);
            led_controller_smp_bond_result( p_info->reason );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            /* using single VS_ID to support storing only one paired device */
            /* see README for multiple concurrent pairing support */
            p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
            wiced_hal_write_nvram (LED_CONTROLLER_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            {
                wiced_bt_device_link_keys_t link_keys;
                p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
                /* using single VS_ID to support storing only one paired device */
                /* see README for multiple concurrent pairing support */
                wiced_hal_read_nvram( LED_CONTROLLER_PAIRED_KEYS_VS_ID,
                                      sizeof(wiced_bt_device_link_keys_t),
                                      (uint8_t *)&link_keys,
                                      &result );

                WICED_BT_TRACE("keys read from NVRAM %B req BDA %B result: %d \n", link_keys.bd_addr, p_event_data->paired_device_link_keys_request.bd_addr, result);

                // Break if link key retrival is failed or link key is not available.
                if (result != WICED_BT_SUCCESS)
                    break;

                // Compare the BDA
                if( memcmp(&(link_keys.bd_addr), &(p_event_data->paired_device_link_keys_request.bd_addr), sizeof(wiced_bt_device_address_t) ) == 0 )
                {
                    memcpy(p_keys, (uint8_t *)&link_keys, sizeof(wiced_bt_device_link_keys_t));
                }
                else
                {
                    result = WICED_BT_ERROR;
                }
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            /* using single VS_ID to support storing only one paired device */
            /* see README for multiple concurrent pairing support */
            p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram (LED_CONTROLLER_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            /* using single VS_ID to support storing only one paired device */
            /* see README for multiple concurrent pairing support */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram(LED_CONTROLLER_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d", p_status->bd_addr, p_status->result);
            led_controller_encryption_changed( p_status->result, p_status->bd_addr );
        break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
            if ( *p_mode == BTM_BLE_ADVERT_OFF )
            {
                led_controller_advertisement_stopped();
            }
            break;

    default:
            break;
    }

    return result;
}
/*
 * Find attribute description by handle
 */
attribute_t * led_controller_get_attribute( uint16_t handle )
{
    uint32_t i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}
/* This function is invoked when connection is established */
wiced_bt_gatt_status_t led_controller_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "led_controller_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id);
    wiced_hal_gpio_set_pin_output(led_pin, GPIO_PIN_OUTPUT_LOW);

    /* Update the connection handler.  Save address of the connected device. */
    led_controller_state.conn_id = p_status->conn_id;
    memcpy(led_controller_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Saving host info in NVRAM  */
    memcpy( led_controller_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    {
        uint8_t bytes_written = 0;
        /* using single VS_ID to support storing only one paired device */
        /* see README for multiple concurrent pairing support */
        bytes_written = wiced_hal_write_nvram(LED_CONTROLLER_VS_ID, sizeof(led_controller_hostinfo), (uint8_t*)&led_controller_hostinfo, &result );

        WICED_BT_TRACE("NVRAM write %d\n", bytes_written);
        UNUSED_VARIABLE(bytes_written);
    }

    return WICED_BT_GATT_SUCCESS;
}
/*
 * This function is invoked when connection is lost
 */
wiced_bt_gatt_status_t led_controller_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n", led_controller_state.remote_addr, p_status->conn_id, p_status->reason );
    wiced_hal_gpio_set_pin_output(led_pin, GPIO_PIN_OUTPUT_HIGH);

    /* Resetting the device info */
    memset( led_controller_state.remote_addr, 0, 6 );
    led_controller_state.conn_id = 0;

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if ( led_controller_state.flag_stay_connected )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );
    }

    UNUSED_VARIABLE(result);

    return WICED_BT_SUCCESS;
}
/*
 * Connection up/down event
 */
wiced_bt_gatt_status_t led_controller_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return led_controller_gatts_connection_up( p_status );
    }

    return led_controller_gatts_connection_down( p_status );
}

static void led_controller_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    /* using single VS_ID to support storing only one paired device */
    /* see README for multiple concurrent pairing support */
    wiced_hal_read_nvram(LED_CONTROLLER_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if(result == WICED_BT_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
    }
    WICED_BT_TRACE("led_controller_load_keys_for_address_resolution %B result:%d \n", p, result );
}
