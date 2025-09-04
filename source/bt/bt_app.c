/*******************************************************************************
* File Name: bt_app.c
*
* Description: This file contains the task that handles bluetooth events and
* notifications.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header file includes
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "cycfg_gap.h"
#include "cybsp_bt_config.h"
#include "cycfg_gatt_db.h"
#include "cycfg_bt_settings.h"
#include "wiced_bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_ble.h"
#include "bt_app.h"
#include "board.h"
#include "capsense.h"
#include "FreeRTOS.h"
#include "task.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* HID Consumer Control bitmask in Report ID 1 (1 byte) */
#define CC_PLAY_PAUSE   (1u << 0)
#define CC_MUTE         (1u << 1)
#define CC_VOL_UP       (1u << 2)
#define CC_VOL_DOWN     (1u << 3)

/* Vervolgradatie: hoeveel slider-% verschil is 1 volumestap? */
#define VOL_STEP_PERCENT   (2u)   /* 2% ≈ zachte, vloeiende stappen */

static cyhal_pwm_t pwm_led;

// andere namen dan die in board.h
typedef enum { LEDM_OFF, LEDM_PULSE, LEDM_ON } ledm_mode_t;

static void led_set(ledm_mode_t m)
{
    switch (m)
    {
        case LEDM_OFF:   cyhal_pwm_set_duty_cycle(&pwm_led, 0.0f, 2.0f);  break;
        case LEDM_PULSE: cyhal_pwm_set_duty_cycle(&pwm_led, 50.0f, 1.0f); break; // 1 Hz
        case LEDM_ON:    cyhal_pwm_set_duty_cycle(&pwm_led, 100.0f, 1.0f);break;
    }
}

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Holds the connection ID */
volatile uint16_t bt_connection_id = 0;
/**
 * Typdef for function used to free allocated buffer to stack
 */
typedef void (*pfn_free_buffer_t)(uint8_t *);

// Extern API voor capsense.c
void bt_hid_play_pause(void);
void bt_hid_mute(void);
void bt_hid_volume_up(void);
void bt_hid_volume_down(void);


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_event_cb(wiced_bt_gatt_evt_t event,
                                            wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t bt_app_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t 
                                                                    *p_conn_status);
wiced_bt_gatt_status_t bt_app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req);
wiced_bt_gatt_status_t bt_app_gatt_req_write_value(uint16_t attr_handle,
                                                    uint8_t *p_val, uint16_t len);
wiced_bt_gatt_status_t bt_app_gatt_req_write_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *p_write_req,
                                                uint16_t len_req);
wiced_bt_gatt_status_t bt_app_gatt_req_read_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_t *p_read_req,
                                                uint16_t len_req);
wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler (uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_by_type_t *p_read_req,
                                                uint16_t len_requested);
static gatt_db_lookup_table_t *bt_app_find_by_handle(uint16_t handle);
static void  bt_app_init(void);
static void* bt_app_alloc_buffer(int len);
static void  bt_app_free_buffer(uint8_t *p_event_data);
static void  bt_print_bd_address(wiced_bt_device_address_t bdadr);

// ======= HID Consumer Control helpers (publiek gebruikt door capsense.c) =======
/*void bt_hid_send_cc_mask(uint8_t mask)
{
    if (bt_connection_id == 0) return;            // niet verbonden → niks doen

    printf("HID report: 0x%02X\r\n", mask);

    // 1-byte input report (Report ID=1 volgens jouw Report Map)
    app_hids_report[0] = mask;
    wiced_bt_gatt_server_send_notification(bt_connection_id,
                                           HDLC_HIDS_REPORT_VALUE,
                                           app_hids_report_len,
                                           app_hids_report,
                                           NULL);

    // "release" sturen
    app_hids_report[0] = 0x00;
    wiced_bt_gatt_server_send_notification(bt_connection_id,
                                           HDLC_HIDS_REPORT_VALUE,
                                           app_hids_report_len,
                                           app_hids_report,
                                           NULL);
}

// Makkelijkere wrappers
void bt_hid_play_pause(void)   { bt_hid_send_cc_mask(0b0001); } // bit0
void bt_hid_mute(void)         { bt_hid_send_cc_mask(0b0010); } // bit1
void bt_hid_volume_up(void)    { bt_hid_send_cc_mask(0b0100); } // bit2
void bt_hid_volume_down(void)  { bt_hid_send_cc_mask(0b1000); } // bit3*/

static inline void hid_notify_press_release(uint8_t mask)
{
    if (bt_connection_id == 0) return;

    app_hids_report[0] = mask;
    wiced_result_t r1 = wiced_bt_gatt_server_send_notification(
            bt_connection_id, HDLC_HIDS_REPORT_VALUE,
            app_hids_report_len, app_hids_report, NULL);
    printf("HID report: 0x%02X (send r=0x%02X)\r\n", mask, r1);

    /* heel kort wachten zodat host de “press” ziet */
    vTaskDelay(pdMS_TO_TICKS(10));  // 10 ms is doorgaans genoeg

    app_hids_report[0] = 0x00;
    wiced_result_t r2 = wiced_bt_gatt_server_send_notification(
            bt_connection_id, HDLC_HIDS_REPORT_VALUE,
            app_hids_report_len, app_hids_report, NULL);
    printf("HID report: 0x00 (release r=0x%02X)\r\n", r2);
}

// wrappers die je vanuit capsense.c aanroept
void bt_hid_play_pause(void)   { hid_notify_press_release(0b0001); } // bit0
void bt_hid_mute(void)         { hid_notify_press_release(0b0010); } // bit1
void bt_hid_volume_up(void)    { hid_notify_press_release(0b0100); } // bit2
void bt_hid_volume_down(void)  { hid_notify_press_release(0b1000); } // bit3

/* Externs uit cycfg_gatt_db.c voor jouw HID Report & CCCD */
extern uint8_t app_hids_report[];                       /* opslag voor Input Report (1 byte) */
extern const uint16_t app_hids_report_len;
extern uint8_t app_hids_report_client_char_config[];    /* CCCD buffer (2 bytes) */

static inline bool hid_cccd_enabled(void)
{
    /* bit0 = notifications enabled */
    return (app_hids_report_client_char_config[0] & 0x01) != 0;
}

static inline void hid_notify(uint8_t val)
{
    if (bt_connection_id == 0) return;
    // 1-byte input report (Report ID = 1 in je Report Map)
    app_hids_report[0] = val;           // bit0=Play/Pause, bit1=Mute, bit2=Vol+, bit3=Vol-
    wiced_bt_gatt_server_send_notification(bt_connection_id,
                                           HDLC_HIDS_REPORT_VALUE,
                                           app_hids_report_len,
                                           app_hids_report,
                                           NULL);
}

/* (optioneel) laatste verstuurde report byte bijhouden voor READ-requests */
static uint8_t g_hid_last_report = 0;

static void hid_send_cc_report(uint8_t bits)
{
    g_hid_last_report = bits;
    if (bt_connection_id && hid_cccd_enabled())
    {
        wiced_bt_gatt_server_send_notification(bt_connection_id,
                                               HDLC_HIDS_REPORT_VALUE, /* jouw value handle */
                                               1, &bits, NULL);
    }
}

/* Maak een klein ADV-pakket met Flags + Appearance + Complete Local Name */
static wiced_result_t set_minimal_adv_flags_only(void)
{
    uint8_t adv[3 + 2];  // genoeg
    uint8_t a = 0;

    /* Flags AD structure: len=2, type=0x01, data=0x06 (General Disc + BR/EDR not supported) */
    adv[a++] = 2;     // length (type + 1 byte data)
    adv[a++] = 0x01;  // AD type: Flags
    adv[a++] = 0x06;  // data

    /* Stop ADV, zet data, start later opnieuw */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    wiced_result_t r1 = wiced_bt_ble_set_raw_advertisement_data(a, adv);
    printf("Set RAW ADV (flags) len=%u -> 0x%04X\r\n", a, r1);

    /* Lege scan response – geef len=0 en pointer = NULL (sommige stacks eisen NULL) */
    wiced_result_t r2 = wiced_bt_ble_set_raw_scan_response_data(0, NULL);
    printf("Clear RAW SCAN -> 0x%04X\r\n", r2);

    return r1;
}


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: bt_task
********************************************************************************
* Summary:
*  Task that handles Bluetooth initialization and updates GATT notification data.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void bt_task(void* param)
{
    static uint32_t nofify_value;
    /* Suppress warning for unused parameter */
    (void)param;

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block till a notification is received. */
        xTaskNotifyWait(0, 0, &nofify_value, portMAX_DELAY);

       /* Command has been received from queue */
        if(nofify_value == NOTIFIY_ON)
        {
            bt_app_send_notification();
        }

        vTaskDelay(10);

    }
}


/*******************************************************************************
* Function Name: bt_app_init
********************************************************************************
* Summary:
*   This function handles application level initialization tasks and is 
*   called from the BT management callback once the LE stack enabled event 
*   (BTM_ENABLED_EVT) is triggered This function is executed in the
*    BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_init(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    printf("Discover the device with name: \"%s\"\r\n", app_gap_device_name);
    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(bt_app_gatt_event_cb);
    printf("GATT event handler registration status: %d \r\n",status);

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %d \r\n",status);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(FALSE, FALSE);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, 
                                                            cy_bt_adv_packet_data);

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != result)
    {
        printf("Failed to start advertisement! \r\n");
        CY_ASSERT(0u);
    }

    // LED init
    if (CY_RSLT_SUCCESS == cyhal_pwm_init(&pwm_led, CYBSP_USER_LED, NULL))
    {
        cyhal_pwm_set_duty_cycle(&pwm_led, 0.0f, 2.0f);
        cyhal_pwm_start(&pwm_led);
    }
}


/*******************************************************************************
* Function Name: bt_app_management_cb
********************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events
*   from the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event 
*                                                 structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t bt_app_management_cb(wiced_bt_management_evt_t event,
                                   wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t local_bda = {0};

    printf("Bluetooth app management callback: 0x%x\r\n", event);

    switch (event)
    {
    	//COMMENTED FOR TESTING PURPOSES

        /*case BTM_ENABLED_EVT:
            // Bluetooth Controller and Host Stack Enabled
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr(cy_bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(local_bda);
                printf("Bluetooth local device address: ");
                bt_print_bd_address(local_bda);

                // Perform application-specific initialization
                bt_app_init();
            }
            else
            {
                printf("Bluetooth enable failed, status = %d \r\n", p_event_data->enabled.status);
            }
            break;*/

		case BTM_ENABLED_EVT:
			if (WICED_BT_SUCCESS == p_event_data->enabled.status)
			{
				wiced_bt_set_local_bdaddr(cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_device_address_t local_bda = {0};
				wiced_bt_dev_read_local_addr(local_bda);
				printf("Bluetooth local device address: ");
				bt_print_bd_address(local_bda);

				bt_app_init();  // PWM/LED init

				// Pairable + discoverable + connectable
				wiced_bt_set_pairable_mode(WICED_TRUE, WICED_TRUE);
				wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, 0, 0);
				wiced_bt_dev_set_connectability(BTM_CONNECTABLE, 0, 0);

				// Start advertising volgens configurator
				wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
				led_set(LEDM_PULSE);
			}
			else
			{
				printf("Bluetooth enable failed, status = %d \r\n", p_event_data->enabled.status);
			}
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
			printf("Bluetooth advertisement state change: 0x%x\r\n",
				   p_event_data->ble_advert_state_changed);
			if (p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF)
				led_set(LEDM_OFF);
			else
				led_set(LEDM_PULSE);
			break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        {
            /* Forceer Just Works (geen pincode), wel bond + encryptie */
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data     = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req     = BTM_LE_AUTH_REQ_BOND;  // bonden, geen MITM (Just Works)
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys    =
                    BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys    =
                    BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK |
                    BTM_LE_KEY_LENC | BTM_LE_KEY_LCSRK;

            result = WICED_BT_SUCCESS;
            break;
        }

        case BTM_SECURITY_REQUEST_EVT:
        {
            // Peer vraagt om beveiliging; sta Just Works toe
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                        WICED_BT_SUCCESS);
            printf("SECURITY REQUEST -> grant sent\r\n");

            result = WICED_BT_SUCCESS;
            break;
        }

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Bluetooth connection parameter update status:%d\r\n"
                    "parameter interval: %d ms\r\n"
                    "parameter latency: %d ms\r\n"
                    "parameter timeout: %d ms\r\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    p_event_data->ble_connection_param_update.supervision_timeout);
            result = WICED_SUCCESS;
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            /* Print the updated BLE physical link*/
            printf("Bluetooth phy update selected TX - %dM\r\n"
                    "Bluetooth phy update selected RX - %dM\r\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

        /*case BTM_PIN_REQUEST_EVT:
            // We ondersteunen geen manuele PIN; antwoord “ok” met 0 cijfers.
            wiced_bt_dev_pin_code_reply(p_event_data->pin_request.bd_addr,
                                        WICED_BT_SUCCESS, 0, NULL);
            result = WICED_BT_SUCCESS;
            break;

        case BTM_PASSKEY_REQUEST_EVT:
            // Just Works: geen passkey – maak het expliciet success zonder key.
            wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_request.bd_addr, 0);
            result = WICED_BT_SUCCESS;
            break;*/

        case BTM_ENCRYPTION_STATUS_EVT:
            printf("Encrypt status: res=0x%02X\r\n",
                   p_event_data->encryption_status.result);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            printf("Pairing complete: res=0x%02X, reason=0x%02X\r\n",
                   p_event_data->pairing_complete.pairing_complete_info.ble.status,
                   p_event_data->pairing_complete.pairing_complete_info.ble.reason);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        {
            wiced_bt_device_link_keys_t *keys =
                &p_event_data->paired_device_link_keys_update;
            wiced_result_t r =
                wiced_bt_dev_add_device_to_address_resolution_db(keys);
            printf("Keys UPDATE -> 0x%02X\r\n", r);
            break;
        }

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            // Gebruik je geen eigen storage? Log alleen; stack hanteert z’n cache.
            printf("Keys REQUEST\r\n");
            break;

        default:
            break;
    }
    return result;
}


/*******************************************************************************
* Function Name: bt_app_gatt_event_cb
********************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                : LE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_event_cb(wiced_bt_gatt_evt_t event,
                                        wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    /* Call the appropriate callback function based on the GATT event type, and 
     * pass the relevant event parameters to the callback function */
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_event_data->connection_status.connected)
        {
            printf("GATT connected: conn_id:%d\r\n",
                   p_event_data->connection_status.conn_id);

            bt_connection_id = p_event_data->connection_status.conn_id; // <-- BELANGRIJK
            led_set(LEDM_ON);
        }
        else
        {
            printf("GATT disconnected: reason:0x%02X\r\n",
                   p_event_data->connection_status.reason);

            bt_connection_id = 0;                                        // <-- clear bij disconnect
            led_set(LEDM_PULSE);
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        }
        status = WICED_BT_GATT_SUCCESS;
        break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            status = bt_app_gatt_req_cb(p_attr_req);
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            bt_app_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)bt_app_free_buffer;
            status = WICED_BT_GATT_SUCCESS;
            break;

            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free =
                (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            status = WICED_BT_GATT_SUCCESS;
        }
            break;

        default:
            status = WICED_BT_GATT_SUCCESS;
            break;
    }

    return status;
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_cb
********************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t p_attr_req : Pointer to GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            status = bt_app_gatt_req_read_handler(p_attr_req->conn_id,
                                                  p_attr_req->opcode,
                                                  &p_attr_req->data.read_req,
                                                  p_attr_req->len_requested);
             break;

        case GATT_REQ_READ_BY_TYPE:
            status = bt_app_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                            p_attr_req->opcode,
                                                          &p_attr_req->data.read_by_type,
                                                          p_attr_req->len_requested);
            break;

        case GATT_REQ_READ_MULTI:
            break;

        case GATT_REQ_MTU:
            status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                       p_attr_req->data.remote_mtu,
                                                       CY_BT_MTU_SIZE);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             status = bt_app_gatt_req_write_handler(p_attr_req->conn_id,
                                                    p_attr_req->opcode,
                                                    &p_attr_req->data.write_req,
                                                    p_attr_req->len_requested);

             if ((GATT_REQ_WRITE == p_attr_req->opcode) &&
                 (WICED_BT_GATT_SUCCESS == status ))
             {
                 wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->data.write_req;
                 wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                                                     p_write_request->handle);
             }
             break;
        case GATT_HANDLE_VALUE_CONF:
        case GATT_HANDLE_VALUE_NOTIF:
             break;

        case GATT_HANDLE_VALUE_IND:
            printf("bt_app_gatt:ind\r\n");
            break;
        default:
            printf("bt_app_gatt: unhandled GATT request: %d\r\n", p_attr_req->opcode);
            break;
    }

    return status;
}


/*******************************************************************************
 * Function Name : bt_app_gatt_req_read_by_type_handler
 * *****************************************************************************
 * Summary :
 *    Process read-by-type request from peer device
 *
 * Parameters:
 *  uint16_t                      conn_id       : Connection ID
 *  wiced_bt_gatt_opcode_t        opcode        : LE GATT request type opcode
 *  wiced_bt_gatt_read_by_type_t  p_read_req    : Pointer to read request 
 *                                                containing the handle to read
 *  uint16_t                      len_req        : Length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t  : LE GATT status
 ******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_by_type_t *p_read_req,
                                                uint16_t len_req)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = bt_app_alloc_buffer(len_req);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("bt_app_gatt:no memory found, len_req: %d!!\r\n",len_req);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, 
     * between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle)
            break;

        if ( NULL == (puAttribute = bt_app_find_by_handle(attr_handle)))
        {
            printf("bt_app_gatt:found type but no attribute for %d \r\n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            bt_app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                                len_req - used_len,
                                                                &pair_len,
                                                                attr_handle,
                                                                puAttribute->cur_len,
                                                                puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        printf("bt_app_gatt:attr not found start_handle: 0x%04x  end_handle: 0x%04x \
                                                        type: 0x%04x\r\n",
                                                        p_read_req->s_handle,
                                                        p_read_req->e_handle,
                                                        p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        bt_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)bt_app_free_buffer);
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_write_value
********************************************************************************
* Summary:
* This function handles writing to the attribute handle in the GATT database
* using the data passed from the BT stack. The value to write is stored in a
* buffer whose starting address is passed as one of the function parameters
*
* Parameters:
*  uint16_t attr_handle      : GATT attribute handle
*  uint8_t p_val            : Pointer to BLE GATT write request value
*  uint16_t len              : length of GATT write request
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_write_value(uint16_t attr_handle,
                                                    uint8_t *p_val, uint16_t len)
{
    wiced_bt_gatt_status_t gatt_status  = WICED_BT_GATT_INVALID_HANDLE;
    wiced_bool_t isHandleInTable = WICED_FALSE;

    /* Zoek het attribuut in de externe lookup tabel en kopieer de waarde */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            isHandleInTable = WICED_TRUE;

            if (app_gatt_db_ext_attr_tbl[i].max_len >= len)
            {
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                /* Optioneel: debug wanneer CCCD geschreven wordt */
                if (attr_handle == HDLD_HIDS_REPORT_CLIENT_CHAR_CONFIG && len == 2)
                {
                    printf("HID CCCD = 0x%02X%02X\r\n", p_val[1], p_val[0]);
                }
            }
            else
            {
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
                printf("GATT write invalid length: handle=0x%04X len=%u (max=%u)\r\n",
                       attr_handle, len, app_gatt_db_ext_attr_tbl[i].max_len);
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
        printf("GATT write to unknown handle: 0x%04X\r\n", attr_handle);
    }

    return gatt_status;
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_write_handler
********************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  uint16_t conn_id       : Connection ID
*  wiced_bt_gatt_opcode_t opcode        : LE GATT request type opcode
*  wiced_bt_gatt_write_req_t p_write_req   : Pointer to LE GATT write request
*  uint16_t len_req       : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_write_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *p_write_req,
                                                uint16_t len_req)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    printf("bt_app_gatt_write_handler: conn_id:%d handle:0x%x offset:%d len:%d\r\n",
                                                            conn_id, 
                                                            p_write_req->handle, 
                                                            p_write_req->offset, 
                                                            p_write_req->val_len );

    /* Attempt to perform the Write Request */
    status = bt_app_gatt_req_write_value(p_write_req->handle,
                                         p_write_req->p_val,
                                         p_write_req->val_len);

    if(WICED_BT_GATT_SUCCESS != status)
    {
        printf("bt_app_gatt:GATT set attr status : 0x%x\n", status);
    }

    return (status);
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_read_handler
********************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* conn_id       : Connection ID
* opcode        : LE GATT request type opcode
* p_read_req    : Pointer to read request containing the handle to read
* len_req       : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_read_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req)
{
    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

    puAttribute = bt_app_find_by_handle(p_read_req->handle);
    if (NULL == puAttribute)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Als de host het HID Input Report leest, vul dan de laatste waarde in */
    if (p_read_req->handle == HDLC_HIDS_REPORT_VALUE)
    {
        app_hids_report[0] = g_hid_last_report;  /* laatste report-byte */
        puAttribute->cur_len = 1;                /* ons report is 1 byte */
    }

    attr_len_to_copy = puAttribute->cur_len;

    printf("bt_app_gatt_read_handler: conn_id:%d handle:0x%04X offset:%d len:%d\r\n",
           conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;

    /* Geen extra context nodig; buffer komt uit de ext. attribuuttabel */
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send,
                                                     from, NULL);
}


/*******************************************************************************
* Function Name: bt_app_gatt_conn_status_cb
********************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that 
*                                                       has connection details
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t 
                                                                    *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_BT_ERROR;

    if ( NULL != p_conn_status )
    {
        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf("Bluetooth connected with device address:" );
            bt_print_bd_address(p_conn_status->bd_addr);
            printf("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id);
            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;
            board_led_set_state(USER_LED1, LED_OFF);
        }
        else
        {
            /* Device has disconnected */
            printf("Bluetooth disconnected with device address:" );
            bt_print_bd_address(p_conn_status->bd_addr);
            printf("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id);
            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;
            /* Restart the advertisements */
            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                                         0, NULL);
            /* Failed to start advertisement. Stop program execution */
            if (CY_RSLT_SUCCESS != result)
            {
                CY_ASSERT(0u);
            }
            board_led_set_blink(USER_LED1, BLINK_SLOW);
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*******************************************************************************
 * Function Name: bt_app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 * 
 * Return:
 *  None
 *
 ******************************************************************************/
void bt_app_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}


/*******************************************************************************
 * Function Name: bt_app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 * 
 * Return:
 *  None
 *
 ******************************************************************************/
void* bt_app_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}


/*******************************************************************************
 * Function Name : bt_app_find_by_handle
 * *****************************************************************************
 * Summary :
 *    Find attribute description by handle
 *
 * Parameters:
 *  uint16_t handle    handle to look up
 *
 * Return:
 *  gatt_db_lookup_table_t   pointer containing handle data
 * 
 ******************************************************************************/
gatt_db_lookup_table_t  *bt_app_find_by_handle(uint16_t handle)
{
    for (uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}


/*******************************************************************************
* Function Name: bt_app_send_notification
********************************************************************************
* Summary: Sends GATT notification.
*
* Parameters:
*    None
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_send_notification(void)
{
    static uint8_t last_slider = 0;

    /* Buttons (voorbeeld) */
    if (capsense_data.buttonstatus1 == 1u) {      // Button0: Play/Pause
        hid_send_cc_report(CC_PLAY_PAUSE);
        hid_send_cc_report(0);
        capsense_data.buttonstatus1 = 0u;
    } else if (capsense_data.buttonstatus1 == 2u) { // Button1: Mute
        hid_send_cc_report(CC_MUTE);
        hid_send_cc_report(0);
        capsense_data.buttonstatus1 = 0u;
    }

    /* Slider in stapjes */
    uint8_t s = capsense_data.sliderdata;  // 0..100
    int diff = (int)s - (int)last_slider;

    if (diff >= (int)VOL_STEP_PERCENT) {
        hid_send_cc_report(CC_VOL_UP);
        hid_send_cc_report(0);
        last_slider += VOL_STEP_PERCENT;
    } else if (diff <= -(int)VOL_STEP_PERCENT) {
        hid_send_cc_report(CC_VOL_DOWN);
        hid_send_cc_report(0);
        last_slider -= VOL_STEP_PERCENT;
    }
}

/*******************************************************************************
* Function Name: bt_print_bd_address
********************************************************************************
* Summary: This is the utility function that prints the address of the 
*          Bluetooth device
*
* Parameters:
*  wiced_bt_device_address_t bdaddr : Bluetooth address
*
* Return:
*  None
*
*******************************************************************************/
void bt_print_bd_address(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=0;i<BD_ADDR_LEN-1;i++)
    {
        printf("%02X:",bdadr[i]);
    }
    printf("%02X\n",bdadr[BD_ADDR_LEN-1]);
}


/* END OF FILE [] */
