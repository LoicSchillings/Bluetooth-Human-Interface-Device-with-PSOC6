/*******************************************************************************
* File Name: capsense.h
*
* Description: This file is the public interface of capsense.c source
*              file.
*
* Related Document: README.md
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
*  Include guard
*******************************************************************************/
#ifndef CAPSENSE_H
#define CAPSENSE_H


/*******************************************************************************
* Header file includes
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/*******************************************************************************
* Global Constants
*******************************************************************************/
typedef enum
{
    CAPSENSE_SCAN,
    CAPSENSE_PROCESS
} capsense_command_t;

/* Structure used for storing CapSense data */
typedef struct
{
    uint8_t sliderdata;         /* Contains CapSense slider data */
    uint8_t buttoncount;        /* Contains CapSense button count */
    uint8_t buttonstatus1;      /* Contains CapSense button status  */
    uint8_t buttonstatus2;      /* Contains CapSense button status  */
}capsense_data_t;


/*******************************************************************************
* Global Variables
*******************************************************************************/
extern QueueHandle_t capsense_command_q;
extern capsense_data_t capsense_data;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void capsense_task(void* param);


#endif /* CAPSENSE_H */