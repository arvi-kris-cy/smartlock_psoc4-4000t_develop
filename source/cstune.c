/*******************************************************************************
 * File Name:   cstune.c
 *
 * Description: This file contains Capsense tuning related functions.
 *
 *******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

/*******************************************************************************
* Include header files
*******************************************************************************/
#include "cybsp.h"
#include "main.h"
#include "cycfg_peripherals.h"

/*******************************************************************************
* User Configurable Macros
*******************************************************************************/


/*******************************************************************************
* Fixed Macros
*******************************************************************************/


/*******************************************************************************
* Global Definitions
*******************************************************************************/
cy_stc_scb_uart_context_t CYBSP_UART_context;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void cstune_receive_uart(uint8_t** packet, uint8_t** tuner_packet, void *context);
void cstune_send_uart(void* context);


/*******************************************************************************
 * Function Name: cstune_send_uart
 ********************************************************************************
 * Summary:
 *  Sends CapSense Tuning data over UART
 *
 * Return:
 * None
 *******************************************************************************/
void cstune_send_uart(void* context)
{
#ifdef ENABLE_CS_TUNING
	uint8_t uart_tx_header[] = {0x0Du, 0x0Au};
    uint8_t uart_tx_tail[]   = {0x00u, 0xFFu, 0xFFu};

    (void)context;

    Cy_SCB_UART_PutArrayBlocking(UART_HW, &(uart_tx_header[0u]), sizeof(uart_tx_header));

    Cy_SCB_UART_PutArrayBlocking(UART_HW, (uint8_t *)&cy_capsense_tuner, sizeof(cy_capsense_tuner));

    Cy_SCB_UART_PutArrayBlocking(UART_HW, uart_tx_tail, sizeof(uart_tx_tail));
#endif
}


/*******************************************************************************
 * Function Name: cstune_receive_uart
 ********************************************************************************
 * Summary:
 *  Receives CapSense tuning data over UART
 *
 * Return:
 * None
 *******************************************************************************/
void cstune_receive_uart(uint8_t** packet, uint8_t** tuner_packet, void* context)
{
#ifdef ENABLE_CS_TUNING
    uint32_t i;
    (void) context;
    static uint32_t data_index = 0u;
    static uint8_t command_packet[16u] = {0u};

    while(0u != Cy_SCB_UART_GetNumInRxFifo(UART_HW))
    {
    	command_packet[data_index++] = (uint8_t)Cy_SCB_UART_Get(UART_HW);
        if (CY_CAPSENSE_COMMAND_PACKET_SIZE <= data_index)
        {
            if (CY_CAPSENSE_COMMAND_OK == Cy_CapSense_CheckTunerCmdIntegrity(&command_packet[0u]))
            {
                /* Found a correct command, reset data index and assign pointers to buffers */
            	data_index = 0u;
                *tuner_packet = (uint8_t *)&cy_capsense_tuner;
                *packet = &command_packet[0u];
                break;
            }
            else
            {
                /* Command is not correct, remove the first byte in commandPacket FIFO */
                data_index--;
                for(i = 0u; i < (CY_CAPSENSE_COMMAND_PACKET_SIZE - 1u); i++)
                {
                    command_packet[i] = command_packet[i + 1u];
                }
            }
        }
    }
#endif
}


/*******************************************************************************
 * Function Name: cstune_run_tuner
 ********************************************************************************
 * Summary:
 *  Runs CapSense tuner
 *
 * Return:
 * None
 *******************************************************************************/
void cstune_run_tuner(void)
{
	/* Establishes synchronized communication with the CapSense Tuner tool */
	Cy_CapSense_RunTuner(&cy_capsense_context);
}


/*******************************************************************************
 * Function Name: cstune_register_callbacks
 ********************************************************************************
 * Summary:
 *  Registers callback functions for the flow of CapSense tuning data
 *
 * Return:
 * None
 *******************************************************************************/
void cstune_register_callbacks(void)
{
	/* Register cs tuner communication call backs */
    cy_capsense_context.ptrInternalContext->ptrTunerSendCallback 	= cstune_send_uart;
    cy_capsense_context.ptrInternalContext->ptrTunerReceiveCallback = cstune_receive_uart;
}


/*******************************************************************************
 * Function Name: cstune_uart_init
 ********************************************************************************
 * Summary:
 *  Initializes CapSense tuning UART
 *
 * Return:
 * None
 *******************************************************************************/
void cstune_uart_init(void)
{
#ifdef ENABLE_CS_TUNING
	Cy_SCB_UART_Init(UART_HW, &UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(UART_HW);
#endif
}


/* [] END OF FILE */


