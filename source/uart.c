/*******************************************************************************
 * File Name:   uart.c
 *
 * Description: This file contains debug UART related functions.
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
#include "cycfg.h"
#include "cycfg_peripherals.h"
#include "main.h"

#include "stdlib.h"

/*******************************************************************************
* User Configurable Macros
*******************************************************************************/
cy_stc_scb_uart_context_t debug_UART_context;

/*******************************************************************************
* Fixed Macros
*******************************************************************************/


/*******************************************************************************
* Global Definitions
*******************************************************************************/

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
 * Function Name: UART_print_data
 ********************************************************************************
 * Summary:
 * Function to print data over UART
 *
 * Parameters:
 *  uint32_t: data
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void UART_print_data(uint32_t data)
{
#if ENABLE_UART
    char buffer[5];

    Cy_GPIO_SetHSIOM(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN, KEYPADLED_BLUE_HSIOM);
    /* Enable the SCB block for the UART operation */
    Cy_SCB_UART_Enable(UART_HW);
    /* Convert Integer to ASCII */
    itoa(data, buffer, 10);
    /* Print the ASCII string */
    Cy_SCB_UART_PutString(UART_HW, buffer);
    /* Wait for UART TX to complete */
    while(!Cy_SCB_UART_IsTxComplete(UART_HW));

    Cy_SCB_UART_Disable(UART_HW, &debug_UART_context);
    Cy_GPIO_SetHSIOM(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN, HSIOM_SEL_GPIO);
#endif
}

/*******************************************************************************
 * Function Name: UART_print_string
 ********************************************************************************
 * Summary:
 * Function to print string over UART
 *
 * Parameters:
 *  char_t const: string
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void UART_print_string(char_t const string[])
{
#if ENABLE_UART
    Cy_GPIO_SetHSIOM(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN, KEYPADLED_BLUE_HSIOM);
    /* Enable the SCB block for the UART operation */
    Cy_SCB_UART_Enable(UART_HW);
    /* Print the ASCII string */
    Cy_SCB_UART_PutString(UART_HW, string);
    /* Wait for UART TX to complete */
    while(!Cy_SCB_UART_IsTxComplete(UART_HW));

    Cy_SCB_UART_Disable(UART_HW, &debug_UART_context);
    Cy_GPIO_SetHSIOM(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN, HSIOM_SEL_GPIO);
#endif
}

/*******************************************************************************
 * Function Name: debug_uart_init
 ********************************************************************************
 * Summary:
 * Initialize UART for debug logs
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void debug_uart_init(void)
{
#if ENABLE_UART
    cy_en_scb_uart_status_t init_status;

    /* Start UART operation */
    init_status =Cy_SCB_UART_Init(UART_HW, &UART_config, &debug_UART_context);
    if (init_status!=CY_SCB_UART_SUCCESS)
    {
      /* Disable all interrupts. */
      __disable_irq();

      CY_ASSERT(CY_ASSERT_FAILED);
    }

//    /* Enable the SCB block for the UART operation */
//    Cy_SCB_UART_Enable(UART_HW);
#endif
}

/* [] END OF FILE */


