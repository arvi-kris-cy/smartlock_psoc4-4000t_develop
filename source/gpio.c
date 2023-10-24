/*******************************************************************************
 * File Name:   gpio.c
 *
 * Description: This file contains GPIO related functions.
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
#include "cy_pdl.h"
#include "cybsp.h"
#include "gpio.h"
#include "cycfg_capsense.h"
#include "main.h"

/*******************************************************************************
 * User Configurable Macros
 *******************************************************************************/

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
 * Function Name: gpio_init
 ********************************************************************************
 * Summary:
 *  Initializes GPIO module
 *
 * Return:
 * -1  - Initialization failed
 *  0   - Initialization success
 *******************************************************************************/
int gpio_init(void)
{
#if (!ENABLE_UART)
    Cy_GPIO_Pin_Init(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN, &KEYPADLED_BLUE_config);
    Cy_GPIO_SetHSIOM(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN, HSIOM_SEL_GPIO);
#endif

#if defined (SWD_PINS_AS_DBG_GPIOS)

    cyhal_hwmgr_free(&SWDIO_obj);
    cyhal_hwmgr_free(&SWDCLK_obj);

	/* Configure SWDIO pins as GPIOs for debug purpose
	 *
	 * With SWDIO as GPIO, you can program P4 using mini-prog but
	 * can't put it into debug mode.
	 *
	 * SWDIO
	 * P3.2  Available on Pin 5 of Mini-Prog header coming out of Smart Lock
	 * Use to debug deep sleep entry and exit points
	 *
	 * SWDCLK
	 * P3.3  Available on Pin 4 of Mini-Prog header coming out of Smart Lock
	 * Used to debug Proximity on state
	 *
	 * Ground is available on Pin 2 of Mini prog header coming out of Smart Lock
	 */
	cyhal_gpio_init(SWDIO, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	cyhal_gpio_init(SWDCLK, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

	/* This is just for debug to identify boot up */
	cyhal_gpio_write(SWDIO, 1);
	cyhal_system_delay_ms(1);
	cyhal_gpio_write(SWDIO, 0);
	cyhal_system_delay_ms(1);
	cyhal_gpio_write(SWDIO, 1);
	cyhal_system_delay_ms(1);
	cyhal_gpio_write(SWDIO, 0);
	cyhal_system_delay_ms(1);
	cyhal_gpio_write(SWDIO, 1);
	cyhal_gpio_write(SWDIO, 0);
#endif //SWD_PINS_AS_DBG_GPIOS

    return 0;
}

/*******************************************************************************
 * Function Name: set_prox_active_pin
 ********************************************************************************
 * Summary:
 *  Sets a GPIO to indicate Proximity sensor is active
 *
 * Return:
 * None
 *******************************************************************************/
void set_prox_active_pin(void)
{

//    cyhal_gpio_write(HOST_INTR, 1);

    DEBUG_GPIO_NFC_ACTIVE;
}

/*******************************************************************************
 * Function Name: clear_prox_active_pin
 ********************************************************************************
 * Summary:
 * Clears a GPIO to indicate Proximity sensor is not active
 *
 * Return:
 * None
 *******************************************************************************/
void clear_prox_active_pin(void)
{
//    cyhal_gpio_write(HOST_INTR, 0);

    DEBUG_GPIO_NFC_INACTIVE;
}

/*******************************************************************************
 * Function Name: trigger_i2c_read
 ********************************************************************************
 * Summary:
 *  Toggles Host Interrupt GPIO to trigger an I2C read from the P6.
 *
 * Return:
 * None
 *******************************************************************************/
void trigger_i2c_read(void)
{
#if (!ENABLE_RUN_TIME_MEASUREMENT)
    Cy_GPIO_Set(HOST_INTR_PORT, HOST_INTR_PIN);
    Cy_SysLib_Delay(1);
    Cy_GPIO_Clr(HOST_INTR_PORT, HOST_INTR_PIN);
#endif
}

/* [] END OF FILE */
