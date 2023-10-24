/*******************************************************************************
 * File Name:   wdt.c
 *
 * Description: This file contains Watchdog timer related functions.
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
#include "wdt.h"
#include "stdio.h"
#include "gpio.h"
#include "main.h"
#include "rgb_led.h"
#include "i2c.h"
#include "uart.h"

/*******************************************************************************
* User Configurable Macros
*******************************************************************************/


/*******************************************************************************
* Fixed Macros
*******************************************************************************/
/* WDT interrupt priority */
#define WDT_INTERRUPT_PRIORITY     		(1U)

#define TIME_IN_MS                      (1000u)

/* ILO frequency for PSoC 4100S Max device */
#define ILO_CLOCK_FACTOR                (40u)

/* Waiting time, in milliseconds, for proper start-up of ILO */
#define ILO_START_UP_TIME               (2U)

/* Set the desired number of ignore bits */
#define IGNORE_BITS                     (0U)

/* Reset value of softCounter */
#define RESET                           (0u)

/* Boolean constants */
#define TRUE                            (1u)
#define FALSE                           (0u)

/*******************************************************************************
* Global Definitions
*******************************************************************************/
/* WDT interrupt service routine configuration */
const cy_stc_sysint_t wdt_isr_cfg =
{
    .intrSrc = srss_interrupt_wdt_IRQn, /* Interrupt source is WDT interrupt */
    .intrPriority = WDT_INTERRUPT_PRIORITY
};

/* Contains watch dog match value to generate periodic interrupt */
volatile uint32_t wdt_match_value;

/* A counter to keep track of time we spend in Cap sense Fast scan
 * mode before we switch over to Cap sense slow scan mode
 **/
static volatile uint16_t fast_mode_timeout_counter = 0;

/* A counter to keep track of time after which Capsense
 * led is turned off */
static volatile uint16_t led_on_timeout_counter = 0;

static volatile uint16_t sensor_stuck_counter = 0;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

void wdt_isr(void);

/*****************************************************************************
* Function Name: wdt_init
******************************************************************************
* Summary:
* This function initializes the WDT block
*
* Return:
* None
*****************************************************************************/
void wdt_init(void)
{
    cy_en_sysint_status_t status = CY_SYSINT_BAD_PARAM;

    /* Step 1- Write the ignore bits - operate with full 16 bits */
    Cy_WDT_SetIgnoreBits(IGNORE_BITS);
    if(Cy_WDT_GetIgnoreBits() != IGNORE_BITS)
    {
        CY_ASSERT(0);
    }

    /* Step 2- Clear match event interrupt, if any */
    Cy_WDT_ClearInterrupt();

    /* Step 3- Enable ILO */
    Cy_SysClk_IloEnable();

    /* Waiting for proper start-up of ILO */
    Cy_SysLib_Delay(ILO_START_UP_TIME);

    set_wdt_match_value(WDT_TIMEOUT_FAST);

    /* Step 5 - Initialize and enable interrupt if periodic interrupt
       mode selected */
    status = Cy_SysInt_Init(&wdt_isr_cfg, wdt_isr);
    if(status != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(wdt_isr_cfg.intrSrc);
    Cy_WDT_UnmaskInterrupt();

    /* Step 6- Enable WDT */
    Cy_WDT_Enable();
    if(Cy_WDT_IsEnabled() == FALSE)
    {
        CY_ASSERT(0);
    }

    /* Check if system reset is due to WDT */
	if (CY_SYSLIB_RESET_HWWDT  == Cy_SysLib_GetResetReason())
	{
		UART_print_string("\r<<<<<<<<<<<<<<<<<< WDT Restart >>>>>>>>>>>>>>>>>>>>>>\r\n");
	}

}


/*****************************************************************************
* Function Name: wdt_isr
******************************************************************************
* Summary:
* This function is the handler for the WDT interrupt
*
* Return:
* None
*****************************************************************************/
void wdt_isr(void)
{
    extern volatile uint8_t pwm_led_active;
	/* Clear WDT Interrupt */
	Cy_WDT_ClearInterrupt();

	/* Mask the WDT interrupt to prevent further triggers */
    Cy_WDT_MaskInterrupt();

    static uint16_t led_toggle_count = 0;
    uint8_t increment_counters = 0;

    if(pwm_led_active)
    {
        pwm_callback();

        led_toggle_count += SW_PWM_FREQ_US;

        if(led_toggle_count >= WDT_TIMEOUT_FAST)
        {
            led_toggle_count = 0;
            increment_counters = 1;
        }
    }
    else
    {
        increment_counters = 1;
        wdt_kick();
    }

    if(increment_counters == 1)
    {
        increment_counters = 0;

        /* Handle the PWM LED indications */
        process_pwm_led();

        fast_mode_timeout_counter++;
        led_on_timeout_counter++;
        sensor_stuck_counter++;
    }

	/* Un-mask the WDT interrupt */
	Cy_WDT_UnmaskInterrupt();
}


/*****************************************************************************
* Function Name: get_sensor_stuck_counter_value
******************************************************************************
* Summary:
* This function returns a counter value
*
* Return:
*  int
*****************************************************************************/
int get_sensor_stuck_counter_value(void)
{
	return sensor_stuck_counter;
}

void inc_sensor_stuck_counter_value(void)
{
    ++sensor_stuck_counter;
}


/*****************************************************************************
* Function Name: get_sensor_stuck_counter_value
******************************************************************************
* Summary:
* This function returns a counter value
*
* Parameters:
*  void
*
* Return:
*  int
*****************************************************************************/
void clear_sensor_stuck_counter_value(void)
{
	sensor_stuck_counter = 0;
}


/*****************************************************************************
* Function Name: get_led_on_counter_value
******************************************************************************
* Summary:
* This function returns a counter value
*
* Return:
*  int
*****************************************************************************/
int get_led_on_counter_value(void)
{
	return led_on_timeout_counter;
}

void inc_led_on_counter_value(void)
{
    ++led_on_timeout_counter;
}


/*****************************************************************************
* Function Name: clear_led_on_counter_value
******************************************************************************
* Summary:
* This function clears a counter value
*
* Return:
*  None
*****************************************************************************/
void clear_led_on_counter_value(void)
{
	led_on_timeout_counter = 0;
}


/*****************************************************************************
* Function Name: get_fast_mode_timeout_counter_value
******************************************************************************
* Summary:
* This function returns a counter value
*
* Return:
*  int
*****************************************************************************/
int get_fast_mode_timeout_counter_value(void)
{
	return fast_mode_timeout_counter;
}


/*****************************************************************************
* Function Name: clear_fast_mode_timeout_counter_value
******************************************************************************
* Summary:
* Clears a counter value
*
* Return:
*  None
*****************************************************************************/
void clear_fast_mode_timeout_counter_value(void)
{
	fast_mode_timeout_counter = 0;
}


/*****************************************************************************
* Function Name: wdt_kick
******************************************************************************
* Summary:
* Kicks the WDT
*
* Return:
*  None
*****************************************************************************/
void wdt_kick(void)
{
    /* Write match value if periodic interrupt mode selected */
	Cy_WDT_SetMatch((uint16_t)(Cy_WDT_GetMatch() + wdt_match_value));
}


/*******************************************************************************
* Function Name: calibrate_wdt_match_value
********************************************************************************
* Summary:
*  This function calibrates the match value of the Watchdog Timer
*
* Return:
*  void
*
* Theory: The ILO is calibrated using IMO to improve the accuracy of ILO.
*
*******************************************************************************/
uint32_t get_calibrated_wdt_match_value(uint32_t wdt_interval_us)
{
	/* Contains ILO Trimmed value */
	uint32_t temp_ilo_counts = 0u;

    /* Starts the ILO accuracy/Trim measurement */
//    Cy_SysClk_IloStartMeasurement();

    /* Calculate the count value to set as WDT match since ILO is inaccurate */
    if(CY_SYSCLK_SUCCESS != Cy_SysClk_IloCompensate(wdt_interval_us, &temp_ilo_counts))
    {
        temp_ilo_counts = (wdt_interval_us * ILO_CLOCK_FACTOR)/1000;
    }

//    Cy_SysClk_IloStopMeasurement();

    return temp_ilo_counts;
}


/*****************************************************************************
* Function Name: set_wdt_match_value
******************************************************************************
* Summary:
* Sets the match value for WDT
*
* Return:
*  None
*****************************************************************************/
void set_wdt_match_value(uint32_t wdt_interval_us)
{
//    wdt_match_value = (wdt_interval_us * ILO_CLOCK_FACTOR)/1000;
    wdt_match_value = get_calibrated_wdt_match_value(wdt_interval_us);

	Cy_WDT_SetMatch((uint16_t)(Cy_WDT_GetCount() + wdt_match_value));
//	Cy_WDT_SetMatch((uint16_t)(Cy_WDT_GetMatch() + wdt_match_value));
}


/* [] END OF FILE */
