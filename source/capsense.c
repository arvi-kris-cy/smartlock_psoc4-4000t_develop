/*******************************************************************************
 * File Name:   capsense.c
 *
 * Description: This file contains the Capsense related functions.
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
#include "cycfg.h"
#include "cycfg_capsense.h"
//#include "cyhal.h"
#include "capsense.h"
#include "gpio.h"
#include "rgb_led.h"
#include "wdt.h"
#include "cstune.h"
#include "main.h"
#include "i2c.h"
#include "uart.h"

/*******************************************************************************
 * User Configurable Macros
 *******************************************************************************/
/* When any of the sensors is active, the WDT period will be 50ms
 * Hence, (50ms *300) = 15 seconds*/
#define STUCK_CAPSENSE_SENSOR_TIMEOUT 300

/*******************************************************************************
 * Fixed Macros
 *******************************************************************************/
/* Number of proximity sensors */
#define PROXIMITY_SENSOR_COUNT 1

/* Number of Keypad sensors */
#define KEYPAD_SENSOR_COUNT     7
#define KEYPAD_ROW_COUNT        4
#define KEYPAD_COLUMN_COUNT     3


/*******************************************************************************
 * Global Definitions
 *******************************************************************************/

char keypad_matrix[KEYPAD_ROW_COUNT][KEYPAD_COLUMN_COUNT] = {{'1','2','3'},
                                                             {'4','5','6'},
                                                             {'7','8','9'},
                                                             {'*','0','#'}};

uint8_t capsense_intr = 0;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/*******************************************************************************
 * Function Name: Capsense_Msc0Isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CAPSENSE MSC0 block.
 *
 * Return:
 * None
 *******************************************************************************/
static void Capsense_Msc0Isr(void)
{
    capsense_intr = 1;
    Cy_CapSense_InterruptHandler(CY_MSCLP0_HW, &cy_capsense_context);
}

#if 0//ENABLE_RUN_TIME_MEASUREMENT
void capsense_scan_end(cy_stc_capsense_active_scan_sns_t * ptrActiveScan)
{
//    UART_print_string("Scan Done\n\r");
}
#endif

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  It initializes the CapSense Blocks and configures the CapSense interrupt.
 *
 *  Return:
 *  None
 *
 *******************************************************************************/
void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CAPSENSE interrupt configuration MSC 0 */
    const cy_stc_sysint_t msc0InterruptConfig =
    {
        .intrSrc = CY_MSCLP0_LP_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CAPSENSE interrupt for MSC 0 */
        Cy_SysInt_Init(&msc0InterruptConfig, Capsense_Msc0Isr);
        NVIC_ClearPendingIRQ(msc0InterruptConfig.intrSrc);
        NVIC_EnableIRQ(msc0InterruptConfig.intrSrc);

#if 0//ENABLE_RUN_TIME_MEASUREMENT
        Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E, capsense_scan_end, &cy_capsense_context);
#endif
        /* Initialize the CAPSENSE firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CAPSENSE sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}

/*******************************************************************************
 * Function Name: reinit_capsense_baseline
 ********************************************************************************
 * Summary:
 *  Re-initialises CapSense baselines if any of the sensors is active for set time
 *
 * Return:
 * None
 *******************************************************************************/
void reinit_capsense_baseline(void)
{
	static int sensor_stuck_counter_latch = 0;

	if (Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
	{
		if(0 == sensor_stuck_counter_latch)
		{
			clear_sensor_stuck_counter_value();
			sensor_stuck_counter_latch = 1;
		}
		if(get_sensor_stuck_counter_value() >= STUCK_CAPSENSE_SENSOR_TIMEOUT)
		{
			sensor_stuck_counter_latch = 0;
			UART_print_string("Baseline Re-Init Done\n\r");

//			Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
			Cy_CapSense_InitializeWidgetBaseline(CY_CAPSENSE_PROXIMITY0_WDGT_ID, &cy_capsense_context);
		}
	}
	else
	{
		sensor_stuck_counter_latch = 0;
	}
}

/*******************************************************************************
 * Function Name: get_keyins
 ********************************************************************************
 * Summary:
 *  Reads status of all the Keypad buttons
 *
 * Return:
 * None
 *******************************************************************************/
void get_keyins(void)
{
    static uint8_t button_pressed = 0;
    uint16_t keypad_pos_x = 255;
    uint16_t keypad_pos_y = 255;


    char key_press = 0;
    static char prev_key_press = 0;

    if ((Cy_CapSense_IsSensorActive(CY_CAPSENSE_COL0_WDGT_ID, CY_CAPSENSE_COL0_SNS0_ID, &cy_capsense_context)) ||
            (Cy_CapSense_IsSensorActive(CY_CAPSENSE_COL1_WDGT_ID, CY_CAPSENSE_COL1_SNS0_ID, &cy_capsense_context)) ||
            (Cy_CapSense_IsSensorActive(CY_CAPSENSE_COL2_WDGT_ID, CY_CAPSENSE_COL2_SNS0_ID, &cy_capsense_context)))
    {
        if ((Cy_CapSense_IsSensorActive(CY_CAPSENSE_ROW0_WDGT_ID, CY_CAPSENSE_ROW0_SNS0_ID, &cy_capsense_context)) ||
                (Cy_CapSense_IsSensorActive(CY_CAPSENSE_ROW1_WDGT_ID, CY_CAPSENSE_ROW1_SNS0_ID, &cy_capsense_context)) ||
                (Cy_CapSense_IsSensorActive(CY_CAPSENSE_ROW2_WDGT_ID, CY_CAPSENSE_ROW2_SNS0_ID, &cy_capsense_context)) ||
                (Cy_CapSense_IsSensorActive(CY_CAPSENSE_ROW3_WDGT_ID, CY_CAPSENSE_ROW3_SNS0_ID, &cy_capsense_context)))
        {
            if (CY_CAPSENSE_COL0_SNS0_DIFF0_VALUE > (CY_CAPSENSE_COL1_SNS0_DIFF0_VALUE + 10))
            {
                if (CY_CAPSENSE_COL0_SNS0_DIFF0_VALUE > (CY_CAPSENSE_COL2_SNS0_DIFF0_VALUE + 10))
                    keypad_pos_y = 0;
                else
                {
                    if ((CY_CAPSENSE_COL0_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_COL2_SNS0_DIFF0_VALUE)
                        keypad_pos_y = 2;
                }
            }
            else
            {
                if ((CY_CAPSENSE_COL0_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_COL1_SNS0_DIFF0_VALUE)
                {
                    if (CY_CAPSENSE_COL1_SNS0_DIFF0_VALUE > (CY_CAPSENSE_COL2_SNS0_DIFF0_VALUE + 10))
                        keypad_pos_y = 1;
                    else
                    {
                        if ((CY_CAPSENSE_COL1_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_COL2_SNS0_DIFF0_VALUE)
                            keypad_pos_y = 2;
                    }
                }
                else
                {
                    if ((CY_CAPSENSE_COL1_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_COL2_SNS0_DIFF0_VALUE)
                        keypad_pos_y = 2;
                }
            }

            if (CY_CAPSENSE_ROW0_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE + 10))
            {
                if (CY_CAPSENSE_ROW0_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10))
                {
                    if (CY_CAPSENSE_ROW0_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE + 10))
                        keypad_pos_x = 0;
                    else
                    {
                        if ((CY_CAPSENSE_ROW0_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                            keypad_pos_x = 3;
                    }
                }
                else
                {
                    if ((CY_CAPSENSE_ROW0_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE)
                    {
                        if (CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE + 10))
                            keypad_pos_x = 2;
                        else
                        {
                            if ((CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                                keypad_pos_x = 3;
                        }
                    }
                    else
                    {
                        if ((CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                            keypad_pos_x = 3;
                    }
                }
            }
            else
            {
                if ((CY_CAPSENSE_ROW0_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE)
                {
                    if (CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10))
                    {
                        if (CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE + 10))
                            keypad_pos_x = 1;
                        else
                        {
                            if ((CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                                keypad_pos_x = 3;
                        }
                    }
                    else
                    {
                        if ((CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE)
                        {
                            if (CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE + 10))
                                keypad_pos_x = 2;
                            else
                            {
                                if ((CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                                    keypad_pos_x = 3;
                            }
                        }
                        else
                        {
                            if ((CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                                keypad_pos_x = 3;
                        }
                    }
                }
                else
                {
                    if ((CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE)
                    {
                        if (CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE > (CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE + 10))
                            keypad_pos_x = 2;
                        else
                        {
                            if ((CY_CAPSENSE_ROW2_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                                keypad_pos_x = 3;
                        }
                    }
                    else
                    {
                        if ((CY_CAPSENSE_ROW1_SNS0_DIFF0_VALUE + 10) < CY_CAPSENSE_ROW3_SNS0_DIFF0_VALUE)
                            keypad_pos_x = 3;
                    }
                }
            }

            key_press = keypad_matrix[keypad_pos_x][keypad_pos_y];

            if ((key_press != prev_key_press) && (keypad_pos_x != 255) && (keypad_pos_y != 255))
            {
                button_pressed = 1;
#if ENABLE_UART
                UART_print_string("\r\nKey Pressed: ");
                UART_print_string(&key_press);
                UART_print_string("\r\n");
#endif

                i2c_send_data(key_press);
                trigger_i2c_read();

                clear_sensor_stuck_counter_value();
                prev_key_press = key_press;
            }
        }
        else
        {
            button_pressed = 0;
        }
    }
    else
    {
        button_pressed = 0;
    }

    if (button_pressed == 0)
    {
        prev_key_press = -1;
    }
}

/*******************************************************************************
 * Function Name: scan_proximity
 ********************************************************************************
 * Summary:
 *  Returns state of Proximity Sensor scanning
 *
 * Return:
 * -1 if scanning is still not complete
 *  0 if scanning is complete
 *******************************************************************************/
int scan_proximity(void)
{
    uint32_t interruptStatus;
    cy_capsense_status_t scan_status = 0;

    scan_status = Cy_CapSense_ScanSlots(CY_CAPSENSE_PROXIMITY0_FIRST_SLOT_ID,
                                        PROXIMITY_SENSOR_COUNT, &cy_capsense_context);
    if (scan_status != CY_CAPSENSE_STATUS_SUCCESS)
    {
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
        UART_print_string(" PROX Scan Failed \r\n");
#endif
        return -1;
    }

    interruptStatus = Cy_SysLib_EnterCriticalSection();

    while ((0 == capsense_intr) ||
            (Cy_CapSense_IsBusy(&cy_capsense_context)))
    {
        capsense_intr = 0;

        Cy_SysPm_CpuEnterDeepSleep();

        Cy_SysLib_ExitCriticalSection(interruptStatus);
        interruptStatus = Cy_SysLib_EnterCriticalSection();
    }
    Cy_SysLib_ExitCriticalSection(interruptStatus);

    return 0;
}

/*******************************************************************************
 * Function Name: scan_keypad
 ********************************************************************************
 * Summary:
 *  Returns state of Keypad scanning
 *
 * Return:
 * -1 if scanning is still not complete
 *  0 if scanning is complete
 *******************************************************************************/
int scan_keypad(void)
{
    uint32_t interruptStatus;

    cy_capsense_status_t scan_status;

    scan_status = Cy_CapSense_ScanSlots(CY_CAPSENSE_COL0_FIRST_SLOT_ID, KEYPAD_SENSOR_COUNT,
                                        &cy_capsense_context);
    if (scan_status != CY_CAPSENSE_STATUS_SUCCESS)
    {
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
        UART_print_string(" Keypad Scan Failed \r\n");
#endif
        return -1;
    }

    interruptStatus = Cy_SysLib_EnterCriticalSection();
    while ((0 == capsense_intr) ||
            (Cy_CapSense_IsBusy(&cy_capsense_context)))
    {
        capsense_intr = 0;

        Cy_SysPm_CpuEnterDeepSleep();

        Cy_SysLib_ExitCriticalSection(interruptStatus);
        interruptStatus = Cy_SysLib_EnterCriticalSection();
    }
    Cy_SysLib_ExitCriticalSection(interruptStatus);

    return 0;
}

/*******************************************************************************
 * Function Name: process_keypad
 ********************************************************************************
 * Summary:
 *  Process all the keypad widgets
 *
 * Return:
 * None
 *******************************************************************************/
void process_keypad(void)
{
	if(CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_ProcessAllWidgets(&cy_capsense_context))
	{
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
	    //Cy_SCB_UART_PutString(UART_HW, "Process Widgets failed\r\n\n");
		UART_print_string("Process Widgets failed\r\n\n");
#endif
	}
}

/*******************************************************************************
 * Function Name: process_proximity
 ********************************************************************************
 * Summary:
 *  Process Proximity widget
 *
 * Return:
 * None
 *******************************************************************************/
void process_proximity(void)
{
	if(CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_ProcessWidget(CY_CAPSENSE_PROXIMITY0_WDGT_ID, &cy_capsense_context))
	{
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
	    // Cy_SCB_UART_PutString(UART_HW, "Process Prox failed\r\n\n");
		 UART_print_string("Process Prox failed\r\n\n");
#endif
	}
#ifdef ENABLE_CS_TUNING
	cstune_run_tuner();
#endif
}

/*******************************************************************************
 * Function Name: is_proximity_active
 ********************************************************************************
 * Summary:
 *  Checks if the Proximity widget is active
 *
 * Return:
 * 1 - Proximity widget is active
 * 0 - Proximity widget is not active
 *******************************************************************************/
int is_proximity_active(void)
{
	int ret = 0;

	if (Cy_CapSense_IsSensorActive(
			CY_CAPSENSE_PROXIMITY0_WDGT_ID,
			CY_CAPSENSE_PROXIMITY0_SNS0_ID, &cy_capsense_context))
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}

	return ret;
}

/* [] END OF FILE */
