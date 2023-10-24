/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC4 MSCLP CAPSENSE low power
 * proximity tuning code example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
//#include "cyhal.h"
#include "main.h"
#include "capsense.h"
#include "gpio.h"
#include "rgb_led.h"
#include "i2c.h"
#include "wdt.h"
#include "uart.h"
#include "releases.h"
#if ENABLE_CS_TUNING
#include "cstune.h"
#endif

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

#if CY_CAPSENSE_BIST_EN
static cy_en_capsense_bist_status_t MeasureSensorCapacitance(uint32_t *SensorCapacitance, uint32_t *ShieldCapacitance);
#endif

#if ENABLE_TUNER
static void Ezi2cIsr(void);
static void InitializeCapsenseTuner(void);
#endif

#if ENABLE_RUN_TIME_MEASUREMENT
static void InitSysTick();
static void StartRuntimeMeasurement();
static uint32_t StopRuntimeMeasurement();
#endif

#if(SLEEP_TYPE == DEEP_SLEEP)
/* Deep Sleep Callback function */
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
                                         cy_en_syspm_callback_mode_t mode);
#elif(SLEEP_TYPE == NORMAL_SLEEP)
cy_en_syspm_status_t sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
                                         cy_en_syspm_callback_mode_t mode);
#else
#endif

static void initialise_peripherals(void);
static void register_callbacks(void);

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/
/* Variables holds the current low power state [ACTIVE, ALR or WOT] */
APPLICATION_STATE appState;

#if ENABLE_TUNER
cy_stc_scb_ezi2c_context_t ezi2cContext;

const cy_stc_scb_ezi2c_config_t CYBSP_EZI2C_config =
{
    .numberOfAddresses = CY_SCB_EZI2C_ONE_ADDRESS,
    .slaveAddress1 = 8U,
    .slaveAddress2 = 0U,
    .subAddressSize = CY_SCB_EZI2C_SUB_ADDR16_BITS,
    .enableWakeFromSleep = true,
};

/* Callback parameters for EzI2C */
cy_stc_syspm_callback_params_t ezi2cCallbackParams =
{
    .base       = SCB1,
    .context    = &ezi2cContext
};

/* Callback declaration for EzI2C Deep Sleep callback */
cy_stc_syspm_callback_t ezi2cCallback =
{
    .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
    .type           = CY_SYSPM_DEEPSLEEP,
    .skipMode       = 0UL,
    .callbackParams = &ezi2cCallbackParams,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 0
};
#endif

#if ENABLE_I2C
extern cy_stc_scb_i2c_context_t CYBSP_I2C_context;
cy_stc_syspm_callback_params_t callback_params_i2c =
{
    .base       = CYBSP_I2C_HW,
    .context    = &CYBSP_I2C_context
};

cy_stc_syspm_callback_t i2c_cb =
{
    Cy_SCB_I2C_DeepSleepCallback,      /* Callback function */
    CY_SYSPM_DEEPSLEEP,       /* Callback type */
    0,                        /* Skip mode */
    &callback_params_i2c,         /* Callback params */
    NULL,
    NULL
};
#endif

/* SysPm callback params */
cy_stc_syspm_callback_params_t sleep_callback_params =
{
    .base       = NULL,
    .context    = NULL
};

#if(SLEEP_TYPE == DEEP_SLEEP)
/* Callback declaration for Deep Sleep mode */
cy_stc_syspm_callback_t deep_sleep_cb =
{
    .callback       = &deep_sleep_callback,
    .type           = CY_SYSPM_DEEPSLEEP,
    .skipMode       = 0UL,
    .callbackParams = &sleep_callback_params,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 2
};
#elif (SLEEP_TYPE == NORMAL_SLEEP)
/* Callback declaration for CPU Sleep mode */
cy_stc_syspm_callback_t sleep_cb =
{
    .callback       = &sleep_callback,
    .type           = CY_SYSPM_SLEEP,
    .skipMode       = 0UL,
    .callbackParams = &sleep_callback_params,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 2
};
#else
#endif

cy_stc_sysclk_context_t sysclk_context;

cy_stc_syspm_callback_params_t sysclk_cb_params =
{
    .base       = NULL,
    .context    = (void*)&sysclk_context
};

/* Callback declaration for Deep Sleep mode */
cy_stc_syspm_callback_t sysclk_callback =
{
    .callback       = &Cy_SysClk_DeepSleepCallback,
    .type           = CY_SYSPM_DEEPSLEEP,
    .skipMode       = 0UL,
    .callbackParams = &sysclk_cb_params,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 0
};

volatile int proximity_status = 0, prev_proximity_status = 0;
volatile int proximity_led_state = 0;

volatile int new_p6_event, prev_p6_event_input = 0;

extern uint8_t pwm_led_active;

/* Variables holds the current low power state [ACTIVE, ALR or WOT] */
APPLICATION_STATE capsense_state = ACTIVE_MODE;
/*******************************************************************************
 * Function Definitions
 *******************************************************************************/

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE
 *  - initialize tuner communication
 *  - scan proximity and touch continuously at 3 different power modes
 *  - serial RGB LED for proximity and touch indication
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t appStateTimeoutCount = 0;

    volatile int clear_fast_mode_timeout_flag = 1;
    static int keypad_prev_scan_state = 0;

    /* This is used to make sure Prox On
     * logs are not printed repeatedly when
     * proximity is active
     */
    static int prox_state_changed = 0;

    /* Variables to store parasitic capacitance values of each sensor measured by BIST*/
#if CY_CAPSENSE_BIST_EN
    uint32_t SensorCapacitance[CY_CAPSENSE_SENSOR_COUNT];
    uint32_t ShieldCapacitance[CY_CAPSENSE_TOTAL_CH_NUMBER];
#endif

#if ENABLE_RUN_TIME_MEASUREMENT
    static uint32_t prox_scan_time;
    static uint32_t keypad_scan_time;
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if ENABLE_RUN_TIME_MEASUREMENT
    InitSysTick();
#endif

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize EZI2C */
#if ENABLE_TUNER
    InitializeCapsenseTuner();
#endif

    /* Register sleep/Deep sleep related call backs*/
    register_callbacks();

    /* Initialize all the peripherals in the system */
    initialise_peripherals();

    /* Initialize MSC CapSense */
    initialize_capsense();

    /* measure sensor Cp */
#if CY_CAPSENSE_BIST_EN
    result = MeasureSensorCapacitance(SensorCapacitance, ShieldCapacitance);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if ENABLE_UART
    UART_print_string("Sensor Capacitance \t");
    for(uint8_t i=0; i < CY_CAPSENSE_SENSOR_COUNT; i++)
    {
        UART_print_data(SensorCapacitance[i]);
        UART_print_string("\t");
    }
    UART_print_string("\r\n");

    UART_print_string("Shield Capacitance \t");
    for(uint8_t i=0; i < CY_CAPSENSE_TOTAL_CH_NUMBER; i++)
    {
        UART_print_data(ShieldCapacitance[i]);
        UART_print_string("\t");
    }
    UART_print_string("\r\n\r\n");
#endif
#endif

    /* Measures the actual ILO frequency and compensate MSCLP wake up timers */
    Cy_CapSense_IloCompensate(&cy_capsense_context);

    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);

#if ENABLE_LED_TEST
    /* Test for Onboard RGB LED's*/
    Led_Blink_Test();
#endif

    for (;;)
    {
        /* If the proximity sensor is active */
        if (1 == proximity_status)
        {
            /* Check if there is change in prox state */
            if (0 == prox_state_changed)
            {
#if ENABLE_UART
                UART_print_string("\r\nPROX ON\r\n");
#endif
#if (!ENABLE_LED)
                Cy_GPIO_Set(KEYPADLED_RED_PORT, KEYPADLED_RED_PIN);
#endif
                prox_state_changed = 1;

#if ENABLE_I2C
                /* Send Proximity active indication to P6 */
                i2c_send_data('P');
                trigger_i2c_read();
#endif
            }
        }
        else /* if (proximity_status == 0) */
        {
            /* Check if prox led timeout has occurred and if there is change in prox state */
            if ((0 == prev_proximity_status) &&
                    (1 == prox_state_changed))
            {
                prox_state_changed = 0;
#if ENABLE_UART
                UART_print_string("PROX OFF\r\n");
#endif
#if (!ENABLE_LED)
                Cy_GPIO_Clr(KEYPADLED_RED_PORT, KEYPADLED_RED_PIN);
#endif

#if ENABLE_I2C
                /* Send Proximity inactive indication to P6 */
                i2c_send_data('p');
                trigger_i2c_read();
#endif
            }
        }

        switch (capsense_state)
        {
            /* Active Refresh-rate Mode */
            case ACTIVE_MODE:

                if (0 == keypad_prev_scan_state)
                {
#if ENABLE_RUN_TIME_MEASUREMENT
                    prox_scan_time = 0u;
#if (ENABLE_UART)
                    UART_print_string("***************scan_prox_test****************\r\n");
#endif
                    StartRuntimeMeasurement();
                    Cy_GPIO_Set(HOST_INTR_PORT, HOST_INTR_PIN);
#endif
                    /* Scan the proximity sensor */
                    if (0 == scan_proximity())
                    {
                        /* Process Proximity sensor scan data */
                        process_proximity();
                        /* Get the proximity sensor status */
                        proximity_status = is_proximity_active();
                    }
                    else
                    {
                        /* Don't take any logical decisions as long as
                         * proximity scan is not complete
                         */
                        break;
                    }

#if ENABLE_RUN_TIME_MEASUREMENT
                    Cy_GPIO_Clr(HOST_INTR_PORT, HOST_INTR_PIN);
                    prox_scan_time = StopRuntimeMeasurement();
#if (ENABLE_UART)
                    UART_print_data(prox_scan_time);
                    UART_print_string("\n\n");
#endif

#endif
                }

                /* If Proximity sensor is active */
                if (1 == proximity_status)
                {
                    clear_fast_mode_timeout_flag = 1;
#if (ENABLE_SNR_MEASUREMENT)
                }
                {
#endif
#if ENABLE_RUN_TIME_MEASUREMENT
                    keypad_scan_time = 0u;
#if (ENABLE_UART)
                    UART_print_string("***************Keypad_process_test****************\r\n");
#endif
                    StartRuntimeMeasurement();
                    Cy_GPIO_Set(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN);
#else
                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(CAPSENSE_BUSY_CHECK_INT_US, &cy_capsense_context);
#endif
                    keypad_prev_scan_state = scan_keypad();
                    if (keypad_prev_scan_state == 0)
                    {
                        process_keypad();

                        /* Get the data on what all keys are pressed */
                        get_keyins();

#if ENABLE_RUN_TIME_MEASUREMENT
                        Cy_GPIO_Clr(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN);
                        keypad_scan_time = StopRuntimeMeasurement();
#if (ENABLE_UART)
                        UART_print_data(keypad_scan_time);
                        UART_print_string("\n\n");
#endif
#endif

                    }
                    else
                    {
                        /* Don't take any logical decisions as long
                         * as keypad scan is not complete
                         */
                        break;
                    }

                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }
                /* If Proximity sensor is not active */
#if (!ENABLE_SNR_MEASUREMENT)
                else if (0 == proximity_status)
#else
                if (0 == proximity_status)
#endif
                {
                    /* Make sure to reset fast mode timeout counter
                     * value as soon as we find out that proximity
                     * is inactive */
                    if ((1 == clear_fast_mode_timeout_flag))
                    {
                        clear_fast_mode_timeout_counter_value();
                        clear_fast_mode_timeout_flag = 0;
                        appStateTimeoutCount = TIMEOUT_RESET;

                        /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                        Cy_CapSense_ConfigureMsclpTimer(WDT_TIMEOUT_FAST, &cy_capsense_context);
                    }
                    else
                    {
                        /* check inactivity for 30 seconds and switch to Cap sense Slow scan mode */
                        if ((get_fast_mode_timeout_counter_value()) > (FASTSCAN_MODE_TIMEOUT))
                        {
                            if(0 == pwm_led_active)
                            {
                                /* Update WDT with a new match value */
                                set_wdt_match_value(WDT_TIMEOUT_SLOW);
                            }

                            /* Change the state to Slow Scan */
                            capsense_state = ALR_MODE;
                            appStateTimeoutCount = TIMEOUT_RESET;
                            /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                            Cy_CapSense_ConfigureMsclpTimer(ALR_MODE_TIMER, &cy_capsense_context);
#if (ENABLE_UART)
                            /* Print Transition to Slow Scan */
                            UART_print_string("T2 Slow Scan \r\n");
#endif
                        }
                        else
                        {
                            /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                            Cy_CapSense_ConfigureMsclpTimer(WDT_TIMEOUT_FAST, &cy_capsense_context);
                        }
                    }
                }

                break;
                /* End of ACTIVE_MODE */

                /* Active Low Refresh-rate Mode */
            case ALR_MODE:

                if (0 == scan_proximity())
                {
                    process_proximity();
                    proximity_status = is_proximity_active();
                }
                else
                {
                    /* Don't take any logical decisions as long as proximity scan is complete */
                    break;
                }
                if (1 == proximity_status)
                {
                    if(!pwm_led_active)
                    {
                        /* Update WDT match value for soft PWM if not already done */
                        set_wdt_match_value(WDT_TIMEOUT_FAST);
                    }

                    /* Switch to Fast Scan mode */
                    capsense_state = ACTIVE_MODE;
                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
#if (ENABLE_UART)
                    /* Print Transition to Fast Scan due to proximity sensor being activated */
                    UART_print_string("T2 Fast Scan due to [Prox Active]\r\n");
#endif
                }
                else if (0 == proximity_status)
                {
                    appStateTimeoutCount++;

                    if(ALR_MODE_TIMEOUT < appStateTimeoutCount)
                    {
                        capsense_state = WOT_MODE;
                        appStateTimeoutCount = TIMEOUT_RESET;
                    }

                    /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ALR_MODE_TIMER, &cy_capsense_context);
                }
                else
                {
                    /* This indicates cap sense is busy processing previous scan */
                }

                break;
                /* End of Active-Low Refresh Rate(ALR) mode */

                /* Wake On Touch Mode */
            case WOT_MODE:
                /* Trigger the low power widget scan */
                Cy_CapSense_ScanAllLpSlots(&cy_capsense_context);

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    /* Enter and stay in Deep Sleep until WOT timeout or a touch is detected. */
                    /* WOT Timeout = WOT scan interval x Num of frames in WOT (in uSec);
                     * Refer to Wake-On-Touch settings in CAPSENSE Configurator for WOT Timeout*/

                    Cy_SysPm_CpuEnterDeepSleep();
                }

                /* Process only the Low Power widgets to detect touch */
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_LOWPOWER0_WDGT_ID, &cy_capsense_context);

                if (Cy_CapSense_IsAnyLpWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_MODE;
                    appStateTimeoutCount = TIMEOUT_RESET;

                    if(!pwm_led_active)
                    {
                        /* Update WDT match value for soft PWM if not already done */
                        set_wdt_match_value(WDT_TIMEOUT_FAST);
                    }
                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }
                else
                {
                    capsense_state = ALR_MODE;
                    appStateTimeoutCount = TIMEOUT_RESET;

                    /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ALR_MODE_TIMER, &cy_capsense_context);
                }
                break;
                /* End of "WAKE_ON_TOUCH_MODE" */

            default:
                /* Unknown scan state. Unexpected situation to be in!!!
                 * Control should never come here. Block the program flow!
                 **/
                while (1)
                    ;
                break;
        }

#if ENABLE_TUNER
        /* Establishes synchronized communication with the CAPSENSE&trade; Tuner tool */
        Cy_CapSense_RunTuner(&cy_capsense_context);
#endif
        /* Update baseline if any widget is found to active for long */
        reinit_capsense_baseline();
    }
}


/*******************************************************************************
 * Function Name: initialise_peripherals
 ********************************************************************************
 * Summary:
 *  Initializes peripherals in the system
 *
 * Return:
 * None
 *******************************************************************************/
static void initialise_peripherals(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#if ENABLE_UART
    debug_uart_init();
#endif

#ifdef ENABLE_CS_TUNING
    cstune_uart_init();
    cstune_register_callbacks();
#endif

    /* Initialize GPIOs */
    gpio_init();

#if ENABLE_I2C
    /* Initialize I2C module */
    result = i2c_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif

    /* Initialize PWM module */
    result = pwm_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize WDT module */
    wdt_init();

//  DBG_INFO("\r\n IFX Smart Lock Touch FW v0.5.0 \r\n");
    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    UART_print_string("\x1b[2J\x1b[;H");
    UART_print_string("************************************************************\r\n");
    UART_print_string("             IFX Smart Lock Touch FW V");
    UART_print_data(releaseFIRMWARE_VERSION_MAJOR);
    UART_print_string(".");
    UART_print_data(releaseFIRMWARE_VERSION_MINOR);
    UART_print_string(".");
    UART_print_data(releaseFIRMWARE_VERSION_PATCH);
    UART_print_string("\r\n");
    UART_print_string("************************************************************\r\n\n");
}

/*******************************************************************************
 * Function Name: InitializeCapsenseTuner
 ********************************************************************************
 * Summary:
 * EZI2C module to communicate with the CAPSENSE Tuner tool.
 *
 *******************************************************************************/
#if ENABLE_TUNER
static void InitializeCapsenseTuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2cIntrConfig =
    {
        .intrSrc = CYBSP_I2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_I2C_HW, &CYBSP_EZI2C_config, &ezi2cContext);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2cIntrConfig, Ezi2cIsr);
    NVIC_EnableIRQ(ezi2cIntrConfig.intrSrc);

    /* Set the CAPSENSE data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_I2C_HW, (uint8_t *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &ezi2cContext);

    Cy_SCB_EZI2C_Enable(CYBSP_I2C_HW);
}

/*******************************************************************************
 * Function Name: Ezi2cIsr
 ********************************************************************************
 * Summary:
 * Wrapper function for handling interrupts from EZI2C block.
 *
 *******************************************************************************/
static void Ezi2cIsr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_I2C_HW, &ezi2cContext);
}

#endif

/*******************************************************************************
 * Function Name: register_callbacks
 ********************************************************************************
 * Summary:
 *  Registers sleep related callbacks
 *
 * Return:
 * None
 *******************************************************************************/
static void register_callbacks(void)
{
#if ENABLE_TUNER
    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);
#endif

#if ENABLE_I2C
    Cy_SysPm_RegisterCallback(&i2c_cb);
#endif

#if(SLEEP_TYPE == DEEP_SLEEP)
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&deep_sleep_cb);
#elif(SLEEP_TYPE == NORMAL_SLEEP)

    /* Register Sleep callback */
    Cy_SysPm_RegisterCallback(&sleep_cb);
#else
#endif

    /* Register SysClk Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysclk_callback);
}

#if(SLEEP_TYPE == DEEP_SLEEP)
/*******************************************************************************
 * Function Name: deep_sleep_callback
 ********************************************************************************
 * Summary:
 * Deep Sleep callback implementation.
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 *******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
                                         cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
        	if(cy_retarget_io_is_tx_active())
        	{
        		ret_val = CY_SYSPM_FAIL;
        	}
#else
            if (0)
            {
                ;
            }
#endif
            else
            {
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
        		cy_retarget_io_deinit();
#endif
//                if (CY_RSLT_SUCCESS != pwm_stop())
//                {
//                    CY_ASSERT(0);
//                }

                ret_val = CY_SYSPM_SUCCESS;
            }
        }
            break;

        case CY_SYSPM_CHECK_FAIL:
        {
            ret_val = CY_SYSPM_SUCCESS;
        }
            break;

        case CY_SYSPM_BEFORE_TRANSITION:
        {
            /* Use GPIO transition to mark entry in to Deep sleep state */
            DEBUG_GPIO_SLEEP_ENTRY;

            ret_val = CY_SYSPM_SUCCESS;
        }

            break;

        case CY_SYSPM_AFTER_TRANSITION:
        {
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
        	if(CY_RSLT_SUCCESS != cy_retarget_io_init(UART_TX, UART_RX, CY_RETARGET_IO_BAUDRATE))
        	{
        		CY_ASSERT(0);
        	}
#endif
            /* Re-start PWM after wake up */
//            if (CY_RSLT_SUCCESS != pwm_start())
//            {
//                CY_ASSERT(0);
//            }

            /* Use GPIO transition to mark exit from Deep sleep state */
            DEBUG_GPIO_SLEEP_EXIT;

            ret_val = CY_SYSPM_SUCCESS;
        }
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }

    return ret_val;
}
#elif(SLEEP_TYPE == NORMAL_SLEEP)
/*******************************************************************************
* Function Name: deep_sleep_callback
********************************************************************************
* Summary:
* Deep Sleep callback implementation.
*
* Parameters:
*  callbackParams: The pointer to the callback parameters structure cy_stc_syspm_callback_params_t.
*  mode: Callback mode, see cy_en_syspm_callback_mode_t
*
* Return:
*  Entered status, see cy_en_syspm_status_t.
*
*******************************************************************************/
cy_en_syspm_status_t sleep_callback(
        cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
#if defined(DEBUG_LEVEL_INFO) || defined(DEBUG_LEVEL_ERR)
        	if(cy_retarget_io_is_tx_active())
        	{
        		ret_val = CY_SYSPM_FAIL;
        	}
#else
        	if(0)
        	{
        		;
        	}
#endif
        	else
        	{
        		ret_val = CY_SYSPM_SUCCESS;
        	}
         }
		break;

        case CY_SYSPM_CHECK_FAIL:
		{
			ret_val = CY_SYSPM_SUCCESS;
		}
		break;

        case CY_SYSPM_BEFORE_TRANSITION:
        {
        	/* Use GPIO transition to mark entry in to sleep state */
        	DEBUG_GPIO_SLEEP_ENTRY;

        	ret_val = CY_SYSPM_SUCCESS;
        }
		break;

        case CY_SYSPM_AFTER_TRANSITION:
        {
			/* Use GPIO transition to mark exit from sleep state */
			DEBUG_GPIO_SLEEP_EXIT;

			ret_val = CY_SYSPM_SUCCESS;
        }
		break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
		break;
    }

    return ret_val;
}
#else
#endif

#if CY_CAPSENSE_BIST_EN
/*******************************************************************************
* Function Name: MeasureSensorCapacitance
********************************************************************************
* Summary:
*  Measure the sensor Capacitance of all sensors configured and stores the values in an array using BIST.
*  BIST Measurements are taken by Connection connecting ISC to Shield.
*  It is based on actual application configuration.
* Parameters:
*   SensorCapacitance - This array holds the measured sensor capacitance values.
*                        array values are arranged as Low power widget sensors first and
*                        followed by regular widget sensors. refer configurator for the
*                        sensor order.
*
*   ShieldCapacitance - This array holds the measured shield capacitance values.
*                        array values are arranged as Low power widget sensors first and
*                        followed by regular widget sensors . refer configurator for the
*                        sensor order.
*
*******************************************************************************/
static cy_en_capsense_bist_status_t MeasureSensorCapacitance(uint32_t *SensorCapacitance, uint32_t *ShieldCapacitance)
{
	cy_en_capsense_bist_status_t result;
    /* For BIST configuration Connecting all Inactive sensor connections (ISC) of CSD sensors to to shield*/
    Cy_CapSense_SetInactiveElectrodeState(CY_CAPSENSE_SNS_CONNECTION_SHIELD,
                                          CY_CAPSENSE_BIST_CSD_GROUP, &cy_capsense_context);
    /* For BIST configuration Connecting all Inactive sensor connections (ISC) of CSX sensors to to GND*/
    Cy_CapSense_SetInactiveElectrodeState(CY_CAPSENSE_SNS_CONNECTION_SHIELD,
                                          CY_CAPSENSE_BIST_CSD_GROUP, &cy_capsense_context);

    /*Runs the BIST to measure the sensor capacitance*/
    result = Cy_CapSense_RunSelfTest(CY_CAPSENSE_BIST_SNS_CAP_MASK,
                                         &cy_capsense_context);

    memcpy(SensorCapacitance,
            cy_capsense_context.ptrWdConfig->ptrSnsCapacitance,
            CY_CAPSENSE_SENSOR_COUNT * sizeof(uint32_t));

    /* For BIST configuration Connecting all Inactive sensor connections (ISC) of Shield sensors to to Shield*/
    Cy_CapSense_SetInactiveElectrodeState(CY_CAPSENSE_SNS_CONNECTION_SHIELD,
                                          CY_CAPSENSE_BIST_SHIELD_GROUP, &cy_capsense_context);

    /*Runs the BIST to measure the Shield capacitance*/
    Cy_CapSense_RunSelfTest(CY_CAPSENSE_BIST_SHIELD_CAP_MASK,
                            &cy_capsense_context);

    memcpy(ShieldCapacitance,
           cy_capsense_context.ptrBistContext->ptrChShieldCap,
           CY_CAPSENSE_TOTAL_CH_NUMBER * sizeof(uint32_t));

    return result;
}
#endif

#if ENABLE_RUN_TIME_MEASUREMENT
/*******************************************************************************
* Function Name: InitSysTick
********************************************************************************
* Summary:
*  initializes the system tick with highest possible value to start counting down.
*
*******************************************************************************/
static void InitSysTick()
{
    Cy_SysTick_Init (CY_SYSTICK_CLOCK_SOURCE_CLK_CPU ,0x00FFFFFF);
}

/*******************************************************************************
* Function Name: StartRuntimeMeasurement
********************************************************************************
* Summary:
*  Initializes the system tick counter by calling Cy_SysTick_Clear() API.
*******************************************************************************/
static void StartRuntimeMeasurement()
{
    Cy_SysTick_Clear();
}

/*******************************************************************************
* Function Name: StopRuntimeMeasurement
********************************************************************************
* Summary:
*  Reads the system tick and converts to time in microseconds(us).
*
*  Returns:
*  runTime - in microseconds(us)
*******************************************************************************/

static uint32_t StopRuntimeMeasurement()
{
    uint32_t ticks;
    uint32_t runTime;
    ticks = Cy_SysTick_GetValue();
    ticks = (SYS_TICK_INTERVAL - Cy_SysTick_GetValue());
    runTime = (ticks * TIME_PER_TICK_IN_US);
    return runTime;
}
#endif

/* [] END OF FILE */
