/******************************************************************************
 * File Name: main.h
 *
 * Description: This file contains all the function prototypes of main
 *
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
 *******************************************************************************/
#ifndef INCLUDE_MAIN_H_
#define INCLUDE_MAIN_H_

/*******************************************************************************
 * Include header files
 *******************************************************************************/
//#include "cy_retarget_io.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "gpio.h"

/*******************************************************************************
 * User configurable Macros
 ********************************************************************************/
/****************************** Debug configurations ****************************/
/* Define this macro if SWD pins are to be used as GPIOs for a debug purpose */
//#define SWD_PINS_AS_DBG_GPIOS
#if defined (SWD_PINS_AS_DBG_GPIOS)

/* SWDIO transition from 1 to 0 is used to identify
 * entry and exit points during deep sleep or normal sleep.
 * A high level on SWDIO pin indicates that the P4 is in deep
 * sleep/sleep state based on type of sleep enabled -
 * deep or normal.
 */
#define DEBUG_GPIO_SLEEP_ENTRY 		(cyhal_gpio_write(SWDIO, 1))
#define DEBUG_GPIO_SLEEP_EXIT 		(cyhal_gpio_write(SWDIO, 0))

/* SWDCLK PIO is used to find if the NFC is active or inactive
 * SWDCLK = 1 indicates NFC active
 * SWDCLK = 0 indicates NFC inactive
 *
 * Logically, NFC is activated only if the proximity sensor is
 * active, transitions on SWDCLK PIO also gives info proximity
 * sensor state.
 **/
#define DEBUG_GPIO_NFC_ACTIVE      (cyhal_gpio_write(SWDCLK, 1))
#define DEBUG_GPIO_NFC_INACTIVE	   (cyhal_gpio_write(SWDCLK, 0))

#else
#define DEBUG_GPIO_SLEEP_ENTRY
#define DEBUG_GPIO_SLEEP_EXIT

#define DEBUG_GPIO_NFC_ACTIVE
#define DEBUG_GPIO_NFC_INACTIVE
#endif

/* Define this macro if Capsense needs to be tuned.
 * The tuning can be done only over UART interface.
 * If this macro is defined, debug prints don't occur
 */
//#define ENABLE_CS_TUNING
//#define DEBUG_LEVEL_INFO	/* Define this macro to enable debugging info messages to be printed */
//#define DEBUG_LEVEL_ERR	/* Define this macro to enable error messages to be printed */
#ifdef DEBUG_LEVEL_INFO
#define DBG_INFO(...) (printf(__VA_ARGS__))
#else
#define DBG_INFO(...)
#endif

#ifdef DEBUG_LEVEL_ERR
#define DBG_ERR(...) (printf(__VA_ARGS__))
#else
#define DBG_ERR(...)
#endif
/********************************************************************************/
/****************************** Sleep configurations ****************************/
#define NORMAL_SLEEP                        (0)
#define DEEP_SLEEP                          (1)

/* Select between sleep and deep sleep as the idle/low power state */
#define SLEEP_TYPE                          (DEEP_SLEEP)
/********************************************************************************/
/****************************** Peripheral configurations ***********************/
/* Enable run time measurements for various modes of the application,
 * this run time is used to calculate MSCLP timer reload value */
#define ENABLE_RUN_TIME_MEASUREMENT         (0u)

/* Disable this macro to disable RGB LED feature which is used for showing proximity and lock related statuses */
#define ENABLE_LED                          (1)

/* Enable this for RGB LED test during device startup */
#define ENABLE_LED_TEST                     (0u)

#if (!ENABLE_RUN_TIME_MEASUREMENT)
/* Host INTR pin and UART TX pins are used for Run time Measurement,
 * So P6 cannot be connected */
/* Enable this for debug logs */
#define ENABLE_UART                         (0u)

/* Disable this macro to disable handling I2C communication between I2C master and this device */
#define ENABLE_I2C                          (1u)
#endif

#if (!ENABLE_I2C)
/* Enable this, if Tuner needs to be enabled */
#define ENABLE_TUNER                        (1u)
/* Enable this for SNR Measurement of sensors */
#define ENABLE_SNR_MEASUREMENT              (0u)
#endif

/********************************************************************************/
/****************************** Capsense configurations *************************/
/* Interval for capsense scan status check */
#define CAPSENSE_BUSY_CHECK_INT_US          (1000u) // 500us

/* 128Hz Refresh rate in Active mode */
#if (!ENABLE_RUN_TIME_MEASUREMENT)
#define ACTIVE_MODE_REFRESH_RATE            (50u)//(128u) // ~8ms ---> (20u) ~20ms
#else
/* Wake up every 100us for timing measurement */
#define ACTIVE_MODE_REFRESH_RATE            (10000u)//(128u) // ~8ms ---> (20u) ~20ms
#endif

/* 32Hz Refresh rate in Active-Low Refresh rate(ALR) mode */
#define ALR_MODE_REFRESH_RATE               (5u)//(32u) // ~31.25ms ---> (5u) ~200ms

/* Timeout to move from ACTIVE mode to ALR mode if there is no user activity */
#define ACTIVE_MODE_TIMEOUT_SEC             (5u)

/* Timeout to move from ALR mode to WOT mode if there is no user activity */
#define ALR_MODE_TIMEOUT_SEC                (5u)

/* Scan time in microseconds */
#define ACTIVE_MODE_FRAME_SCAN_TIME         (37u)

/* Active mode Processing time in us ~= 23us with Serial LED and Tuner disabled*/
#define ACTIVE_MODE_PROCESS_TIME            (23u)

/* Scan time in microseconds */
#define ALR_MODE_FRAME_SCAN_TIME            (37u)

/* ALR mode Processing time in us ~= 23us with Serial LED and Tuner disabled*/
#define ALR_MODE_PROCESS_TIME               (23u)
/********************************************************************************/
/****************************** LED PWM configurations **************************/
/* Decide the pulse width factor for RGB leds to scale down brightness */
#define RGB_LED_DUTYCYCLE_MULTIPLY_FACTOR   (0.2)

/*The WDT is configured to interrupt every 50ms, hence (50ms*30) = 1.5 seconds */
#define PROX_LED_ON_TIMEOUT                 (30)

/* Defines the brightness of the LED on proximity active */
#define PROX_ACTIVE_LED_DUTY_CYCLE          (100 * RGB_LED_DUTYCYCLE_MULTIPLY_FACTOR)

/* Defines the brightness of the LED on Low Battery level */
#define LOW_BATTERY_LED_DUTY_CYCLE          (3)

/* Set WDT period for Fast scan and LED Soft PWM */
#define WDT_TIMEOUT_FAST               (50000UL)

/* 1.6384 second of WDT period, this is the max that
 * can be achieved  with 16-bit WDT timer.
 */
//#define WDT_TIMEOUT_SLOW 65535
/* Set 1000ms as WDT period for watchdog kick */
#define WDT_TIMEOUT_SLOW               (1000000UL)

#if (!ENABLE_SNR_MEASUREMENT)
/* In fast scan, the WDT is configured to interrupt every 50ms, hence (50ms*60) = 3 seconds */
#define FASTSCAN_MODE_TIMEOUT               (PROX_LED_ON_TIMEOUT + PROX_ACTIVE_LED_DUTY_CYCLE + 10)
#else
/* In fast scan, the WDT is configured to interrupt every 50ms, hence (50ms*600) = 30 seconds */
#define FASTSCAN_MODE_TIMEOUT               (600)
#endif

/*******************************************************************************
 * Fixed Macros
 *******************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)

#define CY_ASSERT_FAILED                 (0u)

#define EZI2C_INTR_PRIORITY              (2u)

#define ILO_FREQ                        (40000u)
#define TIME_IN_US                      (1000000u)
#define MINIMUM_TIMER                   (TIME_IN_US / ILO_FREQ)

/* 128Hz Refresh rate in Active mode */
#if ((TIME_IN_US / ACTIVE_MODE_REFRESH_RATE) > (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
    #define ACTIVE_MODE_TIMER           (TIME_IN_US / ACTIVE_MODE_REFRESH_RATE - \
                                        (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
#elif
    #define ACTIVE_MODE_TIMER           (MINIMUM_TIMER)
#endif

#if ((TIME_IN_US / ALR_MODE_REFRESH_RATE) > (ALR_MODE_FRAME_SCAN_TIME + ALR_MODE_PROCESS_TIME))
    #define ALR_MODE_TIMER              (TIME_IN_US / ALR_MODE_REFRESH_RATE - \
                                            (ALR_MODE_FRAME_SCAN_TIME + ALR_MODE_PROCESS_TIME))
#elif
    #define ALR_MODE_TIMER              (MINIMUM_TIMER)
#endif

#define ACTIVE_MODE_TIMEOUT             (ACTIVE_MODE_REFRESH_RATE * ACTIVE_MODE_TIMEOUT_SEC)

#define ALR_MODE_TIMEOUT                (ALR_MODE_REFRESH_RATE * ALR_MODE_TIMEOUT_SEC)

#define TIMEOUT_RESET                   (0u)

#if ENABLE_RUN_TIME_MEASUREMENT
    #define SYS_TICK_INTERVAL           (0x00FFFFFF)
    #define TIME_PER_TICK_IN_US         ((float)1/CY_CAPSENSE_CPU_CLK)*TIME_IN_US
#endif

/*****************************************************************************
 * Finite state machine states for device operating states
 *****************************************************************************/
typedef enum
{
    ACTIVE_MODE = 0x01u,    /* Active mode - All the sensors are scanned in this state
                            * with highest refresh rate */
    ALR_MODE = 0x02u,       /* Active-Low Refresh Rate (ALR) mode - All the sensors are
                            * scanned in this state with low refresh rate */
    WOT_MODE = 0x03u        /* Wake on Touch (WoT) mode - Low Power sensors are scanned
                            * in this state with lowest refresh rate */
} APPLICATION_STATE;

/*******************************************************************************
 *         Function Prototypes
 *******************************************************************************/

#endif /* INCLUDE_MAIN_H_ */

/* [] END OF FILE */
