/******************************************************************************
* File Name: rgb_led.h
*
* Description: This file contains all the function prototypes of RGB Led effects
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
#ifndef INCLUDE_RGB_LED_H_
#define INCLUDE_RGB_LED_H_

/*******************************************************************************
* Include header files
*******************************************************************************/
#include "main.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define P6_CMD_BOOTUP 					1

#define P6_CMD_BLUETOOTH 				2
#define P6_DAT_PAIRING_MODE 			1
#define P6_DAT_PAIRING_STARTED 			2
#define P6_DAT_PAIRING_DONE 			3
#define P6_DAT_PAIRING_FAILED 			4

#define P6_CMD_WIFI 					3
#define P6_DAT_WIFI_STATUS1 			1
#define P6_DAT_WIFI_STATUS2 			2
#define P6_DAT_WIFI_CONNECTED 			3
#define P6_DAT_WIFI_STATUS4  			4
#define P6_DAT_MATTER_PROV_SUCCESS		5
#define P6_DAT_MATTER_PROV_FAILED 		6

#define P6_CMD_MQTT 					4
#define P6_DAT_MQTT_CONNECTING 			1
#define P6_DAT_MQTT_CONNECTED 			2
#define P6_DAT_MQTT_DISCONNECTED 		3

#define P6_CMD_PIN 						5
#define P6_DAT_NEW_PIN_CONFIGURATION 	1
#define P6_DAT_PIN_CORRECT 				2
#define P6_DAT_PIN_INCORRECT 			3
#define P6_DAT_PIN_INCORRECT_3_TIMES 	4
#define P6_DAT_PIN_ENTRY_DISABLED 		5

#define P6_CMD_NFC   					6
#define P6_DAT_NFC_ADD_NEW_CARD			1
#define P6_DAT_NFC_SUCCESS 				2
#define P6_DAT_NFC_FAILED 				3
#define P6_DAT_NFC_PROVISION_CARD       4
#define P6_DAT_NFC_BLOCK_CARDS          5

#define P6_CMD_SMART_LOCK_STATE 		7
#define P6_DAT_LOCK 					1
#define P6_DAT_UNLOCK 					2

#define P6_CMD_BATTERY 					8
#define P6_DAT_BATTERY_LOW 				1
#define P6_DAT_BATTERY_CRITICAL 		2

#define P6_CMD_FACTORY_RST 				9

#define P6_CMD_OTA_IN_PROGRESS 			10

/* Minimum PWM duty cycle required */
#define MIN_PWM_DUTYCYCLE       (5u)
/* Maximum PWM duty cycle required */
#define MAX_PWM_DUTYCYCLE       (100 * RGB_LED_DUTYCYCLE_MULTIPLY_FACTOR)
/* PWM duty cycle Step size for LED fade */
#define PWM_STEP_FADE           (15 * RGB_LED_DUTYCYCLE_MULTIPLY_FACTOR)
/* PWM duty cycle Step size for LED blink */
#define PWM_STEP_BLINK          (20 * RGB_LED_DUTYCYCLE_MULTIPLY_FACTOR)
/* PWM duty cycle Step size for Low Battery Blink */
#define PWM_STEP_LOW_BATT       (LOW_BATTERY_LED_DUTY_CYCLE / 3)

/* LED PWM frequency */
#define LED_PWM_FREQ_US         (20000u)
/* SW PWM Frequency to generate the minimum duty cycle required */
#define SW_PWM_FREQ_US          (MIN_PWM_DUTYCYCLE*LED_PWM_FREQ_US/100)

/*******************************************************************************
 * Structure
 ******************************************************************************/

enum p6_event_e
{
    bootup = 1,                     //1

    pairing_mode,                   //2
    pairing_started,                //3
    pairing_done,                   //4
    pairing_failed,                 //5

    wifi_connected,                 //6
    matter_provision_success,       //7
    matter_provision_failed,        //8

    mqtt_connecting,                //9
    mqtt_connected,                 //10
    mqtt_disconnected,              //11

    pin_new_configuration,          //12
    pin_correct,                    //13
    pin_incorrect,                  //14
    pin_three_times_incorrect,      //15
    pin_entry_disabled,             //16

    nfc_add_new_card,               //17
    nfc_success,                    //18
    nfc_failed,                     //19
    nfc_provision_card,             //20
    nfc_block_cards,                 //21

    lock,                           //22
    unlock,                         //23

    battery_low,                    //24
    battery_critical,               //25

    factory_reset,                  //26
    ota_in_progress                 //27
};

enum led_color_e
{
    RED_COLOR = 1,
    GREEN_COLOR,
    BLUE_COLOR
};

typedef struct {
    uint16_t                    dutyCycle;
    GPIO_PRT_Type*              port;
    uint32_t                    pin;
    bool                        gpio_status;
    uint16_t                    led_on_time;
}pwm_config;

/*******************************************************************************
*         Function Prototypes
*******************************************************************************/
cy_rslt_t pwm_init(void);
void process_pwm_led();
void rgb_led_reset();
void set_pwm(uint16_t red_pwm, uint16_t green_pwm, uint16_t blue_pwm);
void capsense_prox_led(uint16_t duty_cycle);
cy_rslt_t pwm_stop(void);
cy_rslt_t pwm_start(void);

#if ENABLE_LED_TEST
void Led_Blink_Test(void);
#endif

cy_rslt_t wdt_pwm_init(pwm_config *obj,uint16_t duty_cycle,uint32_t gpio_pin,GPIO_PRT_Type* gpio_port);
void pwm_callback(void);

#endif /* INCLUDE_RGB_LED_H_ */

/* [] END OF FILE */
