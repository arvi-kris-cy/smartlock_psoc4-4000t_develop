/*******************************************************************************
 * File Name:   rgb_led.c
 *
 * Description: This file contains RGB led effect related functions.
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
//#include "cyhal.h"
#include "gpio.h"
#include "i2c.h"
#include "rgb_led.h"
#include "main.h"
#include "uart.h"
#include "wdt.h"


/*******************************************************************************
* User Configurable Macros
*******************************************************************************/


/*******************************************************************************
* Fixed Macros
*******************************************************************************/


/*******************************************************************************
* Global Definitions
*******************************************************************************/
pwm_config pwm_led_red;
pwm_config pwm_led_green;
pwm_config pwm_led_blue;

uint32_t local_counter = 0;
uint16_t fading_counter = 0;

uint32_t count_pwm = 0;
uint8_t pwm_led_active = 0;

bool lock_state = true;

extern volatile uint32_t p6_event_input;

extern APPLICATION_STATE capsense_state;
/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: Get_dutyCycle
 ********************************************************************************
 * Summary:
 *  Get the duty cycle of the LED
 *
 * Return:
 *  uint16_t: Dutycycle
 *******************************************************************************/
uint16_t Get_dutyCycle(pwm_config *obj)
{
    return obj->dutyCycle;
}

/*******************************************************************************
 * Function Name: Set_dutyCycle
 ********************************************************************************
 * Summary:
 *  Updates the duty cycle of the LED
 *
 * Return:
 * None
 *******************************************************************************/
void Set_dutyCycle(pwm_config *obj, uint16_t dutycycle)
{
    obj->dutyCycle = dutycycle;
    obj->led_on_time = ((LED_PWM_FREQ_US * obj->dutyCycle) / 100);

    if(!obj->dutyCycle)
    {
        Cy_GPIO_Clr(obj->port, obj->pin);
    }
    else if(obj->dutyCycle == 100)
    {
        Cy_GPIO_Set(obj->port, obj->pin);
    }

    if(!pwm_led_active)
    {
        /* Update WDT match value for soft PWM if not already done */
        set_wdt_match_value(WDT_TIMEOUT_FAST);
    }

    pwm_led_active = 1;
}

/*******************************************************************************
 * Function Name: rgb_led_reset
 ********************************************************************************
 * Summary:
 *  Resets color of RGB leds
 *
 * Return:
 * None
 *******************************************************************************/
void rgb_led_reset()
{
    count_pwm = 0;

    set_pwm(0, 0, 0);
}

/*******************************************************************************
 * Function Name: set_pwm
 ********************************************************************************
 * Summary:
 *  Sets duty cycle of RGB leds
 *
 * Return:
 * None
 *******************************************************************************/
void set_pwm(uint16_t red_pwm,uint16_t green_pwm,uint16_t blue_pwm)
{
#if ENABLE_LED
    Set_dutyCycle(&pwm_led_red, red_pwm);
    Set_dutyCycle(&pwm_led_green,green_pwm);
    Set_dutyCycle(&pwm_led_blue, blue_pwm);
#endif
}

/*******************************************************************************
* Function Name: wdt_pwm_init
****************************************************************************/
cy_rslt_t wdt_pwm_init(pwm_config *obj,uint16_t duty_cycle,uint32_t gpio_pin,GPIO_PRT_Type* gpio_port )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    obj->port = gpio_port;
    obj->pin = gpio_pin;
    obj->dutyCycle = duty_cycle;
    obj->led_on_time = ((LED_PWM_FREQ_US*obj->dutyCycle)/SW_PWM_FREQ_US) / 100;
    obj->gpio_status = 0;

    return result;
}

/*******************************************************************************
 * Function Name: pwm_init
 ********************************************************************************
 * Summary:
 *  Initializes PWM
 *
 * Return:
 * Status of initialization
 *******************************************************************************/
cy_rslt_t pwm_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
#if ENABLE_LED
    result = wdt_pwm_init(&pwm_led_red, 0, KEYPADLED_RED_PIN, KEYPADLED_RED_PORT );
    result = wdt_pwm_init(&pwm_led_green, 0, KEYPADLED_GREEN_PIN, KEYPADLED_GREEN_PORT);
    result = wdt_pwm_init(&pwm_led_blue, 0, KEYPADLED_BLUE_PIN, KEYPADLED_BLUE_PORT);
#endif

    return result;
}

/*******************************************************************************
 * Function Name: set_red_led
 ********************************************************************************
 * Summary:
 *  Set LED color
 *
 * Return:
 * None
 *******************************************************************************/
void set_led_color(uint8_t led_color)
{
    if(led_color == RED_COLOR)
    {
        set_pwm(MAX_PWM_DUTYCYCLE, 0, 0);
    }
    else if(led_color == GREEN_COLOR)
    {
        set_pwm(0, MAX_PWM_DUTYCYCLE, 0);
    }
    else if(led_color == BLUE_COLOR)
    {
        set_pwm(0, 0, MAX_PWM_DUTYCYCLE);
    }

    fading_counter++;
}

/*******************************************************************************
 * Function Name: blink_red_led
 ********************************************************************************
 * Summary:
 *  Blinks red led
 *
 * Return:
 * None
 *******************************************************************************/
void blink_red_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(count_pwm, 0, 0);
    }
    else if (count_pwm < (2*MAX_PWM_DUTYCYCLE))
    {
    }
    else if (count_pwm >= (2*MAX_PWM_DUTYCYCLE))
    {
        uint32_t count_val = count_pwm - (2*MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm((MAX_PWM_DUTYCYCLE - count_val), 0, 0);
    }
    count_pwm += PWM_STEP_BLINK;
}


/*******************************************************************************
 * Function Name: dim_blink_red_led
 ********************************************************************************
 * Summary:
 *  Blinks red led with lower brightness
 *
 * Return:
 * None
 *******************************************************************************/
void dim_blink_red_led(void)
{
    if (count_pwm <= LOW_BATTERY_LED_DUTY_CYCLE)
    {
        set_pwm(count_pwm, 0, 0);
    }
    else if (count_pwm < (3*LOW_BATTERY_LED_DUTY_CYCLE))
    {
    }
    else if (count_pwm >= (3*LOW_BATTERY_LED_DUTY_CYCLE))
    {
        uint32_t count_val = count_pwm - (3*LOW_BATTERY_LED_DUTY_CYCLE);

        if (count_val >= LOW_BATTERY_LED_DUTY_CYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm((LOW_BATTERY_LED_DUTY_CYCLE - count_val), 0, 0);
    }

    count_pwm += PWM_STEP_LOW_BATT;
}


/*******************************************************************************
 * Function Name: blink_green_led
 ********************************************************************************
 * Summary:
 *  Blinks green led
 *
 * Return:
 * None
 *******************************************************************************/
void blink_green_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0, count_pwm, 0);
    }
    else if (count_pwm < (2*MAX_PWM_DUTYCYCLE))
    {
    }
    else if (count_pwm >= (2*MAX_PWM_DUTYCYCLE))
    {
        uint32_t count_val = count_pwm - (2*MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0, (MAX_PWM_DUTYCYCLE - count_val), 0);
    }
    count_pwm += PWM_STEP_BLINK;
}


/*******************************************************************************
 * Function Name: blink_blue_led
 ********************************************************************************
 * Summary:
 *  Blinks blue led
 *
 * Return:
 * None
 *******************************************************************************/
void blink_blue_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0, 0, count_pwm);
    }
    else if (count_pwm < (2*MAX_PWM_DUTYCYCLE))
    {
    }
    else if (count_pwm >= (2*MAX_PWM_DUTYCYCLE))
    {
        uint32_t count_val = count_pwm - (2*MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0, 0, (MAX_PWM_DUTYCYCLE - count_val));
    }
    count_pwm += PWM_STEP_BLINK;
}


/*******************************************************************************
 * Function Name: blink_white_led
 ********************************************************************************
 * Summary:
 *  Blinks white led
 *
 * Return:
 * None
 *******************************************************************************/
void blink_white_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(count_pwm, count_pwm, count_pwm);
    }
    else if (count_pwm < (2*MAX_PWM_DUTYCYCLE))
    {
    }
    else if (count_pwm >= (2*MAX_PWM_DUTYCYCLE))
    {
        uint32_t count_val = count_pwm - (2*MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm((MAX_PWM_DUTYCYCLE - count_val), (MAX_PWM_DUTYCYCLE - count_val), (MAX_PWM_DUTYCYCLE - count_val));
    }
    count_pwm += PWM_STEP_BLINK;
}


/*******************************************************************************
 * Function Name: blink_orange_led
 ********************************************************************************
 * Summary:
 *  Blinks led in orange color
 *
 * Return:
 * None
 *******************************************************************************/
void blink_orange_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0.8 * count_pwm, 0.4 * count_pwm, 0);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = count_pwm - MAX_PWM_DUTYCYCLE;

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0.8 * (MAX_PWM_DUTYCYCLE - count_val), 0.4 * (MAX_PWM_DUTYCYCLE - count_val), 0);
    }
    count_pwm += PWM_STEP_BLINK;
}


/*******************************************************************************
 * Function Name: blink_pink_led
 ********************************************************************************
 * Summary:
 *  Blinks led in purple color
 *
 * Return:
 * None
 *******************************************************************************/
void blink_pink_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0.8 * count_pwm, 0, 0.8 * count_pwm);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = count_pwm - MAX_PWM_DUTYCYCLE;

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0.8 * (MAX_PWM_DUTYCYCLE - count_val), 0, 0.8 * (MAX_PWM_DUTYCYCLE - count_val));
    }
    count_pwm += PWM_STEP_BLINK;
}


/*******************************************************************************
 * Function Name: fade_red_led
 ********************************************************************************
 * Summary:
 *  Fades red led
 *
 * Return:
 * None
 *******************************************************************************/
void fade_red_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(count_pwm, 0, 0);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm((MAX_PWM_DUTYCYCLE - count_val), 0, 0);
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: fade_green_led
 ********************************************************************************
 * Summary:
 *  Fades green led
 *
 * Return:
 * None
 *******************************************************************************/
void fade_green_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0, count_pwm, 0);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0, (MAX_PWM_DUTYCYCLE - count_val), 0);
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: fade_blue_led
 ********************************************************************************
 * Summary:
 *  Fades blue led
 *
 * Return:
 * None
 *******************************************************************************/
void fade_blue_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0, 0, count_pwm);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0, 0, (MAX_PWM_DUTYCYCLE - count_val));
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: fade_white_led
 ********************************************************************************
 * Summary:
 *  Fades LEDs in white color
 *
 * Return:
 * None
 *******************************************************************************/
void fade_white_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(count_pwm, count_pwm, count_pwm);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm((MAX_PWM_DUTYCYCLE - count_val), (MAX_PWM_DUTYCYCLE - count_val), (MAX_PWM_DUTYCYCLE - count_val));
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: fade_orange_led
 ********************************************************************************
 * Summary:
 *  Fades LEDs in orange color
 *
 * Return:
 * None
 *******************************************************************************/
void fade_orange_led(void) // rgb(80%, 40%, 0%)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0.8 * count_pwm, 0.4 * count_pwm, 0);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0.8 * (MAX_PWM_DUTYCYCLE - count_val), 0.4 * (MAX_PWM_DUTYCYCLE - count_val), 0);
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: fade_yellow_led
 ********************************************************************************
 * Summary:
 *  Fades yellow led
 *
 * Return:
 * None
 *******************************************************************************/
void fade_yellow_led(void) // 97%, 46%, 19%
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0.9 * count_pwm, 0.9 * count_pwm, 0 * count_pwm);
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm(0.9 * (MAX_PWM_DUTYCYCLE - count_val), 0.9 * (MAX_PWM_DUTYCYCLE - count_val), 0 * (MAX_PWM_DUTYCYCLE - count_val));
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: fade_pink_led
 ********************************************************************************
 * Summary:
 *  Fades pink led
 *
 * Return:
 * None
 *******************************************************************************/
void fade_pink_led(void)
{
    if (count_pwm <= MAX_PWM_DUTYCYCLE)
    {
        set_pwm(0.8*(count_pwm), 0, (0.8*count_pwm));
    }
    else if (count_pwm > MAX_PWM_DUTYCYCLE)
    {
        uint32_t count_val = (count_pwm - MAX_PWM_DUTYCYCLE);

        if (count_val >= MAX_PWM_DUTYCYCLE)
        {
            set_pwm(0, 0, 0);

            count_pwm = 0;
            fading_counter++;
            return;
        }

        set_pwm((0.8*(MAX_PWM_DUTYCYCLE - count_val)), 0, (0.8*(MAX_PWM_DUTYCYCLE - count_val)));
    }
    count_pwm += PWM_STEP_FADE;
}


/*******************************************************************************
 * Function Name: process_pwm_led
 ********************************************************************************
 * Summary:
 *  Processes event from P6
 *
 * Return:
 * None
 *******************************************************************************/
void process_pwm_led(void)
{
    extern volatile int proximity_status, prev_proximity_status;
    extern volatile int proximity_led_state;

    if(p6_event_input)
    {
        /* if there is a P6 event when proximity is
         * still active, clear all the variables
         * related to proximity active
         */
        if (1 == prev_proximity_status)
        {
            prev_proximity_status = 0;
            proximity_led_state = 0;

            /* LED on time out is over ridden by an event from P6 */
            UART_print_string("LED off due to [P6 event ");
            UART_print_data(p6_event_input);
            UART_print_string("]\r\n");
        }

        /* Note: do not use printf or other delays within this switch cases
         * this affects LED effects timing */
        switch (p6_event_input)
        {
        case 0:
            break;

        case bootup:
            if (fading_counter <= 10 && fading_counter >= 1)
            {
                set_led_color(RED_COLOR);
            }
            else if (fading_counter <= 20 && fading_counter >= 1)
            {
                set_led_color(GREEN_COLOR);
            }
            else if (fading_counter <= 30 && fading_counter >= 1)
            {
                set_led_color(BLUE_COLOR);
            }
            else if (fading_counter > 30)
            {
                lock_state = true; /* Reset local door state to locked */
                fading_counter = 0;
                p6_event_input = 0;

                rgb_led_reset();
            }

            break;

        case pairing_mode:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                fade_blue_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;

                rgb_led_reset();
            }
            break;

        case pairing_started:
            if (fading_counter <= 300 && fading_counter >= 1)
            {
                fade_blue_led();
            }
            else if (fading_counter > 300)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case pairing_done:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                blink_blue_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;

                rgb_led_reset();
            }
            break;

        case pairing_failed:
            switch (fading_counter)
            {
                case 1:
                case 3:
                    blink_blue_led();
                break;

                case 2:
                case 4:
                    blink_red_led();
                break;

                case 5:
                    fading_counter = 0;
                    p6_event_input = 0;
                    rgb_led_reset();
                break;
            }
            break;

        case wifi_connected:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                blink_white_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case matter_provision_success:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                blink_white_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case matter_provision_failed:
            switch (fading_counter)
            {
                case 1:
                case 3:
                    blink_white_led();
                break;

                case 2:
                case 4:
                    blink_red_led();
                break;

                case 5:
                    fading_counter = 0;
                    p6_event_input = 0;
                    rgb_led_reset();
                break;

            }
            break;

        case mqtt_connecting:
            if (fading_counter <= 2 && fading_counter >= 1)
            {
                fade_orange_led();
            }
            else if (fading_counter > 2)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case mqtt_connected:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                blink_orange_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case mqtt_disconnected:
            switch (fading_counter)
            {
                case 1:
                case 3:
                    blink_orange_led();
                break;

                case 2:
                case 4:
                    blink_red_led();
                break;

                case 5:
                    fading_counter = 0;
                    p6_event_input = 0;
                    rgb_led_reset();
                break;

            }
            break;

        case pin_new_configuration:
            if (fading_counter <= 2 && fading_counter >= 1)
            {
                fade_green_led();
            }
            else if (fading_counter >= 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case pin_correct: // this gets superceeded by 7.2
    //        if (fading_counter <= 3 && fading_counter >= 1)
    //        {
    //            blink_green_led();
    //        }
    //        else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
    //            rgb_led_reset();
            }
            break;

        case pin_incorrect:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                blink_red_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case pin_three_times_incorrect:
            if (fading_counter <= 10 && fading_counter >= 1)
            {
                blink_red_led();
            }
            else if (fading_counter > 10)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case pin_entry_disabled:
            if (fading_counter <= 10 && fading_counter >= 1)
            {
                blink_red_led();
            }
            else if (fading_counter > 10)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case nfc_add_new_card:
            if (fading_counter <= 30 && fading_counter >= 1)
            {
                fade_pink_led();
            }
            else if (fading_counter > 30)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case nfc_success:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                blink_green_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case nfc_failed:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                fade_red_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case nfc_provision_card:
            if (fading_counter <= 50 && fading_counter >= 1)
            {
                blink_pink_led();
            }
            else if (fading_counter > 50)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case nfc_block_cards:
            switch (fading_counter)
            {
                case 1:
                case 3:
                    blink_pink_led();
                break;

                case 2:
                case 4:
                    blink_red_led();
                break;

                case 5:
                    fading_counter = 0;
                    p6_event_input = 0;
                    rgb_led_reset();
                break;

            }
            break;

        case lock:
            lock_state = true;

            if (fading_counter <= 2 && fading_counter >= 1)
            {
                blink_red_led();
            }
            else if (fading_counter > 2)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case unlock:
            lock_state = false; /* Unlocked */

            if (fading_counter <= 2 && fading_counter >= 1)
            {
                blink_green_led();
            }
            else if (fading_counter > 2)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case battery_low:
            if (fading_counter <= 2 && fading_counter >= 1)
            {
                dim_blink_red_led();
            }
            else if (fading_counter > 2)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case battery_critical:
            if (fading_counter <= 3 && fading_counter >= 1)
            {
                dim_blink_red_led();
            }
            else if (fading_counter > 3)
            {
                fading_counter = 0;
                p6_event_input = 0;
                rgb_led_reset();
            }
            break;

        case factory_reset:
            switch (fading_counter)
            {
                case 1:
                case 3:
                    blink_green_led();
                break;

                case 2:
                case 4:
                    blink_red_led();
                break;

                case 5:
                    fading_counter = 0;
                    p6_event_input = 0;
                    rgb_led_reset();
                break;

            }

            break;

        default:
            UART_print_string("event on p4 :default pwm case and p6_event_input is");
            UART_print_data(p6_event_input);
            UART_print_string("\r\n");
            p6_event_input = 0;
            break;
        }
    }
    else if(1 == proximity_status)
    {
        /* If the proximity sensor is active */
        /* Turn on Proximity active led only if there is no
         * event from P6, this is to make sure
         * P6 event related RGB led effect is not disturbed
         * by Proximity active led
         */
        proximity_led_state = 1;
        prev_proximity_status = 1;
        capsense_prox_led(PROX_ACTIVE_LED_DUTY_CYCLE);
        /* Clear the LED ON counter value, to be used for Proximity LED timeout counting */
        clear_led_on_counter_value();
    }
    else if (1 == prev_proximity_status)
    {
        /* If the proximity sensor was active before and is not active now */
#if ENABLE_LED
        /* Keep the Proximity led ON till timeout value is reached */
        if(get_led_on_counter_value() >= PROX_LED_ON_TIMEOUT)
#endif
        {
            fading_counter = 1;
            prev_proximity_status = 0;
            /* Clear the LED ON counter value, to be used for Proximity LED fade OFF */
            clear_led_on_counter_value();
        }
    }
    else if(1 == proximity_led_state)
    {
        /* If the proximity LED was ON before and timeout has occurred */
        /* This is to make sure LED off is gradually done.
         * The LED duty cycle is stepped down with every WDT tick.
         * But, logic may not address gradual step down if
         * there is an adhoc event from P6
         */

        int cur_led_on_ctr_val = (PWM_STEP_FADE * get_led_on_counter_value());

        if (cur_led_on_ctr_val <= PROX_ACTIVE_LED_DUTY_CYCLE)
        {
            /* Reduce the LED PWM dutycycle gradually to zero */
            capsense_prox_led((PROX_ACTIVE_LED_DUTY_CYCLE - cur_led_on_ctr_val));
        }
        else
        {
            /* LED is off due to Time out */
            proximity_led_state = 0;
            capsense_prox_led(0);
            UART_print_string("LED off due to TO \r\n");
        }
    }
}


/*******************************************************************************
 * Function Name: capsense_prox_led
 ********************************************************************************
 * Summary:
 *  Controls proximity led
 *
 * Return:
 * None
 *******************************************************************************/
void capsense_prox_led(uint16_t duty_cycle)
{
#if ENABLE_LED
    if (false == lock_state)
    { /* Unlocked */
        set_pwm(0, duty_cycle, 0);
    }
    else
    { /* Locked */
        set_pwm(duty_cycle, 0, 0);
    }
#endif
}


/*******************************************************************************
 * Function Name: pwm_stop
 ********************************************************************************
 * Summary:
 *  Stops PWM  module
 *
 * Return:
 * Status of the action
 *******************************************************************************/
cy_rslt_t pwm_stop(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#if ENABLE_LED
    Set_dutyCycle(&pwm_led_red, 0);
    Set_dutyCycle(&pwm_led_green, 0);
    Set_dutyCycle(&pwm_led_blue, 0);
#endif

    return result;
}


/*******************************************************************************
 * Function Name: pwm_start
 ********************************************************************************
 * Summary:
 *  Starts PWM module
 *
 * Return:
 * Status of the action
 *******************************************************************************/
cy_rslt_t pwm_start(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#ifdef ENABLE_LED
    Set_dutyCycle(&pwm_led_red, 0);
    Set_dutyCycle(&pwm_led_green, 0);
    Set_dutyCycle(&pwm_led_blue, 0);
#endif

    return result;
}

#if ENABLE_LED_TEST
void Led_Blink_Test(void)
{
    /* Toggle the user LED state */

//    UART_print_string("RED LED Test\r\n");
    Cy_GPIO_Set(KEYPADLED_RED_PORT, KEYPADLED_RED_PIN);
    /* Wait for 500ms and Turn OFF LED */
    Cy_SysLib_Delay(500);
    Cy_GPIO_Clr(KEYPADLED_RED_PORT, KEYPADLED_RED_PIN);

//    UART_print_string("GREEN LED Test\r\n");
    Cy_GPIO_Set(KEYPADLED_GREEN_PORT, KEYPADLED_GREEN_PIN);
    /* Wait for 500ms and Turn OFF LED */
    Cy_SysLib_Delay(500);
    Cy_GPIO_Clr(KEYPADLED_GREEN_PORT, KEYPADLED_GREEN_PIN);

//    UART_print_string("BLUE LED Test\r\n\r\n");
    Cy_GPIO_Set(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN);
    /* Wait for 500ms and Turn OFF LED */
    Cy_SysLib_Delay(500);
    Cy_GPIO_Clr(KEYPADLED_BLUE_PORT, KEYPADLED_BLUE_PIN);
}

#endif


/*******************************************************************************
* pwm_callback()
*******************************************************************************/
void pwm_callback(void)
{
    static uint16_t count_time = 0;

    /* Red LED */
    if((pwm_led_red.gpio_status == 1) && (pwm_led_red.dutyCycle != 100))
    {
        if(count_time >= pwm_led_red.led_on_time)
        {
            Cy_GPIO_Clr(pwm_led_red.port, pwm_led_red.pin);

            pwm_led_red.gpio_status = 0;
        }

    }
    else if((pwm_led_red.gpio_status == 0) && (pwm_led_red.dutyCycle != 0))
    {
        if(count_time <= pwm_led_red.led_on_time)
        {
            Cy_GPIO_Set(pwm_led_red.port, pwm_led_red.pin);

            pwm_led_red.gpio_status = 1;
        }
    }

    /* Green LED */
    if((pwm_led_green.gpio_status == 1) && (pwm_led_green.dutyCycle != 100))
    {
        if(count_time >= pwm_led_green.led_on_time)
        {
            Cy_GPIO_Clr(pwm_led_green.port, pwm_led_green.pin);
            pwm_led_green.gpio_status = 0;
        }

    }
    else if((pwm_led_green.gpio_status == 0) && (pwm_led_green.dutyCycle != 0))
    {
        if(count_time <= pwm_led_green.led_on_time)
        {
            Cy_GPIO_Set(pwm_led_green.port, pwm_led_green.pin);
            pwm_led_green.gpio_status = 1;
        }
    }

    /* Blue LED */
    if((pwm_led_blue.gpio_status == 1) && (pwm_led_blue.dutyCycle != 100))
    {
        if(count_time >= pwm_led_blue.led_on_time)
        {
            Cy_GPIO_Clr(pwm_led_blue.port, pwm_led_blue.pin);
            pwm_led_blue.gpio_status = 0;
        }

    }
    else if((pwm_led_blue.gpio_status == 0) && (pwm_led_blue.dutyCycle != 0))
    {
        if(count_time <= pwm_led_blue.led_on_time)
        {
            Cy_GPIO_Set(pwm_led_blue.port, pwm_led_blue.pin);
            pwm_led_blue.gpio_status = 1;
        }
    }

    count_time += SW_PWM_FREQ_US;

    if(count_time == LED_PWM_FREQ_US)
    {
        count_time = 0;
    }

    if ((pwm_led_red.dutyCycle != 0) || (pwm_led_green.dutyCycle != 0) || (pwm_led_blue.dutyCycle != 0))
    {
        pwm_led_active = 1;
        set_wdt_match_value(SW_PWM_FREQ_US);
    }
    else
    {
        count_time = 0;
        pwm_led_active = 0;

        /* Update WDT match value */
        if((capsense_state == ACTIVE_MODE) || (p6_event_input != 0))
        {
            set_wdt_match_value(WDT_TIMEOUT_FAST);
        }
        else
        {
            set_wdt_match_value(WDT_TIMEOUT_SLOW);
        }
    }
}

/* [] END OF FILE */
