/*******************************************************************************
 * File Name:   i2c.c
 *
 * Description: This file contains I2C related functions.
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
#include "i2c.h"
#include "cy_pdl.h"
#include "cybsp.h"
//#include "cy_retarget_io.h"
#include "rgb_led.h"
#include "main.h"
#include "uart.h"

/*******************************************************************************
* User Configurable Macros
*******************************************************************************/


/*******************************************************************************
* Fixed Macros
*******************************************************************************/
/* I2C read buffer size / Number of bytes expected to be read by I2C Master */
#define Pw_SL_RD_BUFFER_SIZE 1

/* Valid command packet size of four bytes */
#define PACKET_SIZE          (0x04u)

/* Master write buffer of size 4 bytes */
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01u)
#define PACKET_EOP           (0x17u)

/*******************************************************************************
* Global Definitions
*******************************************************************************/
uint8_t TXpassword_i2cReadBuffer[Pw_SL_RD_BUFFER_SIZE] = {0};

/* The instance-specific context structure.It is used by the driver
 * for internal configuration & data keeping for the I2C.
 * Do not modify anything in this structure.
 */
cy_stc_scb_i2c_context_t CYBSP_I2C_context;

#if ENABLE_I2C
const cy_stc_sysint_t CYBSP_I2C_SCB_IRQ_config = {
		.intrSrc = (IRQn_Type) CYBSP_I2C_IRQ,
		.intrPriority = 3u
};
#endif

uint8_t i2c_write_buffer[SL_WR_BUFFER_SIZE];

volatile uint32_t p6_event_input = 0;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void i2c_interrupt_handler(void);
void i2c_slave_callback(uint32_t event);

/*******************************************************************************
 * Function Name: i2c_init
 ********************************************************************************
 * Summary:
 *  Initializes I2C module.
 *
 * Return:
 * Status of initialization
 *******************************************************************************/
cy_rslt_t i2c_init(void)
{
	cy_rslt_t result=CY_SCB_I2C_SUCCESS;
#if ENABLE_I2C
	result = Cy_SCB_I2C_Init(CYBSP_I2C_HW, &CYBSP_I2C_config, &CYBSP_I2C_context);
	if(result != CY_SCB_I2C_SUCCESS)
	{
		CY_ASSERT(0);
	}

	result = Cy_SysInt_Init(&CYBSP_I2C_SCB_IRQ_config, &i2c_interrupt_handler);
	if(result != CY_SYSINT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* Configure write buffer. This is the buffer
	 * that the master writes data to
	 */
	Cy_SCB_I2C_SlaveConfigWriteBuf(CYBSP_I2C_HW, i2c_write_buffer, SL_WR_BUFFER_SIZE, &CYBSP_I2C_context);

	/* Register Callback function for interrupt */
	Cy_SCB_I2C_RegisterEventCallback(CYBSP_I2C_HW,
									(cy_cb_scb_i2c_handle_events_t) i2c_slave_callback,
									&CYBSP_I2C_context);

	/*  Enable interrupt and I2C block */
	NVIC_EnableIRQ((IRQn_Type) CYBSP_I2C_SCB_IRQ_config.intrSrc);

	Cy_SCB_I2C_Enable(CYBSP_I2C_HW, &CYBSP_I2C_context);
#endif

	return result;

}

/*******************************************************************************
 * Function Name: i2c_deinit
 ********************************************************************************
 * Summary:
 *  De-initializes I2C module.
 *
 * Return:
 * Status of initialization
 *******************************************************************************/
void i2c_deinit(void)
{
#if ENABLE_I2C
	/*  Enable interrupt and I2C block */
	NVIC_DisableIRQ((IRQn_Type) CYBSP_I2C_SCB_IRQ_config.intrSrc);

	Cy_SCB_I2C_DeInit(CYBSP_I2C_HW);
#endif
}

/*******************************************************************************
 * Function Name: i2c_interrupt_handler
 ********************************************************************************
 * Summary:
 *  Registers I2C interrupt handler
 *
 * Return:
 * None
 *******************************************************************************/
void i2c_interrupt_handler(void)
{
#if ENABLE_I2C
	/* ISR implementation for I2C */
	Cy_SCB_I2C_SlaveInterrupt(CYBSP_I2C_HW, &CYBSP_I2C_context);
#endif
}

/*******************************************************************************
 * Function Name: i2c_slave_callback
 *******************************************************************************
 * Summary:
 *  Handles slave events write and read completion events.
 *
 * Parameters:
 *  event:  Reports slave events.
 *
 ******************************************************************************/
void i2c_slave_callback(uint32_t event)
{
#if ENABLE_I2C
	/* Check write complete event */
	if (0UL != (CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT & event))
	{
		/* Check for errors */
		if (0UL == (CY_SCB_I2C_SLAVE_ERR_EVENT & event))
		{
			/* Check packet length */
			if (PACKET_SIZE == Cy_SCB_I2C_SlaveGetWriteTransferCount(CYBSP_I2C_HW,
					&CYBSP_I2C_context))
			{
				/* Check start and end of packet markers */
				if ((i2c_write_buffer[PACKET_SOP_POS] == PACKET_SOP) &&
						(i2c_write_buffer[PACKET_EOP_POS] == PACKET_EOP))
				{
					execute_command();
				}
			}
		}

		/* Configure write buffer for the next write */
		Cy_SCB_I2C_SlaveConfigWriteBuf(CYBSP_I2C_HW, i2c_write_buffer,
				SL_WR_BUFFER_SIZE, &CYBSP_I2C_context);
	}

	/* Check read complete event */
	if (0UL != (CY_SCB_I2C_SLAVE_RD_CMPLT_EVENT & event))
	{
		if (0UL == (event & CY_SCB_I2C_SLAVE_ERR_EVENT))
		{
			/* Read complete without errors: update buffer content */
			/* Configure read buffer for the next read */
			Cy_SCB_I2C_SlaveConfigReadBuf(CYBSP_I2C_HW, TXpassword_i2cReadBuffer, Pw_SL_RD_BUFFER_SIZE, &CYBSP_I2C_context);
		}
	}

#endif
}

/*******************************************************************************
 * Function Name: i2c_send_data
 ********************************************************************************
 * Summary:
 *  Sends data over I2C
 *
 * Return:
 * None
 *******************************************************************************/
void i2c_send_data(char data)
{
#if ENABLE_I2C
	TXpassword_i2cReadBuffer[0] = data;

	Cy_SCB_I2C_SlaveConfigReadBuf(CYBSP_I2C_HW, TXpassword_i2cReadBuffer, Pw_SL_RD_BUFFER_SIZE, &CYBSP_I2C_context);
#endif
}

/*******************************************************************************
 * Function Name: get_p6_event
 ********************************************************************************
 * Summary:
 *  Returns active event from P6
 *
 * Return:
 * uint32_t
 *******************************************************************************/
inline uint32_t get_p6_event(void)
{
    return p6_event_input;
}

/*******************************************************************************
 * Function Name: execute_command
 ********************************************************************************
 * Summary:
 *  Processes a command from P6
 *
 * Return:
 * None
 *******************************************************************************/
void execute_command(void)
{
    extern uint16_t fading_counter;

    if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_BOOTUP)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == 1)
        {
            UART_print_string("Event:bootup\n\r");

            p6_event_input = bootup;
        }
        else
        {
            UART_print_string("P6_CMD_BOOTUP [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_BLUETOOTH)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PAIRING_MODE)
        {
            UART_print_string("Event:pairing_Mode [Fade Blue]\n\r");

            if(p6_event_input)
            {
                /* If any P6 event indication is already in progress, skip the pairing mode indication */
                return;
            }

            p6_event_input = pairing_mode;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PAIRING_STARTED)
        {
            UART_print_string("Event:pairing_Started [Fade Blue]\n\r");

            p6_event_input = pairing_started;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PAIRING_DONE)
        {
            UART_print_string("Event:pairing_Done [Blink Blue]\n\r");

            p6_event_input = pairing_done;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PAIRING_FAILED)
        {
            UART_print_string("Event:pairing_Failed [Blink White & Red alternatively]\n\r");

            p6_event_input = pairing_failed;
        }
        else
        {
            UART_print_string("P6_CMD_BLUETOOTH [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_WIFI)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_WIFI_STATUS1)
        {
            UART_print_string("Event:wifi status 1.NOT USED\n\r");

            // p6_event_input=wifi_status_1;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_WIFI_STATUS2)
        {
            UART_print_string("Event:wifi status 2.NOT USED\n\r");

            // p6_event_input=wifi_status_2;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_WIFI_CONNECTED)
        {
            UART_print_string("Event:wifi_Connected [Blink White]\n\r");

            p6_event_input = wifi_connected;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_WIFI_STATUS4)
        {
            UART_print_string("Event:wifi status 4.NOT USED\n\r");

            // p6_event_input=wifi_status_4;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_MATTER_PROV_SUCCESS)
        {
            UART_print_string("Event:matter_Provision_Success [Blink White]\n\r");

            p6_event_input = matter_provision_success;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_MATTER_PROV_FAILED)
        {
            UART_print_string("Event:matter_Provision_Failed [Blink Red]\n\r");

            p6_event_input = matter_provision_failed;
        }
        else
        {
            UART_print_string("P6_CMD_WIFI [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_MQTT)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_MQTT_CONNECTING)
        {
            UART_print_string("Event:mqtt_Connecting [Fade Orange]\n\r");

            p6_event_input = mqtt_connecting;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_MQTT_CONNECTED)
        {
            UART_print_string("Event:mqtt_Connected [Blink Orange]\n\r");

            p6_event_input = mqtt_connected;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_MQTT_DISCONNECTED)
        {
            UART_print_string("Event:mqtt_Disconnected [Blink Red]\n\r");

            p6_event_input = mqtt_disconnected;
        }
        else
        {
            UART_print_string("P6_CMD_MQTT [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_PIN)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_NEW_PIN_CONFIGURATION)
        {
            UART_print_string("Event:pin_New_Configuration [Fade Green]\n\r");

            p6_event_input = pin_new_configuration;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PIN_CORRECT)
        {
            UART_print_string("Event:pin_Correct [Blink Green]\n\r");

            p6_event_input = pin_correct;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PIN_INCORRECT)
        {
            UART_print_string("Event:pin_Incorrect [Blink Red]\n\r");

            p6_event_input = pin_incorrect;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PIN_INCORRECT_3_TIMES)
        {
            UART_print_string("Event:pin_Three_Times_Incorrect [Blink Red]\n\r");

            p6_event_input = pin_three_times_incorrect;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_PIN_ENTRY_DISABLED)
        {
            UART_print_string("Event:pin_Entry_Disabled [Blink Red]\n\r");

            p6_event_input = pin_entry_disabled;
        }
        else
        {
            UART_print_string("P6_CMD_PIN [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_NFC)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_NFC_ADD_NEW_CARD)
        {
            UART_print_string("Event:nfc_Provisioning [Fade Pink]\n\r");

            p6_event_input = nfc_add_new_card;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_NFC_SUCCESS)
        {
            UART_print_string("Event:nfc_Success [Blink Green]\n\r");

            p6_event_input = nfc_success;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_NFC_FAILED)
        {
            UART_print_string("Event:nfc_Failed [Fade Red]\n\r");

            p6_event_input = nfc_failed;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_NFC_PROVISION_CARD)
        {
            UART_print_string("Event:nfc_provision_card [Blink Pink]\n\r");

            p6_event_input = nfc_provision_card;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_NFC_BLOCK_CARDS)
        {
            UART_print_string("Event:nfc_block_cards [Fade Red]\n\r");

            p6_event_input = nfc_block_cards;
        }
        else
        {
            UART_print_string("P6_CMD_NFC [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_SMART_LOCK_STATE)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_LOCK)
        {
            UART_print_string("Event:Lock [Blink Red]\n\r");

            p6_event_input = lock;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_UNLOCK)
        {
            UART_print_string("Event:Unlock [Blink Green]\n\r");

            p6_event_input = unlock;
        }
        else
        {
            UART_print_string("P6_CMD_SMART_LOCK_STATE [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_BATTERY)
    {
        if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_BATTERY_LOW)
        {
            UART_print_string("Event:battery_Low [Blink Dim Red]\n\r");

            p6_event_input = battery_low;
        }
        else if (i2c_write_buffer[PACKET_DAT_POS] == P6_DAT_BATTERY_CRITICAL)
        {
            UART_print_string("Event:battery_Critical [Blink Dim Red]\n\r");

            p6_event_input = battery_critical;
        }
        else
        {
            UART_print_string("P6_CMD_BATTERY [Invalid Status]\n\r");
        }
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_FACTORY_RST)
    {
        UART_print_string("Event:factory_Reset [Blink Green Red Green]\n\r");

        p6_event_input = factory_reset;
    }
    else if (i2c_write_buffer[PACKET_CMD_POS] == P6_CMD_OTA_IN_PROGRESS)
    {
        UART_print_string("Event:ota_in_progress\n\r");

        p6_event_input = ota_in_progress;
    }
    else
    {
        UART_print_string("Invalid Command from P6 [");
        UART_print_data(i2c_write_buffer[1]);
        UART_print_string("] [");
        UART_print_data(i2c_write_buffer[2]);
        UART_print_string("]\r\n");

        return;
    }

    /* Reset fading counter */
    fading_counter = 1;

    /* Clear RGB LED effects and start from a known state */
//    rgb_led_reset();
}


/* [] END OF FILE */
