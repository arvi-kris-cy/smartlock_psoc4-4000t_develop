/******************************************************************************
* File Name: i2c.h
*
* Description: This file contains all the function prototypes of I2C
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
#ifndef INCLUDE_I2C_H_
#define INCLUDE_I2C_H_

/*******************************************************************************
* Include header files
*******************************************************************************/
#include "cy_result.h"
#include "main.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/
/* P6 sends a 4-byte packet towards P4 to indicate any event
 * The structure of the packet looks as below
 */
#define PACKET_SOP_POS           (0x00u)    // Start of the packet
#define PACKET_CMD_POS           (0x01u)    // Command
#define PACKET_DAT_POS           (0x02u)    // Data
#define PACKET_EOP_POS           (0x03u)    // End of the packet

/*******************************************************************************
*         Function Prototypes
*******************************************************************************/
cy_rslt_t i2c_init(void);
void i2c_send_data(char data);
void i2c_deinit(void);

uint32_t get_p6_event(void);
void execute_command(void);

#endif /* INCLUDE_I2C_H_ */

/* [] END OF FILE */
