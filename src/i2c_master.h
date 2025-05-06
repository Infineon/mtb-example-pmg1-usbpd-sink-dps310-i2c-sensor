/******************************************************************************
* File Name:  I2CMaster.h
*
* Description:  This file provides constants and parameter values for the I2C
*               Master block which reads from DPS310 I2C Slave temperature and
*               pressure sensor.
*
* Related Document: See Readme.md
*
********************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef SOURCE_I2CMASTER_H_
#define SOURCE_I2CMASTER_H_

#include "cybsp.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define I2C_SUCCESS             (0UL)
#define I2C_FAILURE             (1UL)
#define TRANSFER_CMPLT          (0x00UL)
#define READ_CMPLT              (TRANSFER_CMPLT)

/* Start address of slave buffer */
#define EZI2C_BUFFER_ADDRESS    (0x00)
#define I2C_DPS310_SLAVE_ADDR   (0x77)
#define WRITE_BUFF_SIZE         (2u)

/*******************************************************************************
* Global variables
*******************************************************************************/
typedef struct
{
    uint8_t regAddress;
    uint8_t length;
} RegBlock_t;


extern bool is_sensor_rdy_to_init ;
extern bool is_dps310_initialized;

/******************************************************************************
 * Global function declaration
 ******************************************************************************/
void ReadTemperatureSensor(void);
uint32_t InitI2CMaster(void);
void read_DPS310_tmp_and_psr_data(uint8_t is_tmp);
void ReadDPS310Coefficient(void);
void CYBSP_I2C_Interrupt(void);
void Dps310init(void);
void dps310_read_data (void);
int isDps310InitComplete (void);

/******************************************************************************
 * End of declaration
 ******************************************************************************/
#endif /* SOURCE_I2CMASTER_H_ */
