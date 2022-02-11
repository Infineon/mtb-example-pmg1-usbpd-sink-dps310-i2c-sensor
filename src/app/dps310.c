/******************************************************************************
* File Name:  I2CMaster.c
*
* Description:  This file contains all the functions and variables required for
*               proper operation of DPS310 I2C Slave temperature and pressure
*               sensor.
*
* Related Document: See Readme.md
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
*******************************************************************************/

/* Header file includes */
#include "dps310.h"
#include "config.h"
#include "cyhal_uart.h"

#if ENABLE_DPS310_I2C_INTERFACE
/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global variables
*******************************************************************************/
dps310_param myDPS310 =
{
    .m_slaveAddress     = I2C_DPS310_SLAVE_ADDR
};


static int16_t Dps310getSingleResult(float *result, uint8_t m_opMode);

const int32_t Dps310_scaling_facts[DPS__NUM_OF_SCAL_FACTS] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


/* The Below section of code read and write of DPS310 register field */
int8_t Dps310readByteBitfield(RegMask_t regMask)
{
    uint8_t buff;
    uint32_t ret = DPS310_I2CRegRead (regMask.regAddress, &buff , 0x01);
    if (ret > 0)
    {
        return DPS__FAIL_UNKNOWN;
    }
    return (((uint8_t)buff) & regMask.mask) >> regMask.shift;
}

int8_t Dps310writeByteBitfield(uint8_t data, RegMask_t regMask)
{
    uint8_t old_val;

    uint32_t ret = DPS310_I2CRegRead (regMask.regAddress, &old_val , 0x01);
    if (ret > 0)
    {
        return DPS__FAIL_UNKNOWN;
    }

    uint8_t buff = ((uint8_t)old_val & ~regMask.mask) | ((data << regMask.shift) & regMask.mask);
    ret = DPS310_I2CRegWrite (regMask.regAddress, &buff, 0x01);
    if (ret > 0)
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

void Dps310getTwosComplement(int32_t *raw, uint8_t length)
{
    if (*raw & ((uint32_t)1 << (length - 1)))
    {
        *raw -= (uint32_t)1 << length;
    }
}

/* The Below section of code implements DPS310 Co-efficient register read and its calculation */
int16_t Dps310readcoeffs(void)
{
    uint8_t retry_count = 3;
    uint8_t buffer[18];
    //read COEF registers to buffer
    while (DPS310_I2CRegRead(coeffBlock.regAddress, buffer, coeffBlock.length) != I2C_SUCCESS )
    {
        retry_count--;
        if(retry_count == 0)
        {
            return DPS__FAIL_UNKNOWN;
        }
    }

    //compose coefficients from buffer content
    myDPS310.m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    Dps310getTwosComplement(&myDPS310.m_c0Half, 12);
    //c0 is only used as c0*0.5, so c0_half is calculated immediately
    myDPS310.m_c0Half = myDPS310.m_c0Half / 2U;

    //now do the same thing for all other coefficients
    myDPS310.m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    Dps310getTwosComplement(&myDPS310.m_c1, 12);
    myDPS310.m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    Dps310getTwosComplement(&myDPS310.m_c00, 20);
    myDPS310.m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    Dps310getTwosComplement(&myDPS310.m_c10, 20);

    myDPS310.m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    Dps310getTwosComplement(&myDPS310.m_c01, 16);

    myDPS310.m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    Dps310getTwosComplement(&myDPS310.m_c11, 16);
    myDPS310.m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    Dps310getTwosComplement(&myDPS310.m_c20, 16);
    myDPS310.m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    Dps310getTwosComplement(&myDPS310.m_c21, 16);
    myDPS310.m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    Dps310getTwosComplement(&myDPS310.m_c30, 16);

    return DPS__SUCCEEDED;
}

void Dps310correctTemp(void)
{
    uint8_t efuse_correction[5][2] ={
                                        {0x0E, 0xA5 },
                                        {0x0F, 0x96 },
                                        {0x62, 0x02 },
                                        {0x0E, 0x00 },
                                        {0x0F, 0x00 },    };

    uint8_t regAddr, data;

    for(int i =0 ; i < 5 ; i++ )
    {
        regAddr = efuse_correction[i][0];
        data = efuse_correction[i][1];
        DPS310_I2CRegWrite (regAddr, &data, 0x01);
    }

    float temperature;
    Dps310getSingleResult(&temperature, CMD_TEMP);

}

int16_t Dps310setOpMode(uint8_t opMode)
{
    if (Dps310writeByteBitfield(opMode, config_registers[MSR_CTRL]) == -1)
    {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

/* The Below section of code configures the temperature and pressure sensor data
 * over sampling rate */

int16_t Dps310configTemp(uint8_t tempMr, uint8_t tempOsr)
{
    tempMr &= 0x07;
    tempOsr &= 0x07;
    // two accesses to the same register; for readability
    int8_t ret = Dps310writeByteBitfield(tempMr, config_registers[TEMP_MR]);
    ret = Dps310writeByteBitfield(tempOsr, config_registers[TEMP_OSR]);

    //abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    myDPS310.m_tempMr = tempMr;
    myDPS310.m_tempOsr = tempOsr;

    Dps310writeByteBitfield(myDPS310.m_tempSensor, registers[TEMP_SENSOR]);

    //set TEMP SHIFT ENABLE if oversampling rate higher than eight(2^3)
    if (tempOsr > DPS310__OSR_SE)
    {
        ret = Dps310writeByteBitfield(1U, registers[TEMP_SE]);
    }
    else
    {
        ret = Dps310writeByteBitfield(0U, registers[TEMP_SE]);
    }

    return ret;
}

int16_t Dps310configPressure(uint8_t prsMr, uint8_t prsOsr)
{
    prsMr &= 0x07;
    prsOsr &= 0x07;
    int8_t ret = Dps310writeByteBitfield(prsMr, config_registers[PRS_MR]);
    ret = Dps310writeByteBitfield(prsOsr, config_registers[PRS_OSR]);

    //abort immediately on fail
    if (ret != DPS__SUCCEEDED)
    {
        return DPS__FAIL_UNKNOWN;
    }
    myDPS310.m_prsMr = prsMr;
    myDPS310.m_prsOsr = prsOsr;

    //set PM SHIFT ENABLE if oversampling rate higher than eight(2^3)
    if (prsOsr > DPS310__OSR_SE)
    {
        ret = Dps310writeByteBitfield(1U, registers[PRS_SE]);
    }
    else
    {
        ret = Dps310writeByteBitfield(0U, registers[PRS_SE]);
    }
    return ret;
}

/* The Below section of code implements DPS310 initialization */
void Dps310init(void)
{
    myDPS310.m_initFail = 0u;
    int16_t prodId = Dps310readByteBitfield(registers[PROD_ID]);
    if (prodId < 0)
    {
        //Connected device is not a Dps310
        myDPS310.m_initFail = 1U;
        return;
    }
    myDPS310.m_productID = prodId;

    int16_t revId = Dps310readByteBitfield(registers[REV_ID]);
    if (revId < 0)
    {
        myDPS310.m_initFail = 1U;
        return;
    }
    myDPS310.m_revisionID = revId;

    //find out which temperature sensor is calibrated with coefficients...
    int16_t sensor = Dps310readByteBitfield(registers[TEMP_SENSORREC]);
    if (sensor < 0)
    {
        myDPS310.m_initFail = 1U;
        return;
    }

    //...and use this sensor for temperature measurement
    myDPS310.m_tempSensor = sensor;
    if (Dps310writeByteBitfield((uint8_t)sensor, registers[TEMP_SENSOR]) < 0)
    {
        myDPS310.m_initFail = 1U;
        return;
    }

    int8_t sensor_rdy = Dps310readByteBitfield(config_registers[COEF_RDY]);
    if (sensor_rdy < 0)
    {
        myDPS310.m_initFail = 1U;
        return;
    }

    //read coefficients
    if (Dps310readcoeffs() < 0)
    {
        myDPS310.m_initFail = 1U;
        return;
    }

    //set measurement precision and rate to standard values;
    Dps310configTemp(DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8);
    Dps310configPressure(DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8);

    Dps310setOpMode(CONT_BOTH);

    Dps310correctTemp();
}

int isDps310InitComplete (void)
{
    if (myDPS310.m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }

    return DPS__SUCCEEDED;
}

/* The Below section of code implements the temperature and pressure calculation using the raw
 * register data and the temperature/ pressure co-efficient
 */
float Dps310calcTemp(int32_t raw)
{
    float temp = raw;
    //scale temperature according to scaling table and oversampling
    temp /= Dps310_scaling_facts[myDPS310.m_tempOsr];

    //update last measured temperature
    //it will be used for pressure compensation
    myDPS310.m_lastTempScal = temp;

    //Calculate compensated temperature
    temp = myDPS310.m_c0Half + myDPS310.m_c1 * temp;

    return temp;
}

float Dps310calcPressure(int32_t raw)
{
    float prs = raw;
    //scale pressure according to scaling table and oversampling
    prs /= Dps310_scaling_facts[myDPS310.m_prsOsr];

    //Calculate compensated pressure
    prs = myDPS310.m_c00 + prs * (myDPS310.m_c10 + prs * (myDPS310.m_c20 + prs * myDPS310.m_c30)) +
            myDPS310.m_lastTempScal * (myDPS310.m_c01 + prs * (myDPS310.m_c11 + prs * myDPS310.m_c21));

    prs *= 0.01;
    //return pressure
    return prs;
}

int16_t Dps310getRawResult(int32_t *raw, RegBlock_t reg)
{
    uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};

    if( DPS310_I2CRegRead(reg.regAddress, buffer, reg.length) == I2C_SUCCESS )
    {
        *raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
        Dps310getTwosComplement(raw, 24);
        return DPS__SUCCEEDED;
    }
    return DPS__FAIL_UNKNOWN;
}

/* The Below section of code implements reading the temperature
 * and pressure data from DPS310 .
 */
static int16_t Dps310getSingleResult(float *result, uint8_t m_opMode)
{
    int16_t rdy;
    int32_t raw_val;
    //abort if initialization failed
    if (myDPS310.m_initFail)
    {
        return DPS__FAIL_INIT_FAILED;
    }

    switch (m_opMode)
    {
    case CMD_TEMP: //temperature
        rdy = Dps310readByteBitfield(config_registers[TEMP_RDY]);
        break;
    case CMD_PRS: //pressure
        rdy = Dps310readByteBitfield(config_registers[PRS_RDY]);
        break;
    default: //DPS310 not in command mode
        return DPS__FAIL_TOOBUSY;
    }
    //read new measurement result
    if (rdy == 0x00)
        return DPS__FAIL_TOOBUSY;

    Cy_SysLib_DelayUs(1000);

    switch (m_opMode)
    {
    case CMD_TEMP: //temperature
        if( Dps310getRawResult(&raw_val, registerBlocks[TEMP]) == DPS__SUCCEEDED)
        {
            *result = Dps310calcTemp(raw_val);
            return DPS__SUCCEEDED; 
        }
        return DPS__FAIL_UNKNOWN;
    default:     // Handles the case of CMD_PRS (pressure)
        if( Dps310getRawResult(&raw_val, registerBlocks[PRS]) == DPS__SUCCEEDED)
        {
            *result = Dps310calcPressure(raw_val);
            return DPS__SUCCEEDED; 
        }
        return DPS__FAIL_UNKNOWN;    
    }
}


void dps310_read_data (void)
{
    float temperature;
    float pressure;
    char str[10];
    static uint16_t temp_err_cnt =0, prs_err_cnt =0;

    if(Dps310getSingleResult(&temperature, CMD_TEMP) < DPS__SUCCEEDED )
    {
        temp_err_cnt++;
        if(temp_err_cnt > 3)
        {
            /* Print error message only if the I2C read fails more than 3 times */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r I2C Connection Failed \0");
            temp_err_cnt = 0;
        }
        return;
    }
#if DEBUG_UART_ENABLE
    /* Print the temperature value over UART */
    print_my_debug_messages("\n\r Temperature: ");
    FloatToString(temperature, str);
    print_my_debug_messages(str);
    print_my_debug_messages(" C \0");
#endif

    if(Dps310getSingleResult(&pressure, CMD_PRS) < DPS__SUCCEEDED )
    {
        prs_err_cnt++;
        if(prs_err_cnt >3)
        {
            /* Print error message only if the I2C read fails more than 3 times */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r I2C connection Failed \0");
            prs_err_cnt = 0;
        }
        return;
    }
#if DEBUG_UART_ENABLE
     /* Print the Pressure value over UART */
    print_my_debug_messages("\n\r Pressure: ");
    FloatToString(pressure, str);
    print_my_debug_messages(str);
    print_my_debug_messages(" hPa \0");
#endif
}


#endif /* ENABLE_DPS310_I2C_INTERFACE */
