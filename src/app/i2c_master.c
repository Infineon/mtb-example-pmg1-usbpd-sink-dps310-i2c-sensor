/******************************************************************************
* File Name:  I2CMaster.c
*
* Description:  This file contains all the functions and variables required for
*               proper operation of I2C Master block to read the values
*               from DPS310 I2C Slave temperature and pressure sensor.
*
* Related Document: See Readme.md
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "i2c_master.h"
#include "stdio.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_pdstack_common.h"

#if ENABLE_DPS310_I2C_INTERFACE
/*******************************************************************************
* Macros
*******************************************************************************/
/* I2C master interrupt macros */
#define I2C_INTR_NUM        CYBSP_I2C_IRQ
#define I2C_INTR_PRIORITY   (3UL)

/* I2C transfer delay */
#define CY_SCB_WAIT_DELAY  (100UL)

extern cy_stc_pdutils_sw_timer_t gl_TimerCtx;
extern cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

 /* Timer used to ensure I2C transfers to the MPS complete on time. */
#define SCB_I2C_TIMER_ID                            (0xC3u)
/* The MPS transfer timeout is set to 10 ms timeout period. */
#define SCB_I2C_TIMER_PERIOD                        (10u)

/* MUX access timeout indication. */
static volatile bool i2c_xfer_timeout = false;

/* Timer callback used for I2C transactions to the MUX. */
static void i2c_xfer_timer_cb(
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    /*
     * I2C transmission to MUX continues longer than timeout. Slave doesn't
     * respond.
     */
    i2c_xfer_timeout = true;
}


/*******************************************************************************
* Global variables
*******************************************************************************/
/* Structure for master transfer configuration */
cy_stc_scb_i2c_master_xfer_config_t masterTransferCfg =
{
    .slaveAddress = I2C_DPS310_SLAVE_ADDR,
    .buffer       = NULL,
    .bufferSize   = 0U,
    .xferPending  = false
};

/** The instance-specific context structure.
 * It is used by the driver for internal configuration and
 * data keeping for the I2C. Do not modify anything in this structure.
 */
static cy_stc_scb_i2c_context_t CYBSP_I2C_context;

/*******************************************************************************
* Function Declaration
*******************************************************************************/
void CYBSP_I2C_Interrupt(void);
/*******************************************************************************
* Function Name: CYBSP_I2C_Interrupt
****************************************************************************//**
*
* Summary:
*   Invokes the Cy_SCB_I2C_Interrupt() PDL driver function.
*
*******************************************************************************/
void CYBSP_I2C_Interrupt(void)
{
    Cy_SCB_I2C_Interrupt(CYBSP_I2C_HW, &CYBSP_I2C_context);
}

/*******************************************************************************
* Function Name: InitI2CMaster
********************************************************************************
*
* Summary:
*   This function initializes and enables master SCB
*
* Return:
*   Status of initialization
*
*******************************************************************************/
uint32_t InitI2CMaster(void)
{
    cy_en_scb_i2c_status_t initStatus;
    cy_en_sysint_status_t sysStatus;
    cy_stc_sysint_t CYBSP_I2C_SCB_IRQ_cfg =
    {
            /*.intrSrc =*/ I2C_INTR_NUM,
            /*.intrPriority =*/ I2C_INTR_PRIORITY
    };

    /*Initialize and enable the I2C in master mode*/
    initStatus = Cy_SCB_I2C_Init(CYBSP_I2C_HW, &CYBSP_I2C_config, &CYBSP_I2C_context);
    if(initStatus != CY_SCB_I2C_SUCCESS)
    {
        return I2C_FAILURE;
    }

    /* Hook interrupt service routine */
    sysStatus = Cy_SysInt_Init(&CYBSP_I2C_SCB_IRQ_cfg, &CYBSP_I2C_Interrupt);
    if(sysStatus != CY_SYSINT_SUCCESS)
    {
        return I2C_FAILURE;
    }
    NVIC_EnableIRQ((IRQn_Type) CYBSP_I2C_SCB_IRQ_cfg.intrSrc);
    Cy_SCB_I2C_Enable(CYBSP_I2C_HW, &CYBSP_I2C_context);
    return I2C_SUCCESS;
}

/*******************************************************************************
* Function Name: DPS310_I2CRegRead
********************************************************************************
*
* Summary:
*   This function performs I2C register Reads from the DPS310 Sensor
*
* Parameter:
*     regAddr: DPS310 Register address that needs to be read
*     read_buffer: The read register value will be stored in this read_buffer
*     count: No of bytes that needs to be read from the regAddr
*
* Return:
*   Status of I2C read. Returns I2C_SUCCESS upon successful completion of register
*   read . Otherwise the function returns I2C_FAILURE
*
*******************************************************************************/
uint32_t DPS310_I2CRegRead (uint8_t regAddr, uint8_t *read_buffer , uint8_t count)
{
    cy_en_scb_i2c_status_t status;
    uint32_t masterStatus =0;

    /* Clear the timeout flag and start a timer. */
    i2c_xfer_timeout = false;
    Cy_PdUtils_SwTimer_Start  (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)SCB_I2C_TIMER_ID,
            SCB_I2C_TIMER_PERIOD, i2c_xfer_timer_cb);


    uint8_t dps310_read_buff = regAddr;
    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = &dps310_read_buff;
    masterTransferCfg.bufferSize = 0x01;

    /* Initiate write pointer value transaction */
    status = Cy_SCB_I2C_MasterWrite(CYBSP_I2C_HW, &masterTransferCfg, &CYBSP_I2C_context);
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Wait until master completes write transfer*/
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_HW, &CYBSP_I2C_context);
            //Cy_SysLib_DelayUs(CY_SCB_WAIT_DELAY);

        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (!i2c_xfer_timeout));

        /* Setup transfer specific parameters */
        masterTransferCfg.buffer     = read_buffer;
        masterTransferCfg.bufferSize = count;

        /* Initiate read temperature register transaction */
        status = Cy_SCB_I2C_MasterRead(CYBSP_I2C_HW, &masterTransferCfg, &CYBSP_I2C_context);
        if(status == CY_SCB_I2C_SUCCESS)
        {
            /* Wait until master completes read transfer */
            do
            {
                masterStatus  = Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_HW, &CYBSP_I2C_context);
                //Cy_SysLib_DelayUs(CY_SCB_WAIT_DELAY);

            } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (!i2c_xfer_timeout));

        }
    }

    if((status != CY_SCB_I2C_SUCCESS) || (i2c_xfer_timeout == true) ||
        (0UL != (masterStatus & CY_SCB_I2C_MASTER_ADDR_NAK)))
        return I2C_FAILURE;

    return I2C_SUCCESS;

}

/*******************************************************************************
* Function Name: DPS310_I2CRegWrite
********************************************************************************
*
* Summary:
*   This function performs I2C write to the DPS310 Sensor Registers
*
* Parameter:
*     regAddr: DPS310 Register address that needs to be written
*     write_buffer: Data that needs to written to the register r
*     count: No of bytes that needs to be written to the regAddr
*
* Return:
*   Status of I2C read. Returns I2C_SUCCESS upon successful completion of register
*   read . Otherwise the function returns I2C_FAILURE
*
*******************************************************************************/
uint32_t DPS310_I2CRegWrite (uint8_t regAddr, uint8_t *write_buffer, uint8_t count)
{
    cy_en_scb_i2c_status_t status;
    uint32_t masterStatus =0;

    /* Clear the timeout flag and start a timer. */
    i2c_xfer_timeout = false;
    Cy_PdUtils_SwTimer_Start  (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)SCB_I2C_TIMER_ID,
                SCB_I2C_TIMER_PERIOD, i2c_xfer_timer_cb);


    uint8_t dps310_write_reg[WRITE_BUFF_SIZE] = {0};

    dps310_write_reg[0] = regAddr;

    for(uint8_t i=0 ; i<count ;  i++)
    {
        dps310_write_reg[i+1] = write_buffer[i];
    }

    if(count > WRITE_BUFF_SIZE)
        count = WRITE_BUFF_SIZE;

    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = dps310_write_reg;
    masterTransferCfg.bufferSize = count+1;

    /* Initiate write pointer value transaction */
    status = Cy_SCB_I2C_MasterWrite(CYBSP_I2C_HW, &masterTransferCfg, &CYBSP_I2C_context);
    if(status == CY_SCB_I2C_SUCCESS)
    {
        /* Wait until master completes write transfer*/
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_HW, &CYBSP_I2C_context);
            //Cy_SysLib_DelayUs(CY_SCB_WAIT_DELAY);

        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY))&&  (!i2c_xfer_timeout));
    }

    if((status != CY_SCB_I2C_SUCCESS) || (i2c_xfer_timeout == true) ||
        (0UL != (masterStatus & CY_SCB_I2C_MASTER_ADDR_NAK)))
            return I2C_FAILURE;

    return I2C_SUCCESS;
}

#endif /* ENABLE_DPS310_I2C_INTERFACE  */
