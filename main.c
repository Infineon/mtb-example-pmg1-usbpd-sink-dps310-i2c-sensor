/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PMG1 USBPD Sink DPS310 I2C Sensor
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "i2c_master.h"

#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "charger_detect.h"
#include "mtbcfg_ezpd.h"
#include <stdio.h>
#include <inttypes.h>

/* CY ASSERT failure */
#define CY_ASSERT_FAILED    (0u)

/* Debug print macro to enable UART print */
#define DEBUG_PRINT    (1u)

cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;

cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

/* Structure for UART Context */
cy_stc_scb_uart_context_t CYBSP_UART_context;

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_PdStackPort0Ctx,
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;

    if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
    {
        cy_stc_pd_packet_extd_t * ext_mes = (cy_stc_pd_packet_extd_t * )data;
        if ((ext_mes->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler (&(gl_TimerCtx));
}

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
    vbus_get_value,
};

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/* The Below section of code implements the De-bounce for the User button press */
#if ENABLE_USER_BUTTON

#define BUTTON_PRESS_DEBOUNCE_TIMER_ID          (0xC1u)
#define BUTTON_PRESS_DEBOUNCE_TIMER_PERIOD      (100u)

#define BUTTON_PRESS_TOTAL_DEBOUNCE_COUNT       (15u)

volatile uint16_t button_press_count = 0;
volatile bool sys_pwr_state = SYSTEM_PWR_STATE_OFF;
volatile bool prev_sys_state = SYSTEM_PWR_STATE_OFF;

void user_button_press_timer_cb (
    cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
    void *callbackContext)       /**< Timer module Context. */
{
    if(0UL == Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM))
    {
        /* increase the button press count */
        button_press_count++;
    }

    /* start the user button press debounce timer*/
    Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, callbackContext, id, BUTTON_PRESS_DEBOUNCE_TIMER_PERIOD,
                    user_button_press_timer_cb);
}

/* The Below section of code handles the GPIO interrupt for User button Press */
void gpio_interrupt_handler(void)
{
    if(1UL == Cy_GPIO_GetInterruptStatus(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM))
    {
        /* Read the input state of P2.0 */
        if(1UL == Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM))
        {
            /* Insert logic for High pin state here */

            /* Check if the user button is pressed longer than the 1.5 second debounce and switch
             * system power state after the button press was released*/
            if(button_press_count > BUTTON_PRESS_TOTAL_DEBOUNCE_COUNT)
            {

                if ( sys_pwr_state == SYSTEM_PWR_STATE_OFF )
                {
                  sys_pwr_state = SYSTEM_PWR_STATE_ON;
                }
                else
                {
                  sys_pwr_state = SYSTEM_PWR_STATE_OFF;
                }

            }
            button_press_count = 0;
            Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, BUTTON_PRESS_DEBOUNCE_TIMER_ID);
        }
        else
        {
            /* Insert logic for Low pin state */
            Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)BUTTON_PRESS_DEBOUNCE_TIMER_ID,
                    BUTTON_PRESS_DEBOUNCE_TIMER_PERIOD, user_button_press_timer_cb);
        }

        /* Clear the P2.0 interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
    }

}
#endif /* ENABLE_USER_BUTTON */


#if ENABLE_DPS310_I2C_INTERFACE

volatile bool temperature_sensor_read_flag = false;
#define TIMER_PERIOD_MSEC                    2000U

/* This function handles the timer interrupt that set the flag for the DPS310 temperature sensor to be read.*/
void timer_interrupt_handler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC );

#if ENABLE_DPS310_I2C_INTERFACE
    temperature_sensor_read_flag = true;
#endif /* ENABLE_DPS310_I2C_INTERFACE */
}

/* This function handles the initialization of the Timer modules used to measure the temperature data periodically */
void init_sensor_timer_module (void)
{
    /* Start the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not. It is not used
     * here for simplicity.
     */
    cy_rslt_t result;

    result = Cy_TCPWM_Counter_Init(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, &CYBSP_TIMER_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
    {
       Cy_TCPWM_SetInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC);
    }

    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1.
     */
    Cy_TCPWM_Counter_SetPeriod(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, TIMER_PERIOD_MSEC-1 );

    /* Trigger a software start on the counter instance. This is required when
     * no other hardware input signal is connected to the component to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(CYBSP_TIMER_HW, CYBSP_TIMER_MASK);

}

/* Below function disables the Timer module */
void deinit_sensor_timer_module(void)
{
    Cy_TCPWM_Counter_Disable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);
}

#endif /* ENABLE_DPS310_I2C_INTERFACE */

int main(void)
{
    cy_rslt_t result;
    cy_en_sysint_status_t intr_result;
    cy_stc_pdutils_timer_config_t timerConfig;
    cy_en_usbpd_status_t usbpd_result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if DEBUG_PRINT

     /* Configure and enable the UART peripheral */
     Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
     Cy_SCB_UART_Enable(CYBSP_UART_HW);

#endif


    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    intr_result = Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {   
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

#if ENABLE_DPS310_I2C_INTERFACE
    /* Enable TCPWM interrupt */
    cy_stc_sysint_t intrCfg =
    {
       /*.intrSrc =*/ CYBSP_TIMER_IRQ, /* Interrupt source is Timer interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };
    intr_result = Cy_SysInt_Init(&intrCfg, timer_interrupt_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {   
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable Timer Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);
#endif /* ENABLE_DPS310_I2C_INTERFACE */

#if ENABLE_USER_BUTTON
    /* Enable GPIO interrupt */
    cy_stc_sysint_t gpioIntrCfg =
    {
       /*.intrSrc =*/ CYBSP_USER_BTN_IRQ, /* Interrupt source is USer Button GPIO interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };
    intr_result = Cy_SysInt_Init(&gpioIntrCfg, gpio_interrupt_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable GPIO Interrupt */
    NVIC_EnableIRQ(gpioIntrCfg.intrSrc);

#endif /* ENABLE_USER_BUTTON */

#if ENABLE_DPS310_I2C_INTERFACE
    /* Initialize I2C master SCB */
    InitI2CMaster();
#endif /* ENABLE_DPS310_I2C_INTERFACE */

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    intr_result = Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    intr_result = Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    usbpd_result = Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    usbpd_result = Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#endif
    if (usbpd_result != CY_USBPD_STAT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);

     /*
     * After the initialization is complete, keep processing the
     * USB-PD device policy manager task in a loop.
     */
    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);

        /* Perform any application level tasks. */
        app_task(&gl_PdStackPort0Ctx);

#if ENABLE_DPS310_I2C_INTERFACE
        /* Check if the System state is moved to ON and Initialize the DPS310 Sensor */
        if((is_sensor_rdy_to_init == true) && (sys_pwr_state == SYSTEM_PWR_STATE_ON))
        {
            Dps310init();
            is_sensor_rdy_to_init= false;

            /* Check if the sensor initialization was complete */
            if(isDps310InitComplete() == 0u)
            {
                is_dps310_initialized = true;
#if DEBUG_PRINT
              /* Print Debug messages over UART */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r DPS310 Sensor Init Complete \0" );
#endif
            }
            else
            {
                is_dps310_initialized = false;
#if DEBUG_PRINT
                /* Print Debug messages over UART */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r DPS310 Sensor Init Failed \0");
#endif
            }
        }

        if ((temperature_sensor_read_flag == true) &&
            (is_dps310_initialized == true) &&
            (sys_pwr_state == SYSTEM_PWR_STATE_ON))
        {
            /* Read the temperature and pressure data */
            dps310_read_data();
            /* Set the read flag to false to wait till timer expires again */
            temperature_sensor_read_flag = false;
        }
#endif /* ENABLE_DPS310_I2C_INTERFACE */

        /* Check the system state change and turn on the sink FET is system
         * moves to ON state. Turn off the Sink FET if system moved to
         * OFF state */
        if(prev_sys_state != sys_pwr_state)
        {
            if((sys_pwr_state == SYSTEM_PWR_STATE_ON) &&
                (gl_PdStackPort0Ctx.dpmConfig.attach == true))
            {
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 0x00);
                sink_fet_on(&gl_PdStackPort0Ctx);

#if DEBUG_PRINT
                /* Print Debug messages over UART */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r System State : ON\0");
#endif

#if ENABLE_DPS310_I2C_INTERFACE
                init_sensor_timer_module();
                is_sensor_rdy_to_init = true;
#endif /* ENABLE_DPS310_I2C_INTERFACE */

            }
            else
            {
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, 0x01);
                sink_fet_off(&gl_PdStackPort0Ctx);

#if DEBUG_PRINT
                /* Print Debug messages over UART */
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r System State : OFF\0");
                /*Delays for 50 milliseconds is applied to make above debug print to display before entering deep sleep.*/
                Cy_SysLib_Delay(50);
#endif

#if ENABLE_DPS310_I2C_INTERFACE
                is_dps310_initialized = false;
                deinit_sensor_timer_module();
#endif /* ENABLE_DPS310_I2C_INTERFACE */

            }
            prev_sys_state = sys_pwr_state;
        }

        /* Perform tasks associated with instrumentation. */
        instrumentation_task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        if(sys_pwr_state == SYSTEM_PWR_STATE_OFF)
        {
            system_sleep(&gl_PdStackPort0Ctx,
                    NULL
                    );
        }
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
