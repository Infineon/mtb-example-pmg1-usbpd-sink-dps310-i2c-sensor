/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USBPD Sink DPS310-I2C-Sensor
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
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


/*******************************************************************************
 * Include header files
 ******************************************************************************/
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
#include "cy_app_instrumentation.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"
#include "cy_app_battery_charging.h"
#include "cy_app_charger_detect.h"
#include "mtbcfg_ezpd.h"
#include "cy_app_fault_handlers.h"
#include "stdio.h"
#include "inttypes.h"

/* Debug print macro to enable UART print */
#define DEBUG_PRINT    (1u)

/*******************************************************************************
* Structure definitions
*******************************************************************************/
/* Structure to hold the user LED status. */
typedef struct
{
    GPIO_PRT_Type* gpioPort;    /* User LED port base address */
    uint32_t gpioPin;           /* User LED pin GPIO number */
    uint16_t blinkRate;         /* User LED blink rate in millisecond */
}cy_stc_user_led_status;

/*******************************************************************************
* Global Variables
*******************************************************************************/
#if ENABLE_DPS310_I2C_INTERFACE
bool is_sensor_rdy_to_init = false;
bool is_dps310_initialized = false;
volatile bool sys_pwr_state = SYSTEM_PWR_STATE_OFF;
#endif /* ENABLE_DPS310_I2C_INTERFACE */

cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;
void NumToString(int n, char *buffer);
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
/* Variable to store the user LED status */
static cy_stc_user_led_status gl_userLedStatus[NO_OF_TYPEC_PORTS] =
{
    {CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, LED_TIMER_PERIOD_DETACHED},
#if PMG1_PD_DUALPORT_ENABLE
    {CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN, LED_TIMER_PERIOD_DETACHED},
#endif /* PMG1_PD_DUALPORT_ENABLE */
};
#endif /* APP_FW_LED_ENABLE */

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
        .dpmSnkWaitCapPeriod = 400,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
        .dpmSnkWaitCapPeriod = 400,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

uint32_t gl_discIdResp[7] = {0xFF00A841, 0x184004B4, 0x00000000, 0xF5000000};

const cy_stc_app_params_t port0_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdResp[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_app_params_t port1_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdResp[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
        &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
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

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: get_pdstack_context
********************************************************************************
* Summary:
*   Returns the respective port PD Stack Context
*
* Parameters:
*  portIdx - Port Index
*
* Return:
*  cy_stc_pdstack_context_t
*
*******************************************************************************/
cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/*******************************************************************************
* Function Name: sln_pd_event_handler
********************************************************************************
* Summary:
*   Solution PD Event Handler
*   Handles the Extended message event
*
* Parameters:
*  ctx - PD Stack Context
*  evt - App Event
*  data - Data
*
* Return:
*  None
*
*******************************************************************************/
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;
    switch (evt)
       {
           case APP_EVT_TYPEC_STARTED:
               Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r TYPEC_STARTED \0");
                break;
           case APP_EVT_CONNECT:
               Cy_SCB_UART_PutString(CYBSP_UART_HW, "\n\r TYPEC_CONNECT \0");
#if ENABLE_DPS310_I2C_INTERFACE
            if(sys_pwr_state == SYSTEM_PWR_STATE_ON)
            {
                is_sensor_rdy_to_init= true;
            }
#endif /* ENABLE_DPS310_I2C_INTERFACE */
               break;
           default:
               break;
       }
    (void)data;
}

/*******************************************************************************
* Function Name: instrumentation_cb
********************************************************************************
* Summary:
*  Callback function for handling instrumentation faults
*
* Parameters:
*  port - Port
*  evt - Event
*
* Return:
*  None
*
*******************************************************************************/
void instrumentation_cb(uint8_t port, uint8_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

/*******************************************************************************
* Function Name: wdt_interrupt_handler
********************************************************************************
* Summary:
*  Interrupt Handler for Watch Dog Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
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

/*******************************************************************************
* Function Name: cy_usbpd0_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: cy_usbpd1_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd1_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
/*******************************************************************************
* Function Name: led_timer_cb
********************************************************************************
* Summary:
*  Sets the desired LED blink rate based on the Type-C connection
*
* Parameters:
*  id - Timer ID
*  callbackContext - Context
*
* Return:
*  None
*
*******************************************************************************/
void led_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
    cy_stc_user_led_status *user_led = &gl_userLedStatus[stack_ctx->port];
#if BATTERY_CHARGING_ENABLE
    const cy_stc_bc_status_t    *bc_stat;
#endif /* #if BATTERY_CHARGING_ENABLE */

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(user_led->gpioPort, user_led->gpioPin);

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.contractExist)
        {
            user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC;
        }
        else
        {
#if BATTERY_CHARGING_ENABLE
            bc_stat = Cy_App_Bc_GetStatus(stack_ctx->ptrUsbPdContext);
            if (bc_stat->bc_fsm_state == BC_FSM_SINK_DCP_CONNECTED)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_DCP_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_CDP_CONNECTED)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_CDP_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_APPLE_BRICK_ID_DETECT)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_APPLE_SRC;
            }
            else
#endif /* BATTERY_CHARGING_ENABLE */
            {
                user_led->blinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
            }
        }
    }
    else
    {
        user_led->blinkRate = LED_TIMER_PERIOD_DETACHED;
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, user_led->blinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

/*******************************************************************************
* Function Name: get_dpm_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 0
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: get_dpm_port1_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 1
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    .app_event_handler = Cy_App_EventHandler,
    .vconn_enable = Cy_App_VconnEnable,
    .vconn_disable = Cy_App_VconnDisable,
    .vconn_is_present = Cy_App_VconnIsPresent,
    .vbus_is_present = Cy_App_VbusIsPresent,
    .vbus_discharge_on = Cy_App_VbusDischargeOn,
    .vbus_discharge_off = Cy_App_VbusDischargeOff,
    .psnk_set_voltage = Cy_App_Sink_SetVoltage,
    .psnk_set_current = Cy_App_Sink_SetCurrent,
    .psnk_enable = Cy_App_Sink_Enable,
    .psnk_disable = Cy_App_Sink_Disable,
    .eval_src_cap = Cy_App_Pdo_EvalSrcCap,
    .eval_dr_swap = Cy_App_Swap_EvalDrSwap,
    .eval_pr_swap = Cy_App_Swap_EvalPrSwap,
    .eval_vconn_swap = Cy_App_Swap_EvalVconnSwap,
    .eval_vdm = Cy_App_Vdm_EvalVdmMsg,
    .vbus_get_value = Cy_App_VbusGetValue
};

/*******************************************************************************
* Function Name: app_get_callback_ptr
********************************************************************************
* Summary:
*  Returns pointer to the structure holding the application callback functions
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  cy_stc_pdstack_app_cbk_t
*
*******************************************************************************/
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*******************************************************************************
* Function Name: soln_sink_fet_off
********************************************************************************
* Summary:
*  Turns off the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_off(cy_stc_pdstack_context_t * context)
{
#if CY_APP_SINK_FET_CTRL_GPIO_EN
    if (context->port == 0u)
    {
        Cy_GPIO_Clr (PFET_SNK_CTRL_P0_PORT, PFET_SNK_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Clr (PFET_SNK_CTRL_P1_PORT, PFET_SNK_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_SINK_FET_CTRL_GPIO_EN */
}

/*******************************************************************************
* Function Name: soln_sink_fet_on
********************************************************************************
* Summary:
*  Turns on the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_on(cy_stc_pdstack_context_t * context)
{
#if CY_APP_SINK_FET_CTRL_GPIO_EN
    if (context->port == 0u)
    {
        Cy_GPIO_Set (PFET_SNK_CTRL_P0_PORT, PFET_SNK_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Set (PFET_SNK_CTRL_P1_PORT, PFET_SNK_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_SINK_FET_CTRL_GPIO_EN */
}

/* The Below section of code implements the De-bounce for the User button press */
#if ENABLE_USER_BUTTON

#define BUTTON_PRESS_DEBOUNCE_TIMER_ID          (0xC1u)
#define BUTTON_PRESS_DEBOUNCE_TIMER_PERIOD      (100u)

#define BUTTON_PRESS_TOTAL_DEBOUNCE_COUNT       (15u)

volatile uint16_t button_press_count = 0;
volatile bool prev_sys_state = SYSTEM_PWR_STATE_OFF;

/*******************************************************************************
* Function Name: user_button_press_timer_cb
********************************************************************************
* Summary:
*  - Start the user button press debounce timer
*
* Parameters:
*  id - Timer ID for which callback is being generated
*  callbackContext - Timer module Context
*
* Return:
*  None
*
*******************************************************************************/
void user_button_press_timer_cb (cy_timer_id_t id, void *callbackContext)
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

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*  - Handles the GPIO interrupt for User button Press
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
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

/*******************************************************************************
* Function Name: timer_interrupt_handler
********************************************************************************
* Summary:
*  - Handles the timer interrupt that set the flag for the DPS310 temperature
*     sensor to be read
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void timer_interrupt_handler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC );

#if ENABLE_DPS310_I2C_INTERFACE
    temperature_sensor_read_flag = true;
#endif /* ENABLE_DPS310_I2C_INTERFACE */
}

/*******************************************************************************
* Function Name: init_sensor_timer_module
********************************************************************************
* Summary:
*  - Handles the initialization of the Timer modules used to measure the temperature
*     data periodically
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
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
        CY_ASSERT(0);
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

/*******************************************************************************
* Function Name: deinit_sensor_timer_module
********************************************************************************
* Summary:
*  - Disables the Timer module
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void deinit_sensor_timer_module(void)
{
    Cy_TCPWM_Counter_Disable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);
}

#endif /* ENABLE_DPS310_I2C_INTERFACE */

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - Initial setup of device
*  - Enables Watchdog timer, USBPD interrupts
*  - Initializes USBPD block and PDStack
*  - Runs device policy tasks and application level tasks
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_sysint_status_t intr_result;
    cy_stc_pdutils_timer_config_t timerConfig;
    cy_stc_scb_uart_context_t CYBSP_UART_context;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#if DEBUG_PRINT
     /* Configure and enable the UART peripheral */
     Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
     Cy_SCB_UART_Enable(CYBSP_UART_HW);
#endif /* DEBUG_PRINT */

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
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
        CY_ASSERT(0);
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
        CY_ASSERT(0);
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
    Cy_App_Instrumentation_Init(&gl_TimerCtx);

    /* Register callback function to be executed when instrumentation fault occurs. */
    Cy_App_Instrumentation_RegisterCb((cy_app_instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    Cy_App_Init(&gl_PdStackPort0Ctx, &port0_app_params);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Init(&gl_PdStackPort1Ctx, &port1_app_params);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    Cy_App_Fault_InitVars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Fault_InitVars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    Cy_App_Instrumentation_Start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
            LED_TIMER_PERIOD_DETACHED, led_timer_cb);
#if PMG1_PD_DUALPORT_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort1Ctx, (cy_timer_id_t)LED2_TIMER_ID,
            LED_TIMER_PERIOD_DETACHED, led_timer_cb);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#endif /* APP_FW_LED_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */
    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform any application level tasks. */
        Cy_App_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_App_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

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
                soln_sink_fet_on(&gl_PdStackPort0Ctx);

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
                soln_sink_fet_off(&gl_PdStackPort0Ctx);

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
        Cy_App_Instrumentation_Task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        Cy_App_SystemSleep(&gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
