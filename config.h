/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the PMG1
*              MCU USBPD Sink Example for ModusToolBox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cybsp.h"

/*******************************************************************************
 * Macro declarations
 ******************************************************************************/

/* The ADC which should be used to measure VBus voltage on the Type-C side. */
#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)

/*
 * The Analog-MUX bus input which is used to measure VBus voltage. Choose AMUXBUS_A on PMG1-S2 and AMUXBUS_B on
 * other devices.
 */
#if defined(CY_DEVICE_CCG3)
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_A)
#else
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)
#endif /* defined(CY_DEVICE_CCG3) */

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                       (0u)

/*
 * Activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                            (CY_PDUTILS_TIMER_USER_START_ID)

/*
 * Port-1 activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED2_TIMER_ID                           (CY_PDUTILS_TIMER_USER_START_ID + 1u)

/*
 * The LED toggle period (ms) to be used when Type-C connection hasn't been detected.
 */
#define LED_TIMER_PERIOD_DETACHED               (1000u)

/*
 * The LED toggle period (ms) to be used when a Type-C power source is connected.
 */
#define LED_TIMER_PERIOD_TYPEC_SRC              (500u)

/*
 * The LED toggle period (ms) to be used when a USB-PD power source is connected.
 */
#define LED_TIMER_PERIOD_PD_SRC                 (100u)

/*
 * The LED toggle period (ms) to be used when a BC 1.2 DCP (Downstream Charging Port) source without PD support is connected.
 */
#define LED_TIMER_PERIOD_DCP_SRC                (3000u)

/*
 * The LED toggle period (ms) to be used when an Apple charging source without PD support is connected.
 */
#define LED_TIMER_PERIOD_APPLE_SRC              (5000u)

/*
 * The LED toggle period (ms) to be used when a BC 1.2 CDP (Charging Downstream Port) source without PD support is connected.
 */
#define LED_TIMER_PERIOD_CDP_SRC                (10000u)

/*
 * Enable/Disable firmware supporting User Button Interrupt Service routine &
 * System Power ON/Off Control
 */
#define ENABLE_GPIO_INT_SECTION                 (1u)

#if ENABLE_GPIO_INT_SECTION
#define ENABLE_USER_BUTTON                     (1u)
#endif /* ENABLE_GPIO_INT_SECTION */

#if DEBUG_PRINT
void NumToString(int n, char *buffer);
void FloatToString(float n, char *buffer);
#endif
/*
 * Enable/Disable firmware required to support DPS310 Temperature & Pressure Sensor Interface
 * This macro enables SCB Interface configured as I2C to communicate with sensor, TCPWM configured as counter to monitor
 * sensor data every 2 seconds and the DPS310 library code.
 */
#define ENABLE_DPS310_I2C_INTERFACE             (1u)

#if defined(CY_DEVICE_CCG3PA)
#define PMG1_S0                                 (1u)
#endif

#if ENABLE_USER_BUTTON
/* Debug print macro to enable UART print */
#define SYSTEM_PWR_STATE_OFF                    (0u)
#define SYSTEM_PWR_STATE_ON                     (1u)
#endif /* ENABLE_USER_BUTTON */

#define DEBUG_PRINT                             (1u)

#endif /* _CONFIG_H_ */

/* End of file */
