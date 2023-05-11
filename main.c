/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the DC Monitoring Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define NO_OF_DC_SAMPLES                    (20)


/*******************************************************************************
* Global Variables
*******************************************************************************/
static uint16_t batmon_samples[NO_OF_DC_SAMPLES];
static uint8_t dc_sample_cnt = 0;
static uint32_t batmon_dc_avg = 0;
static uint8_t batmon_cplt = 1;
static uint16_t batt_level_mv;

/* ADCMIC interrupt configuration parameters */
const cy_stc_sysint_t ADCMIC_IRQ_cfg = {
    .intrSrc = (IRQn_Type)adcmic_0_IRQ,
    .intrPriority = 7
};


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
 * Function Name: adcmic_dc_intr_handler
********************************************************************************
 * Summary:
 *  ADC DC Measurement ISR handler. On Every ISR, one DC sample is read and
 *  stored in an array.
 *
 * Parameters: 
 *  void
 *
 * Return:
 *  static
 *
 ******************************************************************************/
static void adcmic_dc_intr_handler(void)
{
    uint32_t intr_status;

    /* Disable the timer before reading the DC sample */
    Cy_ADCMic_DisableTimer(adcmic_0_HW);

    /* Clear the ADC timer interrupt */
    intr_status = Cy_ADCMic_GetInterruptStatusMasked(adcmic_0_HW);
    Cy_ADCMic_ClearInterrupt(adcmic_0_HW, intr_status);

    /* Read the DC sample */
    batmon_samples[dc_sample_cnt] = Cy_ADCMic_GetDcResult(adcmic_0_HW);
    dc_sample_cnt++;

    /* Stop measurement on receiving the number of defined ADC samples */
    if(dc_sample_cnt >= NO_OF_DC_SAMPLES)
    {
        dc_sample_cnt = 0;

        /* Stop the conversion */
        Cy_ADCMic_Disable(adcmic_0_HW);
        Cy_ADCMic_SetInterruptMask(adcmic_0_HW, 0);
        batmon_cplt  = 0;
    }
    else
    {
        /* Enable ADC timer interrupt to read the next DC sample */
        Cy_ADCMic_EnableTimer(adcmic_0_HW);
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It demonstrates the 
*    1. ADC initialization
*    2. ADC values are displayed in mV
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****************************** "
            "DC Monitoring example started "
            "***************************** \r\n\n");

    /* Register the interrupt handler of ADC IRQ*/
    Cy_SysInt_Init(&ADCMIC_IRQ_cfg,adcmic_dc_intr_handler);
    NVIC_ClearPendingIRQ(ADCMIC_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(ADCMIC_IRQ_cfg.intrSrc);

    /* Initialize the ADCMic for DC monitoring mode */
    Cy_ADCMic_Init(adcmic_0_HW, &adcmic_0_config, CY_ADCMIC_DC);

    while (1)
    {
        /* Monitoring for every one second */
        Cy_SysLib_Delay(1000);

        Cy_ADCMic_SetInterruptMask(adcmic_0_HW, CY_ADCMIC_INTR_DC);
        Cy_ADCMic_ClearInterrupt(adcmic_0_HW, CY_ADCMIC_INTR);

        /* Enable the DC monitoring */
        Cy_ADCMic_Enable(adcmic_0_HW);

        /* Enable ADC timer to generate interrupt for reading the sample */
        Cy_ADCMic_EnableTimer(adcmic_0_HW);

        /* Wait for DC monitoring samples to complete */
        while(batmon_cplt);

        for (int i = 0; i < NO_OF_DC_SAMPLES; i++)
        {
            batmon_dc_avg = batmon_dc_avg + batmon_samples[i];
        }

        batmon_dc_avg = batmon_dc_avg/NO_OF_DC_SAMPLES;

        /* Convert the ADC code in millivolts */
        batt_level_mv = Cy_ADCMic_CountsTo_mVolts((uint16_t)batmon_dc_avg, adcmic_0_config.dcConfig->context );

        printf("batt_level_mv = %d \r\n",batt_level_mv);

        /* Reset for the next set of readings */
        batmon_cplt = 1;
        batmon_dc_avg = 0;
    }
}


/* [] END OF FILE */
