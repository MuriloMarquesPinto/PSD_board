/**
  MAIN MANAGER Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    main manager.c

  @Summary
    This is the generated driver implementation file for the MAIN MANAGER driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for MAIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75
        Device            :  PIC32MM0032GPL028
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB 	          :  MPLAB X v5.05
 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

/**
  Section: Included Files
 */
#include "mcc_generated_files/system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mcc_generated_files/adc1.h"
#include "mcc_generated_files/pin_manager.h"

xSemaphoreHandle xSemaphore;

static void in(void *pvParameters) {
    int i;
    unsigned int e = 0, x = 0;
    for (;;) {
        if (xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
            ADC1_ChannelSelect(ADC1_CHANNEL_AN0);
            ADC1_Start();
            //Provide Delay
            for (i = 0; i < 1000; i++) {
            }
            ADC1_Stop();
            while (!ADC1_IsConversionComplete()) {
            }
            x = ADC1_ConversionResultGet();
            xSemaphoreGive(xSemaphore);
        }
            if (e != x) {
                e = x;
            } else {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        
        if (e <= 306) {
            LED0PWM_SetLow();
            LED1PWM_SetLow();
            LED2PWM_SetLow();
            LED3PWM_SetLow();
            LED4PWM_SetLow();
            LED5PWM_SetLow();

        }

        if (e >= 307 && e <= 436) {
            LED0PWM_SetHigh();
            LED1PWM_SetLow();
            LED2PWM_SetLow();
            LED3PWM_SetLow();
            LED4PWM_SetLow();
            LED5PWM_SetLow();
        }
        if (e >= 437 && e <= 490) {
            LED0PWM_SetHigh();
            LED1PWM_SetHigh();
            LED2PWM_SetLow();
            LED3PWM_SetLow();
            LED4PWM_SetLow();
            LED5PWM_SetLow();
        }
        if (e >= 491 && e <= 535) {
            LED0PWM_SetHigh();
            LED1PWM_SetHigh();
            LED2PWM_SetHigh();
            LED3PWM_SetLow();
            LED4PWM_SetLow();
            LED5PWM_SetLow();
        }
        if (e >= 536 && e <= 589) {
            LED0PWM_SetHigh();
            LED1PWM_SetHigh();
            LED2PWM_SetHigh();
            LED3PWM_SetHigh();
            LED4PWM_SetLow();
            LED5PWM_SetLow();
        }
        if (e >= 590 && e <= 718) {
            LED0PWM_SetHigh();
            LED1PWM_SetHigh();
            LED2PWM_SetHigh();
            LED3PWM_SetHigh();
            LED4PWM_SetHigh();
            LED5PWM_SetLow();
        }
        if (e >= 719) {
            LED0PWM_SetHigh();
            LED1PWM_SetHigh();
            LED2PWM_SetHigh();
            LED3PWM_SetHigh();
            LED4PWM_SetHigh();
            LED5PWM_SetHigh();
        }
    }
}

static void out(void *pvParameters) {
    int i;
    unsigned int c = 0, x = 0;
    for (;;) {
        if (xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
            ADC1_ChannelSelect(ADC1_CHANNEL_AN1);

            ADC1_Start();
            //Provide Delay
            for (i = 0; i < 1000; i++) {
            }
            ADC1_Stop();
            while (!ADC1_IsConversionComplete()) {
            }
            x = ADC1_ConversionResultGet();
            xSemaphoreGive(xSemaphore);
        }
            if (c != x) {
                c = x;
            } else {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            
        if (c <= 306) {
            LED0OUT_SetLow();
            LED1OUT_SetLow();
            LED2OUT_SetLow();
            LED3OUT_SetLow();
            LED4OUT_SetLow();
            LED5OUT_SetLow();
        }

        if (c >= 307 && c <= 436) {
            LED0OUT_SetHigh();
            LED1OUT_SetLow();
            LED2OUT_SetLow();
            LED3OUT_SetLow();
            LED4OUT_SetLow();
            LED5OUT_SetLow();
        }
        if (c >= 437 && c <= 490) {
            LED0OUT_SetHigh();
            LED1OUT_SetHigh();
            LED2OUT_SetLow();
            LED3OUT_SetLow();
            LED4OUT_SetLow();
            LED5OUT_SetLow();
        }
        if (c >= 491 && c <= 535) {
            LED0OUT_SetHigh();
            LED1OUT_SetHigh();
            LED2OUT_SetHigh();
            LED3OUT_SetLow();
            LED4OUT_SetLow();
            LED5OUT_SetLow();
        }
        if (c >= 536 && c <= 589) {
            LED0OUT_SetHigh();
            LED1OUT_SetHigh();
            LED2OUT_SetHigh();
            LED3OUT_SetHigh();
            LED4OUT_SetLow();
            LED5OUT_SetLow();
        }
        if (c >= 590 && c <= 718) {
            LED0OUT_SetHigh();
            LED1OUT_SetHigh();
            LED2OUT_SetHigh();
            LED3OUT_SetHigh();
            LED4OUT_SetHigh();
            LED5OUT_SetLow();
        }
        if (c >= 719) {
            LED0OUT_SetHigh();
            LED1OUT_SetHigh();
            LED2OUT_SetHigh();
            LED3OUT_SetHigh();
            LED4OUT_SetHigh();
            LED5OUT_SetHigh();
        }
    }
}

static void ref(void *pvParameters) {
    int i;
    unsigned int r = 0, x = 0;
    for (;;) {
        if (xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
            ADC1_ChannelSelect(ADC1_CHANNEL_AN2);

            ADC1_Start();
            //Provide Delay
            for (i = 0; i < 1000; i++) {
            }
            ADC1_Stop();
            while (!ADC1_IsConversionComplete()) {
            }
            x = ADC1_ConversionResultGet();
            xSemaphoreGive(xSemaphore);
        }
            if (r != x) {
                r = x;
            } else {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            
        if (r <= 306) {
            LED0REF_SetLow();
            LED1REF_SetLow();
            LED2REF_SetLow();
            LED3REF_SetLow();
            LED4REF_SetLow();
            LED5REF_SetLow();
        }
        if (r >= 307 && r <= 436) {
            LED0REF_SetHigh();
            LED1REF_SetLow();
            LED2REF_SetLow();
            LED3REF_SetLow();
            LED4REF_SetLow();
            LED5REF_SetLow();
        }
        if (r >= 437 && r <= 490) {
            LED0REF_SetHigh();
            LED1REF_SetHigh();
            LED2REF_SetLow();
            LED3REF_SetLow();
            LED4REF_SetLow();
            LED5REF_SetLow();
        }
        if (r >= 491 && r <= 535) {
            LED0REF_SetHigh();
            LED1REF_SetHigh();
            LED2REF_SetHigh();
            LED3REF_SetLow();
            LED4REF_SetLow();
            LED5REF_SetLow();
        }
        if (r >= 536 && r <= 589) {
            LED0REF_SetHigh();
            LED1REF_SetHigh();
            LED2REF_SetHigh();
            LED3REF_SetHigh();
            LED4REF_SetLow();
            LED5REF_SetLow();
        }
        if (r >= 590 && r <= 718) {
            LED0REF_SetHigh();
            LED1REF_SetHigh();
            LED2REF_SetHigh();
            LED3REF_SetHigh();
            LED4REF_SetHigh();
            LED5REF_SetLow();
        }
        if (r >= 719) {
            LED0REF_SetHigh();
            LED1REF_SetHigh();
            LED2REF_SetHigh();
            LED3REF_SetHigh();
            LED4REF_SetHigh();
            LED5REF_SetHigh();
        }
    }
}

void main(void) {
    SYSTEM_Initialize();

    xSemaphore = xSemaphoreCreateMutex();

    xTaskCreate(in, /* The function that implements the task. */
            "Entrada", /* The text name assigned to the task - for debug only. */
            configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
            NULL, /* The parameter passed to the task - just to check functionality. */
            tskIDLE_PRIORITY + 1, /* The priority assigned to the task. */
            NULL);

    xTaskCreate(ref, /* The function that implements the task. */
            "Referencia", /* The text name assigned to the task - for debug only. */
            configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
            NULL, /* The parameter passed to the task - just to check functionality. */
            tskIDLE_PRIORITY + 1, /* The priority assigned to the task. */
            NULL);

    xTaskCreate(out, /* The function that implements the task. */
            "Saida", /* The text name assigned to the task - for debug only. */
            configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
            NULL, /* The parameter passed to the task - just to check functionality. */
            tskIDLE_PRIORITY + 1, /* The priority assigned to the task. */
            NULL);


    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vAssertCalled(const char * pcFile, unsigned long ulLine) {
    volatile unsigned long ul = 0;
    (void) pcFile;
    (void) ulLine;
    __asm volatile( "di");
    {
        /* Set ul to a non-zero value using the debugger to step out of this
        function. */
        while (ul == 0) {
            portNOP();
        }
    }
    __asm volatile( "ei");
}
/**
 End of File
 */

