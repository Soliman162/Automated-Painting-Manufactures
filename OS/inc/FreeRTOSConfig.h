/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#include "gpio.h"

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			1
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )	
#define configTICK_RATE_HZ			( ( TickType_t ) 500 )
#define configMAX_PRIORITIES		( 8 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 150 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 15 * 1024 ) )
#define configMAX_TASK_NAME_LEN		( 16 )
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1
#define configUSE_MUTEXES			0

#define configCHECK_FOR_STACK_OVERFLOW 	2
#define configUSE_MALLOC_FAILED_HOOK	1

#define configASSERT_DEFINED 0

/* Define configASSERT() to call vAssertCalled() if the assertion fails.  The assertion
has failed if the value of the parameter passed into configASSERT() equals zero. */
// #define configASSERT ( x )     if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

// #define configASSERT( x )    { if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); } }

#define configUSE_APPLICATION_TASK_TAG 			1

#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configUSE_COUNTING_SEMAPHORES       1
#define configSUPPORT_STATIC_ALLOCATION     0

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE 

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		0
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete				0
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			0
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				0

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 		255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /* equivalent to 0xb0, or priority 11. */


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

/*User Defines*/
#define configKEIL_TIMELINE_ANALYSIS	0

#if configKEIL_TIMELINE_ANALYSIS == 1
#define traceTASK_SWITCHED_IN()\
									do\
									{\
										int TaskTag = (int)xTaskGetApplicationTaskTag( NULL \
										if( TaskTag > (int)GPIO_PIN_5 )\
										{\
											HAL_GPIO_WritePin(GPIOC,(uint16_t)TaskTag,GPIO_PIN_SET);\
										}\
										else\
										{\
											HAL_GPIO_WritePin(GPIOA,(uint16_t)TaskTag,GPIO_PIN_SET);\
										}\
									}\
									while(0)		

#define traceTASK_SWITCHED_OUT()\
									do\
									{\
										int TaskTag = (int)xTaskGetApplicationTaskTag( NULL \
										if( TaskTag > (int)GPIO_PIN_5  )\
										{\
											HAL_GPIO_WritePin(GPIOC,(uint16_t)TaskTag,GPIO_PIN_RESET);\
										}\
										else\
										{\
											HAL_GPIO_WritePin(GPIOA,(uint16_t)TaskTag,GPIO_PIN_RESET);\
										}\
									}\
									while(0)	

#endif

#endif /* FREERTOS_CONFIG_H */

