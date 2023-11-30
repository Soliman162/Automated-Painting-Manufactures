/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Ahmed Elsayed
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/*******************************************************************************************************/
/*                                         Includes                                                    */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*******************************************************************************************************/
/*                                         FreeRtos                                                    */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
/*******************************************************************************************************/
/*                                               HW                                                    */
#include "main.h"
#include "usart.h"
#include "gpio.h"
/*******************************************************************************************************/
#include "stepper_interface.h"
#include "DC_interface.h"
#include "IR_inferared_interface.h"
#include "Valve_interface.h"
#include "PUMP_interface.h"
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                                                                         						 */
#define BIT_0	              ( 1 << 0 )
#define BIT_1	              ( 1 << 1 )  
#define BIT_2	              ( 1 << 2 )
#define BIT_3	              ( 1 << 3 )
#define BIT_4	              ( 1 << 4 )
#define BIT_5	              ( 1 << 5 )

#define FILLING_FLAG          BIT_1
#define CAPPING_FLAG          BIT_2
#define LINE1_STEEPER         BIT_3
#define FILLING_BOTTEL_FLAG   BIT_4
#define CAPPING_BOTTEL_FLAG   BIT_5
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                       HW Configrations                                  						 */
// 14 15 uart1 A9 A10 uart2 A3 A2 
#define IR_PORT               GPIOA
#define PUMP_PORT             GPIOA
#define CAPPING_MOTOR_PORT    GPIOA
#define CAPPING_VALVE_PORT    GPIOA

#define CAPPING_MOTOR_CW_PIN    GPIO_PIN_14
#define CAPPING_MOTOR_CCW_PIN   GPIO_PIN_15

#define Filling_Pump_PIN      GPIO_PIN_7

#define CAPPING_VALVE_PIN     GPIO_PIN_8
#define FEEDING_IR_PIN        GPIO_PIN_11
#define CAPPING_IR_PIN        GPIO_PIN_12

#define FILLING_IR_PIN        GPIO_PIN_6

#define STEPPER_PORT          GPIOB

#define LINE1_STEPPER_PIN0     GPIO_PIN_0
#define LINE1_STEPPER_PIN1     GPIO_PIN_1
#define LINE1_STEPPER_PIN2     GPIO_PIN_2
#define LINE1_STEPPER_PIN3     GPIO_PIN_3

#define LINE2_STEPPER_PIN0     GPIO_PIN_4
#define LINE2_STEPPER_PIN1     GPIO_PIN_5
#define LINE2_STEPPER_PIN2     GPIO_PIN_6
#define LINE2_STEPPER_PIN3     GPIO_PIN_7

#define ROTARY_STEPPER_PIN0     GPIO_PIN_8
#define ROTARY_STEPPER_PIN1     GPIO_PIN_9
#define ROTARY_STEPPER_PIN2     GPIO_PIN_10
#define ROTARY_STEPPER_PIN3     GPIO_PIN_11
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Tasks Names                                                     */

#define FEEDING_TASK_NAME        "Feeding Task"
#define CAPPING_TASK_NAME        "Capping Task"
#define FILLING_TASK_NAME        "Filling Task"
#define ROTATE_LINE1_TASK_NAME   "Line1 Task"
#define ROTATE_LINE2_TASK_NAME   "Line2 Task"
#define ROTATE_ROTARY_TASK_NAME  "Rotary Task"
#define UART_CONTROL_TASK_NAME   "Uart Control"
/********************************************************************************************************/
/********************************************************************************************************/
/*                                     Timers Names                                                     */

#define CAPPING_TIMER_NAME        (char * )"Capping Timer"
#define FILLING_TIMER_NAME        (char * )"Filling Timer"
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Tasks perodicties                                               */
#define FILLING_CONTROL_PERODICTY       150	
#define CAPPING_CONTROL_PERODICTY       200  
#define FEEDING_CONTROL_PERODICTY       250  
#define UART_CONTROL_PERUODICTY         500  
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Tasks priorities                                                */
#define 	FEEDING_TASK_PRIORITY					(tskIDLE_PRIORITY+2)
#define 	CAPPING_TASK_PRIORITY					(tskIDLE_PRIORITY+3)
#define 	FILLING_TASK_PRIORITY					(tskIDLE_PRIORITY+4)
#define 	ROTATE_LINE1_TASK_PRIORITY		(tskIDLE_PRIORITY+6)
#define 	ROTATE_LINE2_TASK_PRIORITY		(tskIDLE_PRIORITY+5)
#define 	ROTATE_ROTARY_TASK_PRIORITY		(tskIDLE_PRIORITY+7)
#define 	UART_CONTROL_TASK_PRIORITY		(tskIDLE_PRIORITY+1)
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                       HW Components                                                 */
DC_Motor_Config_t Capping_motor;

PUMP_Config_t Filling_Pump;

VALVE_Config_t Capping_Valve;

IR_Config_t Filling_IR;
IR_Config_t Capping_IR;
IR_Config_t Feeding_IR;

STEPPER_Config_t Line1_Stepper;
STEPPER_Config_t Line2_Stepper;
STEPPER_Config_t Rotary_Stepper ;
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                       Trace pins	                                           		     */
// tick -->0   task portA--> 1 14 15 13 12  tasks portC--> 13 14 15
#if configKEIL_TIMELINE_ANALYSIS == 1
#define FEDDINGTASKTRACEPIN				GPIO_PIN_0
#define CAPPINGTASKTRACEPIN				GPIO_PIN_1
#define FILLINGTASKTRACEPIN				GPIO_PIN_4
#define UARTCONTROLTASKTRACEPIN		GPIO_PIN_5
//port C
#define  LINE1TASKTRACEPIN				GPIO_PIN_13
#define  LINE2TASKTRACEPIN				GPIO_PIN_14
#define  ROTARYTASKTRACEPIN				GPIO_PIN_15
#endif
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     TasksHandlers                                                   */

TaskHandle_t  Feeding_Task_Handle = NULL;
TaskHandle_t  Filling_Task_Handle = NULL;
TaskHandle_t  Capping_Task_Handle = NULL;
TaskHandle_t  Rotate_Line1_Task_Handle = NULL;
TaskHandle_t  Rotate_Line2_Task_Handle = NULL;
TaskHandle_t  Rotate_Rotary_Task_Handle = NULL;
TaskHandle_t  Uart_Control_Task_Handle = NULL;
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Semaphores                                                      */

SemaphoreHandle_t Line1_stepper_semaphore = NULL;
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Timers                                                          */

TimerHandle_t Filling_Timer_Handle = NULL;
TimerHandle_t Capping_Timer_Handle = NULL;
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Event Groups                                                    */

EventGroupHandle_t  RotaryControlEvent_GRP = NULL;
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                     Bottel Counters                                                 */
uint32_t Capping_u32counter = 0;
uint32_t Filling_u32counter = 0;
/*******************************************************************************************************/
/*                                  Initilaization Functions                                           */
void Setup_HardWare(void);
void SystemClock_Config(void);
void Init_RTOS(void);

/* test */
#if configKEIL_TIMELINE_ANALYSIS == 1
static inline void InitTracePins(void);
#endif
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                 Timer CallBack Functions                                            */

void vFillingTimerCallback( TimerHandle_t xTimer )
{
  volatile uint32_t Expired_Counter = 0;
  vPortEnterCritical();
  /* Optionally do something if the pxTimer parameter is NULL. */
  configASSERT( xTimer );
  Expired_Counter = (uint32_t) pvTimerGetTimerID(xTimer);
  Expired_Counter++;
  vTimerSetTimerID(xTimer,(void *)Expired_Counter);
  /*stop filling op*/
  PUMP_voidClose(&Filling_Pump);
  /*clear filling flag to rotate rotary*/
  xEventGroupClearBits(RotaryControlEvent_GRP,FILLING_FLAG);
  xTimerStop(xTimer,0);
  vPortExitCritical();
}

void vCappingTimerCallback( TimerHandle_t xTimer )
{
  volatile uint32_t Expired_Counter = 0;
  vPortEnterCritical();
  /* Optionally do something if the pxTimer parameter is NULL. */
  configASSERT(xTimer);
  Expired_Counter = (uint32_t) pvTimerGetTimerID(xTimer);
  Expired_Counter++;
  vTimerSetTimerID(xTimer,(void *)Expired_Counter);
  /*stop capping op*/
  VALVE_voidOFF(&Capping_Valve);
  DCMotor_voidStop(&Capping_motor);
  /*clear capping flag to rotate rotary*/
  xEventGroupClearBits(RotaryControlEvent_GRP,CAPPING_FLAG);
  xTimerStop(xTimer,0);
  vPortExitCritical();
}
/*******************************************************************************************************/
/*******************************************************************************************************/
/*                                          Tasks                                                      */
// 100 us
void vtaskFeedingCotrol(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(FEEDING_CONTROL_PERODICTY);
  EXISTANCE Check_Flag = OBJECT_ABSENT;
  #if configKEIL_TIMELINE_ANALYSIS == 1
	  vTaskSetApplicationTaskTag( NULL, ( void * )FEDDINGTASKTRACEPIN );
  #endif
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Check_voidObject_Existance(&Feeding_IR, &Check_Flag);

    if ( (Check_Flag == OBJECT_PRESENT) && 
         (xSemaphoreTake(Line1_stepper_semaphore,(TickType_t)10 ) == pdTRUE) // take Line1_stepper_semaphore to stop vtaskRotateLine1
       )
    {
      /*start rotary*/
      xEventGroupClearBits(RotaryControlEvent_GRP,BIT_0);
    }
  }
}

/* 0.3 ms */
void vtaskFillingCotrol(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(FILLING_CONTROL_PERODICTY);
  EXISTANCE Check_Flag = OBJECT_ABSENT;
  #if configKEIL_TIMELINE_ANALYSIS == 1
	vTaskSetApplicationTaskTag(NULL,(void *)FILLINGTASKTRACEPIN);
  #endif
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Check_voidObject_Existance(&Filling_IR, &Check_Flag);
    if ( (Check_Flag == OBJECT_PRESENT) &&
         ((FILLING_BOTTEL_FLAG & xEventGroupGetBits(RotaryControlEvent_GRP)) == 0) 
       )
    {
      /*set filling flag to stop rotary & set FILLING_BOTTEL_FLAG to make sure that the bottel exist  */
      xEventGroupSetBits(RotaryControlEvent_GRP,FILLING_FLAG|FILLING_BOTTEL_FLAG);
      /*give semaphore to start feeding*/
      xSemaphoreGive(Line1_stepper_semaphore);
      PUMP_voidOpen(&Filling_Pump);
      if( xTimerIsTimerActive(Filling_Timer_Handle) == pdFALSE )
      {
        xTimerStart(Filling_Timer_Handle,(TickType_t)100);
      }
      Filling_u32counter = (uint32_t)pvTimerGetTimerID(Filling_Timer_Handle);
    }
  }
}

/* 0.2215 ms */
void vtaskCappingCotrol(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(CAPPING_CONTROL_PERODICTY);
  EXISTANCE Check_flag = OBJECT_ABSENT;
  #if configKEIL_TIMELINE_ANALYSIS == 1
	vTaskSetApplicationTaskTag(NULL,(void *)CAPPINGTASKTRACEPIN );
  #endif
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Check_voidObject_Existance(&Capping_IR, &Check_flag);
    if ( (Check_flag == OBJECT_PRESENT) &&
         ((CAPPING_BOTTEL_FLAG & xEventGroupGetBits(RotaryControlEvent_GRP)) == 0)
       )
    {
      /*set capping flag to stop rotary  set CAPPING_BOTTEL_FLAG to make sure that the bottel exist */
      xEventGroupSetBits(RotaryControlEvent_GRP,CAPPING_FLAG|CAPPING_BOTTEL_FLAG);
      VALVE_voidON(&Capping_Valve);
      DCMotor_voidRotate_CW(&Capping_motor);
      if( xTimerIsTimerActive(Capping_Timer_Handle) == pdFALSE )
      {
        xTimerStart(Capping_Timer_Handle,(TickType_t)100);
      }
      Capping_u32counter = (uint32_t)pvTimerGetTimerID(Capping_Timer_Handle);
    }
  }
}

/* 93 us */
void vtaskRotateRotary(void *pvParameters)
{
  uint8_t step = 0;
  uint8_t Step_Counter = 0;
  const uint8_t end = Rotary_Stepper.movingSequence == HALF_STEP ?8:4;
  const TickType_t perodicty = pdMS_TO_TICKS(Rotary_Stepper.speed*10);
  EventBits_t Flag_Bits = 0;
  #if configKEIL_TIMELINE_ANALYSIS == 1
	vTaskSetApplicationTaskTag(NULL,(void *)ROTARYTASKTRACEPIN);
  #endif
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, perodicty);
    Flag_Bits = xEventGroupGetBits(RotaryControlEvent_GRP);
    if( ( (BIT_0|FILLING_FLAG|CAPPING_FLAG) & Flag_Bits ) == 0 )
    {
      stepper_voidRotate_RTOS(&Rotary_Stepper,step);
      step = (step+1) % end;
      if( ( ( FILLING_BOTTEL_FLAG & Flag_Bits ) == FILLING_BOTTEL_FLAG ) ||
          ( ( CAPPING_BOTTEL_FLAG & Flag_Bits ) == CAPPING_BOTTEL_FLAG )
        )
      {
        if(++Step_Counter > 25 )
        {
          Step_Counter = 0;
          xEventGroupClearBits(RotaryControlEvent_GRP,CAPPING_BOTTEL_FLAG|FILLING_BOTTEL_FLAG);
        }
      }
    }
  }
}

/* 0.123 ms */
void vtaskRotateLine1(void *pvParameters)
{
  uint8_t step = 0;
  const uint8_t end = Line1_Stepper.movingSequence == HALF_STEP ?8:4;
  const TickType_t perodicty = pdMS_TO_TICKS(Line1_Stepper.speed*10);
  #if configKEIL_TIMELINE_ANALYSIS == 1
	vTaskSetApplicationTaskTag(NULL,(void *)LINE1TASKTRACEPIN);
  #endif
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, perodicty);
    if( ( (LINE1_STEEPER & xEventGroupGetBits(RotaryControlEvent_GRP)) == 0 ) && 
        (xSemaphoreTake(Line1_stepper_semaphore, (TickType_t)10) == pdTRUE) 
      )
    {
      stepper_voidRotate_RTOS(&Line1_Stepper,step);
      step = (step+1) % end;
      xSemaphoreGive(Line1_stepper_semaphore);
    }
  }
}

/* 82.375 us */
void vtaskRotateLine2(void *pvParameters)
{
  uint8_t step = 0;
  const uint8_t end = Line2_Stepper.movingSequence == HALF_STEP ?8:4;
  const TickType_t perodicty = pdMS_TO_TICKS(Line2_Stepper.speed*10);
  #if configKEIL_TIMELINE_ANALYSIS == 1
	vTaskSetApplicationTaskTag(NULL,(void *)LINE2TASKTRACEPIN);
  #endif
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, perodicty);
    stepper_voidRotate_RTOS(&Line2_Stepper,step);
    step = (step+1) % end;
  }
}

void vtaskUartControl(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(UART_CONTROL_PERUODICTY);
  uint8_t tempRecive = '\0' ;
  char Message_buffer[50];
  uint8_t RequiredBottelsNumber = 0;
  volatile EventBits_t xFlag_Bits = 0;
  #if configKEIL_TIMELINE_ANALYSIS == 1
	vTaskSetApplicationTaskTag(NULL,(void *)UARTCONTROLTASKTRACEPIN);
  #endif 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xFlag_Bits = xEventGroupGetBits(RotaryControlEvent_GRP);
    if( ( (LINE1_STEEPER & xFlag_Bits) == LINE1_STEEPER ) &&
        (RequiredBottelsNumber == 0) 
      )
    {
      HAL_UART_Receive(&huart2, (uint8_t *)&tempRecive, 1, HAL_MAX_DELAY);
      if( 
          (tempRecive>='0') && 
          (tempRecive<='9') &&
          (tempRecive != '\0') 
       )
      {
        RequiredBottelsNumber = tempRecive - '0';
        /*start line1 stepper*/
        xEventGroupClearBits(RotaryControlEvent_GRP,LINE1_STEEPER);
      }
    }
    else if( ((FILLING_FLAG | CAPPING_FLAG) & xFlag_Bits) > 0 )
    {
      memset(Message_buffer,'\0',50);
      sprintf(Message_buffer,"Filled Bottels number = %ld \r\n",Filling_u32counter);
      HAL_UART_Transmit(&huart2, (const uint8_t *)Message_buffer, sizeof(Message_buffer), HAL_MAX_DELAY);
      memset(Message_buffer,'\0',50);
      sprintf(Message_buffer,"Capping Bottels number = %ld \r\n",Capping_u32counter);
      HAL_UART_Transmit(&huart2, (const uint8_t *)Message_buffer, sizeof(Message_buffer), HAL_MAX_DELAY);
      if( 
          (Filling_u32counter == RequiredBottelsNumber) &&
          (Capping_u32counter == RequiredBottelsNumber) 
        )
      {
        Filling_u32counter = 0;
        Capping_u32counter = 0;
        RequiredBottelsNumber = 0;
        /*stop Line1 stepper */
        xEventGroupSetBits(RotaryControlEvent_GRP,LINE1_STEEPER);
        /*Stop rotary*/
        xEventGroupSetBits(RotaryControlEvent_GRP,BIT_0);
        vTimerSetTimerID( Filling_Timer_Handle , (void *)RESET );
        vTimerSetTimerID( Capping_Timer_Handle , (void *)RESET );
      } 
    }
  }
}
/*******************************************************************************************************/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Setup_HardWare();
  Init_RTOS();
                                              /*feeding task*/
  if (xTaskCreate(vtaskFeedingCotrol,
                  FEEDING_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  FEEDING_TASK_PRIORITY,
                  &Feeding_Task_Handle) == pdPASS )
  {
    HAL_Delay(20);
  }
                                              /*Filling task*/
  if (xTaskCreate(vtaskFillingCotrol,
                  FILLING_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  FILLING_TASK_PRIORITY,
                  &Filling_Task_Handle) == pdPASS)
  {
    HAL_Delay(20);
  }
                                              /*Capping task*/
  if (xTaskCreate(vtaskCappingCotrol,
                  CAPPING_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  CAPPING_TASK_PRIORITY,
                  &Capping_Task_Handle) ==pdPASS )
  {
    HAL_Delay(20);
  }
                                              /*Rotate line1 task*/
  if ( xTaskCreate(vtaskRotateLine1,
                  ROTATE_LINE1_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  ROTATE_LINE1_TASK_PRIORITY,
                  &Rotate_Line1_Task_Handle) == pdPASS )
  {
    HAL_Delay(20);
  }
                                              /*Rotate line2 task*/
  if (xTaskCreate(vtaskRotateLine2,
                  ROTATE_LINE2_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  ROTATE_LINE2_TASK_PRIORITY,
                  &Rotate_Line2_Task_Handle) == pdPASS )
  {
    HAL_Delay(20);
  }
                                              /*Rotate Rotary task*/
  if (xTaskCreate(vtaskRotateRotary,
                  ROTATE_ROTARY_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  ROTATE_ROTARY_TASK_PRIORITY,
                  &Rotate_Rotary_Task_Handle) == pdPASS)
  {
    HAL_Delay(20);
  }
                                              /*Uart Control task*/
  if (xTaskCreate(vtaskUartControl,
                  UART_CONTROL_TASK_NAME,
                  configMINIMAL_STACK_SIZE,
                  (void *)NULL,
                  UART_CONTROL_TASK_PRIORITY,
                  &Uart_Control_Task_Handle) == pdPASS)
  {
    HAL_Delay(20);
  }
  
  vTaskStartScheduler();
  for (;;)

  return 0;
}


void vApplicationTickHook(void)
{
  #if configKEIL_TIMELINE_ANALYSIS == 1
  HAL_GPIO_WritePin(GPIOA, Tick_pin.Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, Tick_pin.Pin, GPIO_PIN_RESET);
  #endif
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
{
  while (1)
  {
    /* code */
  }
  
}

void vApplicationMallocFailedHook( void )
{
  while (1)
  {
    /* code */
  }
  
}

void Init_RTOS(void)
{
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  Line1_stepper_semaphore = xSemaphoreCreateBinary();
  Filling_Timer_Handle = xTimerCreate(FILLING_TIMER_NAME,pdMS_TO_TICKS(4000),pdFALSE,(void *)0,vFillingTimerCallback);
  Capping_Timer_Handle = xTimerCreate(CAPPING_TIMER_NAME,pdMS_TO_TICKS(3000),pdFALSE,(void *)0,vCappingTimerCallback);
  RotaryControlEvent_GRP = xEventGroupCreate();

  if( RotaryControlEvent_GRP != NULL )
  {
    /*stop rotary & stop line 1 stepper */
    xEventGroupSetBits(RotaryControlEvent_GRP,LINE1_STEEPER|BIT_0);
    /*clear bottels flags to check if bottel still exist after filling or capping*/
    xEventGroupClearBits(RotaryControlEvent_GRP,FILLING_BOTTEL_FLAG|CAPPING_BOTTEL_FLAG);
  }
  /* start feeding */
	xSemaphoreGive(Line1_stepper_semaphore);
}
#if configKEIL_TIMELINE_ANALYSIS == 1
static inline void InitTracePins(void)
{
	GPIO_InitTypeDef FeddingTaskTracePin =(GPIO_InitTypeDef){GPIO_PIN_0,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};
	GPIO_InitTypeDef CappingTaskTracePin =(GPIO_InitTypeDef){GPIO_PIN_1,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};
	GPIO_InitTypeDef FillingTaskTracePin =(GPIO_InitTypeDef){GPIO_PIN_4,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};
	GPIO_InitTypeDef UartControlTaskTracePin = (GPIO_InitTypeDef){GPIO_PIN_5,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};

	GPIO_InitTypeDef line1TaskTracePin  =(GPIO_InitTypeDef){GPIO_PIN_13,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};
	GPIO_InitTypeDef line2TaskTracePin  =(GPIO_InitTypeDef){GPIO_PIN_14,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};
	GPIO_InitTypeDef RotaryTaskTracePin =(GPIO_InitTypeDef){GPIO_PIN_15,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH};
  //HAL_GPIO_Init(GPIOA,&Tick_pin);
	HAL_GPIO_Init(GPIOC,&FeddingTaskTracePin);
	HAL_GPIO_Init(GPIOC,&CappingTaskTracePin);
	HAL_GPIO_Init(GPIOC,&FillingTaskTracePin);
	HAL_GPIO_Init(GPIOA,&line1TaskTracePin );
	HAL_GPIO_Init(GPIOA,&line2TaskTracePin );
	HAL_GPIO_Init(GPIOA,&RotaryTaskTracePin);
	HAL_GPIO_Init(GPIOA,&UartControlTaskTracePin);
						
  //HAL_GPIO_WritePin(GPIOA,Tick_pin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC,FeddingTaskTracePin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC,CappingTaskTracePin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC,FillingTaskTracePin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,line1TaskTracePin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,line2TaskTracePin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,RotaryTaskTracePin.Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,UartControlTaskTracePin.Pin,GPIO_PIN_RESET);	
}
#endif
// 0x080032b4
void Setup_HardWare(void)
{
  Capping_motor = (DC_Motor_Config_t){   
                                      CAPPING_MOTOR_PORT,
                                      {
                                      (GPIO_InitTypeDef){CAPPING_MOTOR_CW_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){CAPPING_MOTOR_CCW_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH} 
                                      } 
                                     };
  
  Filling_Pump = (PUMP_Config_t){PUMP_PORT,(GPIO_InitTypeDef){Filling_Pump_PIN,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH}};
  
  Capping_Valve = (VALVE_Config_t){CAPPING_VALVE_PORT,(GPIO_InitTypeDef){CAPPING_VALVE_PIN,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH}};
  
  Filling_IR = (IR_Config_t){IR_PORT,(GPIO_InitTypeDef){FILLING_IR_PIN,GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH}};
  
  Capping_IR = (IR_Config_t){IR_PORT,(GPIO_InitTypeDef){CAPPING_IR_PIN,GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH}};
  
  Feeding_IR = (IR_Config_t){IR_PORT,(GPIO_InitTypeDef){FEEDING_IR_PIN,GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH}};
  
  Line1_Stepper = (STEPPER_Config_t){
                                      (SPEED)SPEED_5,//5
                                      (MVType)WAVE_DRIVE,
                                      STEPPER_PORT,
                                    {
                                      (GPIO_InitTypeDef){LINE1_STEPPER_PIN0,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){LINE1_STEPPER_PIN1,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){LINE1_STEPPER_PIN2,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){LINE1_STEPPER_PIN3,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH}
                                    }                                   
                                    } ;
  
  Line2_Stepper = (STEPPER_Config_t){
                                      (SPEED)MIN_SPEED,//10
                                      (MVType)WAVE_DRIVE,
                                      STEPPER_PORT,
                                     {
                                      (GPIO_InitTypeDef){LINE2_STEPPER_PIN0,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){LINE2_STEPPER_PIN1,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){LINE2_STEPPER_PIN2,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                      (GPIO_InitTypeDef){LINE2_STEPPER_PIN3,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH}
                                    }
                                    } ;
  
  Rotary_Stepper =  (STEPPER_Config_t){
                                        (SPEED)MAX_SPEED,//2
                                        (MVType)WAVE_DRIVE,
                                        STEPPER_PORT,
                                      {
                                        (GPIO_InitTypeDef){ROTARY_STEPPER_PIN0,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                        (GPIO_InitTypeDef){ROTARY_STEPPER_PIN1,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                        (GPIO_InitTypeDef){ROTARY_STEPPER_PIN2,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH},
                                        (GPIO_InitTypeDef){ROTARY_STEPPER_PIN3,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH}
                                      }
                                      } ;							
 
 DCMotor_voidInit(&Capping_motor);
 
 VALVE_voidInit(&Capping_Valve);
 
 IR_voidInit(&Filling_IR);
 
 IR_voidInit(&Capping_IR);
 
 IR_voidInit(&Feeding_IR);
 
 Stepper_voidInit(&Line1_Stepper);
 
 Stepper_voidInit(&Line2_Stepper);
 
 Stepper_voidInit(&Rotary_Stepper);
  #if configKEIL_TIMELINE_ANALYSIS == 1
	InitTracePins();
  #endif
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
