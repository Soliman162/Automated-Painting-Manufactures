#include "gpio.h"

#include "stepper_interface.h"
#include "stepper_private.h"
#include "stepper_config.h"

void Stepper_voidInit(const STEPPER_Config_t * Stepper)
{
    uint8_t Coil = 0;

    for(Coil = 0 ; Coil <= 3 ;Coil++)
    {
        HAL_GPIO_Init(Stepper->GPIO_PORT,(GPIO_InitTypeDef *)&Stepper->Coils[Coil]);
        HAL_GPIO_WritePin(Stepper->GPIO_PORT,Stepper->Coils[Coil].Pin,GPIO_PIN_RESET);
    }
}

void stepper_voidRotate_RTOS(const STEPPER_Config_t * Stepper,uint8_t step)
{
    uint8_t Coil = 0;
    uint32_t stepValue = Stepper_movment_Mode[Stepper->movingSequence][step];
    for(Coil=0 ; Coil <= 3 ;Coil++)
    {
        HAL_GPIO_WritePin(Stepper->GPIO_PORT,Stepper->Coils[Coil].Pin,READ_BIT(stepValue,Coil) );
    }
}

void stepper_voidStop(const STEPPER_Config_t * Stepper)
{
    uint8_t Coil = 0;

    for(Coil = 0; Coil <= 3 ;Coil++)
    {
        HAL_GPIO_WritePin(Stepper->GPIO_PORT,Stepper->Coils[Coil].Pin,GPIO_PIN_RESET);
    }
}
