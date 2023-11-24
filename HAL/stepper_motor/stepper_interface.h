#ifndef STEPPER_INTERFACE_H
#define STEPPER_INTERFACE_H

typedef enum 
{
    WAVE_DRIVE=0,
    HALF_STEP=1,
    FULL_STEP=2
    
}MVType;

typedef enum 
{
    MAX_SPEED=2,
    SPEED_3,
    SPEED_4,
    SPEED_5,
    SPEED_6,
    SPEED_7,
    SPEED_8,
    SPEED_9,
    SPEED_10
}SPEED;

typedef struct 
{

    SPEED speed;
    MVType  movingSequence;
    GPIO_TypeDef *GPIO_PORT;
    GPIO_InitTypeDef Coils[4];

}STEPPER_Config_t;


void Stepper_voidInit(const STEPPER_Config_t * Stepper);
void stepper_voidRotate_RTOS(const STEPPER_Config_t * Stepper,uint8_t step);
void stepper_voidStop(const STEPPER_Config_t * Stepper);


#endif