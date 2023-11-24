#ifndef DC_INTERFACE_H
#define DC_INTERFACE_H


typedef struct
{
    GPIO_TypeDef *GPIO_PORT ;
    GPIO_InitTypeDef Motor_Data[2];

}DC_Motor_Config_t;

void DCMotor_voidInit(const DC_Motor_Config_t* CopY_Motor);
void DCMotor_voidRotate_CW(const DC_Motor_Config_t* CopY_Motor);
void DCMotor_voidRotate_CCW(const DC_Motor_Config_t* CopY_Motor);
void DCMotor_voidStop(const DC_Motor_Config_t *CopY_Motor);

#endif