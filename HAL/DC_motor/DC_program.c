#include "gpio.h"

#include "DC_interface.h"
#include "DC_private.h"
#include "DC_config.h"

void DCMotor_voidInit(const DC_Motor_Config_t* CopY_Motor)
{
    HAL_GPIO_Init(CopY_Motor->GPIO_PORT, (GPIO_InitTypeDef *)&CopY_Motor->Motor_Data[CW_INDEX]);
    HAL_GPIO_Init(CopY_Motor->GPIO_PORT, (GPIO_InitTypeDef *)&CopY_Motor->Motor_Data[CCW_INDEX]);
}
void DCMotor_voidRotate_CW(const DC_Motor_Config_t* CopY_Motor)
{
    HAL_GPIO_WritePin(CopY_Motor->GPIO_PORT, CopY_Motor->Motor_Data[CCW_INDEX].Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin(CopY_Motor->GPIO_PORT,CopY_Motor->Motor_Data[CW_INDEX].Pin, GPIO_PIN_SET);
}
void DCMotor_voidRotate_CCW(const DC_Motor_Config_t* CopY_Motor)
{
    HAL_GPIO_WritePin(CopY_Motor->GPIO_PORT, CopY_Motor->Motor_Data[CW_INDEX].Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin(CopY_Motor->GPIO_PORT,CopY_Motor->Motor_Data[CCW_INDEX].Pin, GPIO_PIN_SET);
}
void DCMotor_voidStop(const DC_Motor_Config_t *CopY_Motor)
{
    HAL_GPIO_WritePin(CopY_Motor->GPIO_PORT, CopY_Motor->Motor_Data[CCW_INDEX].Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin(CopY_Motor->GPIO_PORT,CopY_Motor->Motor_Data[CW_INDEX].Pin, GPIO_PIN_RESET);
}
