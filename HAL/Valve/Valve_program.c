#include "gpio.h"

#include "Valve_interface.h"

void VALVE_voidInit(const VALVE_Config_t * Copy_valvePin )
{
    HAL_GPIO_Init(Copy_valvePin->GPIO_PORT,(GPIO_InitTypeDef *)&Copy_valvePin->Valve_pin);
}
void VALVE_voidON(const VALVE_Config_t * Copy_valvePin)
{
    HAL_GPIO_WritePin(Copy_valvePin->GPIO_PORT,Copy_valvePin->Valve_pin.Pin,GPIO_PIN_SET);
}
void VALVE_voidOFF(const VALVE_Config_t * Copy_valvePin)
{
    HAL_GPIO_WritePin(Copy_valvePin->GPIO_PORT,Copy_valvePin->Valve_pin.Pin,GPIO_PIN_RESET);
}