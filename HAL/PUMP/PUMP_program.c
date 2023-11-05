#include "gpio.h"

#include "PUMP_interface.h"

void PUMP_voidInit(const PUMP_Config_t * Copy_pumpPin )
{
    HAL_GPIO_Init(&Copy_pumpPin->GPIO_PORT,&Copy_pumpPin->Pump_pin);
}
void PUMP_voidOpen(const PUMP_Config_t * Copy_pumpPin)
{
    HAL_GPIO_WritePin(&Copy_pumpPin->GPIO_PORT,Copy_pumpPin->Pump_pin.Pin,GPIO_PIN_SET);
}
void PUMP_voidClose(const PUMP_Config_t * Copy_pumpPin)
{
    HAL_GPIO_WritePin(&Copy_pumpPin->GPIO_PORT,Copy_pumpPin->Pump_pin.Pin,GPIO_PIN_RESET);
}