#include "gpio.h"

#include "IR_inferared_interface.h"
#include "IR_inferared_private.h"
#include "IR_inferared_config.h"


void IR_voidInit(const IR_Config_t * Copy_ptrstrIR)
{
    HAL_GPIO_Init(&Copy_ptrstrIR->GPIO_PORT,&Copy_ptrstrIR->IR_pin_config );
}

void Check_voidObject_Existance(const IR_Config_t * Copy_ptrstrIR, EXISTANCE *Copy_Check )
{
    *Copy_Check = HAL_GPIO_ReadPin(&Copy_ptrstrIR->GPIO_PORT, Copy_ptrstrIR->IR_pin_config.Pin);
}