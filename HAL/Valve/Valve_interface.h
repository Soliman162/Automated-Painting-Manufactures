#ifndef VALVE_INTERFACE_H
#define VALVE_INTERFACE_H

typedef struct 
{
    GPIO_TypeDef *GPIO_PORT ;
    GPIO_InitTypeDef Valve_pin ;
    
}VALVE_Config_t;


void VALVE_voidInit(const VALVE_Config_t * Copy_valvePin );
void VALVE_voidON(const VALVE_Config_t * Copy_valvePin);
void VALVE_voidOFF(const VALVE_Config_t * Copy_valvePin);


#endif