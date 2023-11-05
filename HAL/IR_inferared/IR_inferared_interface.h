#ifndef IR_INFERARED_INTERFACE_H
#define IR_INFERARED_INTERFACE_H

typedef enum
{
    OBJECT_PRESENT = GPIO_PIN_RESET,
    OBJECT_ABSENT = GPIO_PIN_SET

}EXISTANCE;

typedef struct
{
    GPIO_TypeDef    GPIO_PORT ;
    GPIO_InitTypeDef IR_pin_config ; 

}IR_Config_t;

void IR_voidInit(const IR_Config_t * Copy_ptrstrIR);
void Check_voidObject_Existance(const IR_Config_t * Copy_ptrstrIR, EXISTANCE *Copy_Check );

#endif