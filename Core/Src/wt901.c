/*
 * wt901.c
 *
 *  Created on: May 22, 2024
 *      Author: SINA
 */

#include "wt901.h"

Transmit_Messages_Structure_t Transmit_Messages;

void WT901_Init(Transmit_Messages_Structure_t *Transmit_msgs){
    // Transmit_msgs.
}

void WT901_Transmit_Message(UART_HandleTypeDef *huart, Transmit_Message_Struct_t *Transmit_msg){
    uint8_t Transmit_Buffer[5] = {0x00};

    Transmit_Buffer[1] = First_Byte_Transmit_Data_Pack;
    Transmit_Buffer[2] = Second_Byte_Transmit_Data_Pack;
    Transmit_Buffer[3] = Transmit_msg->Address;
    Transmit_Buffer[4] = Transmit_msg->DataL;
    Transmit_Buffer[5] = Transmit_msg->DataH;

    HAL_UART_Transmit_DMA(huart, Transmit_Buffer, 5);
}
