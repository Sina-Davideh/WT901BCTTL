/*
 * WT901BCTTL.c
 *
 *  Created on: May 22, 2024
 *      Author: SINA
 */

#include "WT901BCTTL.h"

Transmit_Messages_Structure_t Transmit_Messages;

void WT901_Init(Transmit_Messages_Structure_t *Transmit_msgs){
    // Transmit_msgs.
}

void WT901_Transmit_Message(UART_HandleTypeDef *huart, Transmit_Message_Struct_t *Transmit_msg){
    uint8_t Transmit_Buffer[5] = {0x00};

//    Transmit_Buffer[1] = First_Byte_Transmit_Data_Pack;
//    Transmit_Buffer[2] = Second_Byte_Transmit_Data_Pack;
    Transmit_Buffer[1] = Transmit_msg->First_Byte_Transmit;
    Transmit_Buffer[2] = Transmit_msg->Second_Byte_Transmit;
    Transmit_Buffer[3] = Transmit_msg->Address;
    Transmit_Buffer[4] = Transmit_msg->DataL;
    Transmit_Buffer[5] = Transmit_msg->DataH;

    while(HAL_UART_Transmit_DMA(huart, Transmit_Buffer, 5) != HAL_OK);
}

void WT901_Receive_Message(UART_HandleTypeDef *huart, Transmit_Message_Struct_t *Transmit_msg, Receive_Message_Struct_t *Receive_msg){
	uint8_t Transmit_Buffer[5] = {0x00};
	uint8_t Receive_Buffer[11] = {0x00};

	while(HAL_UART_Transmit_DMA(huart, Transmit_Buffer, 5) != HAL_OK);

	while(HAL_UART_Receive_DMA(huart, Receive_Buffer, 11) != HAL_OK);

	Receive_msg->First_Byte_Receive = Receive_Buffer[0];
	Receive_msg->Message_Address_Receive = Receive_Buffer[1];
	Receive_msg->Data_Byte_0 = Receive_Buffer[2];
	Receive_msg->Data_Byte_1 = Receive_Buffer[3];
	Receive_msg->Data_Byte_2 = Receive_Buffer[4];
	Receive_msg->Data_Byte_3 = Receive_Buffer[5];
	Receive_msg->Data_Byte_4 = Receive_Buffer[6];
	Receive_msg->Data_Byte_5 = Receive_Buffer[7];
	Receive_msg->Data_Byte_6 = Receive_Buffer[8];
	Receive_msg->Data_Byte_7 = Receive_Buffer[9];
	Receive_msg->Checksum = Receive_Buffer[10];
}

void WT901_Get_Time(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Acceleration(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Angular_Velocity(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Angle(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Magnetic(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Data_Port_Status(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Atmospheric_Pressure_Height(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Longitude_Latitude(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Ground_Speed(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Quaternion(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}

void WT901_Get_Satellite_Positioning_Accuracy(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}


