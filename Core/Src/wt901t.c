/*
 * WT901T.c
 *
 *  Created on: May 22, 2024
 *      Author: SINA
 */

#include "wt901t.h"

//Transmit_Messages_Structure_t Transmit_Messages;
WT901T_Output_Data_Struct_t WT901T;

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

void WT901_Update_Message(void){
	uint8_t WT901_RX_Buffer[11];

	HAL_UART_Receive_DMA(&huart3, WT901_RX_Buffer, sizeof(WT901_RX_Buffer));

	Receive_Message_Struct_t Receive_Message_Temporary_Buffer;

	Receive_Message_Temporary_Buffer.First_Byte_Receive = WT901_RX_Buffer[0];
	Receive_Message_Temporary_Buffer.Message_Address_Receive = WT901_RX_Buffer[1];
	Receive_Message_Temporary_Buffer.Data_Byte_0 = WT901_RX_Buffer[2];
	Receive_Message_Temporary_Buffer.Data_Byte_1 = WT901_RX_Buffer[3];
	Receive_Message_Temporary_Buffer.Data_Byte_2 = WT901_RX_Buffer[4];
	Receive_Message_Temporary_Buffer.Data_Byte_3 = WT901_RX_Buffer[5];
	Receive_Message_Temporary_Buffer.Data_Byte_4 = WT901_RX_Buffer[6];
	Receive_Message_Temporary_Buffer.Data_Byte_5 = WT901_RX_Buffer[7];
	Receive_Message_Temporary_Buffer.Data_Byte_6 = WT901_RX_Buffer[8];
	Receive_Message_Temporary_Buffer.Data_Byte_7 = WT901_RX_Buffer[9];
	Receive_Message_Temporary_Buffer.Checksum = WT901_RX_Buffer[10];


	uint16_t Receive_Message_Validate = Receive_Message_Temporary_Buffer.First_Byte_Receive \
									  + Receive_Message_Temporary_Buffer.Message_Address_Receive \
									  + Receive_Message_Temporary_Buffer.Data_Byte_0 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_1 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_2 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_3 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_4 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_5 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_6 \
									  + Receive_Message_Temporary_Buffer.Data_Byte_7;


	Receive_Message_Addr_e Receive_Message_Addr = Receive_Message_Temporary_Buffer.Message_Address_Receive;


	if(Receive_Message_Temporary_Buffer.First_Byte_Receive == First_Byte_Receive_Data_Pack && \
	  (Receive_Message_Validate & Receive_Message_Temporary_Buffer.Checksum) == Receive_Message_Temporary_Buffer.Checksum){
		switch (Receive_Message_Addr) {
		  case Time:
			  WT901T.Time.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Time.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Time.YY 		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Time.MM 		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Time.DD 		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Time.hh 		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Time.mm 		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Time.ss 		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Time.msL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Time.msH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Time.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Acceleration:
				WT901T.Acceleration.Msg_Begin = Receive_Message_Temporary_Buffer.First_Byte_Receive;
				WT901T.Acceleration.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
				WT901T.Acceleration.AxL		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
				WT901T.Acceleration.AxH		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
				WT901T.Acceleration.AyL		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
				WT901T.Acceleration.AyH		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
				WT901T.Acceleration.AzL		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
				WT901T.Acceleration.AzH		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
				WT901T.Acceleration.TL 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
				WT901T.Acceleration.TH 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
				WT901T.Acceleration.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Angular_Velocity:
			  WT901T.Angular_Velocity.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Angular_Velocity.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Angular_Velocity.WxL		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Angular_Velocity.WxH		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Angular_Velocity.WyL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Angular_Velocity.WyH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Angular_Velocity.WzL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Angular_Velocity.WzH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Angular_Velocity.TL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Angular_Velocity.TH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Angular_Velocity.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Angle:
			  WT901T.Angle.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Angle.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Angle.RollL		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Angle.RollH		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Angle.PitchL	= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Angle.PitchH	= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Angle.YawL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Angle.YawH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Angle.TL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Angle.TH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Angle.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Magnetic:
			  WT901T.Magnetic.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Magnetic.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Magnetic.HxL		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Magnetic.HxH		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Magnetic.HyL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Magnetic.HyH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Magnetic.HzL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Magnetic.HzH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Magnetic.TL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Magnetic.TH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Magnetic.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Data_Port_Status:
			  WT901T.Data_Port_Status.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Data_Port_Status.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Data_Port_Status.D0L		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Data_Port_Status.D0H		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Data_Port_Status.D1L		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Data_Port_Status.D1H		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Data_Port_Status.D2L		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Data_Port_Status.D2H		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Data_Port_Status.D3L 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Data_Port_Status.D3H 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Data_Port_Status.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Atmospheric_Pressure_Height:
			  WT901T.Atmospheric_Pressure_Height.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Atmospheric_Pressure_Height.Msg_Addr   	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Atmospheric_Pressure_Height.P0		 	= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Atmospheric_Pressure_Height.P1		 	= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Atmospheric_Pressure_Height.P2		 	= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Atmospheric_Pressure_Height.P3		 	= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Atmospheric_Pressure_Height.H0		 	= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Atmospheric_Pressure_Height.H1		 	= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Atmospheric_Pressure_Height.H2 		 	= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Atmospheric_Pressure_Height.H3 		 	= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Atmospheric_Pressure_Height.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Longitude_Latitude:
			  WT901T.Longitude_Latitude.Msg_Begin = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Longitude_Latitude.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Longitude_Latitude.Lon0		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Longitude_Latitude.Lon1		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Longitude_Latitude.Lon2		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Longitude_Latitude.Lon3		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Longitude_Latitude.Lat0		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Longitude_Latitude.Lat1		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Longitude_Latitude.Lat2		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Longitude_Latitude.Lat3 	 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Longitude_Latitude.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Ground_Speed:
			  WT901T.Ground_Speed.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Ground_Speed.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Ground_Speed.GPSHeightL	= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Ground_Speed.GPSHeightH	= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Ground_Speed.GPSYawL	= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Ground_Speed.GPSYawH	= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Ground_Speed.GPSV0		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Ground_Speed.GPSV1		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Ground_Speed.GPSV2 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Ground_Speed.GPSV3 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Ground_Speed.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Quaternion:
			  WT901T.Quaternion.Msg_Begin  	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Quaternion.Msg_Addr   	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Quaternion.Q0L			= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Quaternion.Q0H			= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Quaternion.Q1L			= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Quaternion.Q1H			= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Quaternion.Q2L			= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Quaternion.Q2H			= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Quaternion.Q3L 			= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Quaternion.Q3H 			= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Quaternion.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Satellite_Positioning_Accuracy:
			  WT901T.Satellite_Positioning_Accuracy.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Satellite_Positioning_Accuracy.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Satellite_Positioning_Accuracy.SNL		 	= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Satellite_Positioning_Accuracy.SNH		 	= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Satellite_Positioning_Accuracy.PDOPL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Satellite_Positioning_Accuracy.PDOPH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Satellite_Positioning_Accuracy.HDOPL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Satellite_Positioning_Accuracy.HDOPH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Satellite_Positioning_Accuracy.VDOPL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Satellite_Positioning_Accuracy.VDOPH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Satellite_Positioning_Accuracy.SUM 		 	= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  default:
		// code block
    	}
	}
}

