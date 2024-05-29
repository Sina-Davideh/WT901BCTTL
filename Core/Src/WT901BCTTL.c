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

void WT901_Update_Message(void){
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
			  WT901_Out_msg.Time.Registers.Msg_Begin = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Time.Registers.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Time.Registers.YY 		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Time.Registers.MM 		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Time.Registers.DD 		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Time.Registers.hh 		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Time.Registers.mm 		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Time.Registers.ss 		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Time.Registers.msL 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Time.Registers.msH 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Time.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Time.Millisecond = ((WT901_Out_msg.Time.Registers.msH << 8) | WT901_Out_msg.Time.Registers.msL);
			break;

		  case Acceleration:
				WT901_Out_msg.Acceleration.Registers.Msg_Begin 	 = Receive_Message_Temporary_Buffer.First_Byte_Receive;
				WT901_Out_msg.Acceleration.Registers.Msg_Addr  	 = Receive_Message_Temporary_Buffer.Message_Address_Receive;
				WT901_Out_msg.Acceleration.Registers.AxL		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
				WT901_Out_msg.Acceleration.Registers.AxH		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
				WT901_Out_msg.Acceleration.Registers.AyL		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
				WT901_Out_msg.Acceleration.Registers.AyH		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
				WT901_Out_msg.Acceleration.Registers.AzL		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
				WT901_Out_msg.Acceleration.Registers.AzH		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
				WT901_Out_msg.Acceleration.Registers.TL 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
				WT901_Out_msg.Acceleration.Registers.TH 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
				WT901_Out_msg.Acceleration.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

				WT901_Out_msg.Acceleration.X_Acceleration = ((WT901_Out_msg.Acceleration.Registers.AxH << 8) | WT901_Out_msg.Acceleration.Registers.AxL) / 32768 * 16 * 9.8;
				WT901_Out_msg.Acceleration.Y_Acceleration = ((WT901_Out_msg.Acceleration.Registers.AyH << 8) | WT901_Out_msg.Acceleration.Registers.AyL) / 32768 * 16 * 9.8;
				WT901_Out_msg.Acceleration.Z_Acceleration = ((WT901_Out_msg.Acceleration.Registers.AzH << 8) | WT901_Out_msg.Acceleration.Registers.AzL) / 32768 * 16 * 9.8;
				WT901_Out_msg.Acceleration.Temperature = ((WT901_Out_msg.Acceleration.Registers.TH << 8) | WT901_Out_msg.Acceleration.Registers.TL) / 100;
			break;

		  case Angular_Velocity:
			  WT901_Out_msg.Angular_Velocity.Registers.Msg_Begin = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Angular_Velocity.Registers.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Angular_Velocity.Registers.WxL		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Angular_Velocity.Registers.WxH		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Angular_Velocity.Registers.WyL		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Angular_Velocity.Registers.WyH		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Angular_Velocity.Registers.WzL		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Angular_Velocity.Registers.WzH		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Angular_Velocity.Registers.TL 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Angular_Velocity.Registers.TH 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Angular_Velocity.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Angular_Velocity.X_Angular_Velocity = ((WT901_Out_msg.Angular_Velocity.Registers.WxH << 8) | WT901_Out_msg.Angular_Velocity.Registers.WxL) / 32768 * 2000;
			  WT901_Out_msg.Angular_Velocity.Y_Angular_Velocity = ((WT901_Out_msg.Angular_Velocity.Registers.WyH << 8) | WT901_Out_msg.Angular_Velocity.Registers.WyL) / 32768 * 2000;
			  WT901_Out_msg.Angular_Velocity.Z_Angular_Velocity = ((WT901_Out_msg.Angular_Velocity.Registers.WzH << 8) | WT901_Out_msg.Angular_Velocity.Registers.WzL) / 32768 * 2000;
			  WT901_Out_msg.Angular_Velocity.Temperature = ((WT901_Out_msg.Angular_Velocity.Registers.TH << 8) | WT901_Out_msg.Angular_Velocity.Registers.TL) / 100;
			break;

		  case Angle:
			  WT901_Out_msg.Angle.Registers.Msg_Begin 	 = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Angle.Registers.Msg_Addr  	 = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Angle.Registers.RollL		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Angle.Registers.RollH		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Angle.Registers.PitchL		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Angle.Registers.PitchH		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Angle.Registers.YawL		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Angle.Registers.YawH		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Angle.Registers.TL 		 	 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Angle.Registers.TH 		 	 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Angle.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Angle.Roll 	= ((WT901_Out_msg.Angle.Registers.RollH << 8)  | WT901_Out_msg.Angle.Registers.RollL)  / 32768 * 180;
			  WT901_Out_msg.Angle.Pitch = ((WT901_Out_msg.Angle.Registers.PitchH << 8) | WT901_Out_msg.Angle.Registers.PitchL) / 32768 * 180;
			  WT901_Out_msg.Angle.Yaw 	= ((WT901_Out_msg.Angle.Registers.YawH << 8)   | WT901_Out_msg.Angle.Registers.YawL)   / 32768 * 180;
			  WT901_Out_msg.Angle.Temperature = ((WT901_Out_msg.Angle.Registers.TH << 8) | WT901_Out_msg.Angle.Registers.TL) / 100;
			break;

		  case Magnetic:
			  WT901_Out_msg.Magnetic.Registers.Msg_Begin = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Magnetic.Registers.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Magnetic.Registers.HxL		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Magnetic.Registers.HxH		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Magnetic.Registers.HyL		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Magnetic.Registers.HyH		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Magnetic.Registers.HzL		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Magnetic.Registers.HzH		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Magnetic.Registers.TL 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Magnetic.Registers.TH 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Magnetic.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Magnetic.X_Magnetic = ((WT901_Out_msg.Magnetic.Registers.HxH << 8) | WT901_Out_msg.Magnetic.Registers.HxL);
			  WT901_Out_msg.Magnetic.Y_Magnetic = ((WT901_Out_msg.Magnetic.Registers.HyH << 8) | WT901_Out_msg.Magnetic.Registers.HyL);
			  WT901_Out_msg.Magnetic.Z_Magnetic = ((WT901_Out_msg.Magnetic.Registers.HzH << 8) | WT901_Out_msg.Magnetic.Registers.HzL);
			  WT901_Out_msg.Magnetic.Temperature = ((WT901_Out_msg.Magnetic.Registers.TH << 8) | WT901_Out_msg.Magnetic.Registers.TL) / 100;
			break;

		  case Data_Port_Status:
			  WT901_Out_msg.Data_Port_Status.Registers.Msg_Begin = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Data_Port_Status.Registers.Msg_Addr  = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Data_Port_Status.Registers.D0L		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Data_Port_Status.Registers.D0H		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Data_Port_Status.Registers.D1L		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Data_Port_Status.Registers.D1H		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Data_Port_Status.Registers.D2L		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Data_Port_Status.Registers.D2H		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Data_Port_Status.Registers.D3L 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Data_Port_Status.Registers.D3H 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Data_Port_Status.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Data_Port_Status.Port_0 = (WT901_Out_msg.Data_Port_Status.Registers.D0H << 8) | WT901_Out_msg.Data_Port_Status.Registers.D0L;
			  WT901_Out_msg.Data_Port_Status.Port_1 = (WT901_Out_msg.Data_Port_Status.Registers.D1H << 8) | WT901_Out_msg.Data_Port_Status.Registers.D1L;
			  WT901_Out_msg.Data_Port_Status.Port_2 = (WT901_Out_msg.Data_Port_Status.Registers.D2H << 8) | WT901_Out_msg.Data_Port_Status.Registers.D2L;
			  WT901_Out_msg.Data_Port_Status.Port_3 = (WT901_Out_msg.Data_Port_Status.Registers.D3H << 8) | WT901_Out_msg.Data_Port_Status.Registers.D3L;
			break;

		  case Atmospheric_Pressure_Height:
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.Msg_Begin  = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.Msg_Addr   = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P0		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P1		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P2		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P3		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H0		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H1		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H2 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H3 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Atmospheric_Pressure_Height.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Atmospheric_Pressure_Height.Atmospheric_Pressure = ((WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P3 << 24) | \
			  			  			  					  	  	  	   	   	    (WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P2 << 16) | \
																				(WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P1 << 8)  | \
																				 WT901_Out_msg.Atmospheric_Pressure_Height.Registers.P0);

			  WT901_Out_msg.Atmospheric_Pressure_Height.Height = ((WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H3 << 24) | \
			  			  			  			  				  (WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H2 << 16) | \
			  													  (WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H1 << 8)  | \
			  													   WT901_Out_msg.Atmospheric_Pressure_Height.Registers.H0);
			break;

		  case Longitude_Latitude:
			  WT901_Out_msg.Longitude_Latitude.Registers.Msg_Begin 	 = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Longitude_Latitude.Registers.Msg_Addr  	 = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lon0		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lon1		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lon2		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lon3		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lat0		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lat1		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lat2 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Longitude_Latitude.Registers.Lat3 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Longitude_Latitude.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Longitude_Latitude.Longitude = ((WT901_Out_msg.Longitude_Latitude.Registers.Lon3 << 24) | \
			  					  	  	  	  	  	  	  	(WT901_Out_msg.Longitude_Latitude.Registers.Lon2 << 16) | \
			  												(WT901_Out_msg.Longitude_Latitude.Registers.Lon1 << 8)  | \
															 WT901_Out_msg.Longitude_Latitude.Registers.Lon0);

			  WT901_Out_msg.Longitude_Latitude.Latitude = ((WT901_Out_msg.Longitude_Latitude.Registers.Lat3 << 24) | \
			  			  					  	  	  	   (WT901_Out_msg.Longitude_Latitude.Registers.Lat2 << 16) | \
			  			  								   (WT901_Out_msg.Longitude_Latitude.Registers.Lat1 << 8)  | \
			  												WT901_Out_msg.Longitude_Latitude.Registers.Lat0);
			break;

		  case Ground_Speed:
			  WT901_Out_msg.Ground_Speed.Registers.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Ground_Speed.Registers.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Ground_Speed.Registers.GPSHeightL	= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Ground_Speed.Registers.GPSHeightH	= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Ground_Speed.Registers.GPSYawL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Ground_Speed.Registers.GPSYawH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Ground_Speed.Registers.GPSV0		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Ground_Speed.Registers.GPSV1		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Ground_Speed.Registers.GPSV2 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Ground_Speed.Registers.GPSV3 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Ground_Speed.Registers.SUM 		 	= Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Ground_Speed.GPS_Height = ((WT901_Out_msg.Ground_Speed.Registers.GPSHeightH << 8) | WT901_Out_msg.Ground_Speed.Registers.GPSHeightL) / 10;
			  WT901_Out_msg.Ground_Speed.GPS_Yaw 	= ((WT901_Out_msg.Ground_Speed.Registers.GPSYawH << 8) | WT901_Out_msg.Ground_Speed.Registers.GPSYawL) / 10;
			  WT901_Out_msg.Ground_Speed.GPS_Velocity = ((WT901_Out_msg.Ground_Speed.Registers.GPSV3 << 24) | \
					  	  	  	  	  	  	  	  	  	 (WT901_Out_msg.Ground_Speed.Registers.GPSV2 << 16) | \
														 (WT901_Out_msg.Ground_Speed.Registers.GPSV1 << 8)  | \
														  WT901_Out_msg.Ground_Speed.Registers.GPSV0) / 1000;
			break;

		  case Quaternion:
			  WT901_Out_msg.Quaternion.Registers.Msg_Begin  = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Quaternion.Registers.Msg_Addr   = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Quaternion.Registers.Q0L		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Quaternion.Registers.Q0H		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Quaternion.Registers.Q1L		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Quaternion.Registers.Q1H		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Quaternion.Registers.Q2L		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Quaternion.Registers.Q2H		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Quaternion.Registers.Q3L 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Quaternion.Registers.Q3H 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Quaternion.Registers.SUM 		= Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Quaternion.Quaternion_0 = ((WT901_Out_msg.Quaternion.Registers.Q0H << 8) | WT901_Out_msg.Quaternion.Registers.Q0L) / 32768;
			  WT901_Out_msg.Quaternion.Quaternion_1 = ((WT901_Out_msg.Quaternion.Registers.Q1H << 8) | WT901_Out_msg.Quaternion.Registers.Q1L) / 32768;
			  WT901_Out_msg.Quaternion.Quaternion_2 = ((WT901_Out_msg.Quaternion.Registers.Q2H << 8) | WT901_Out_msg.Quaternion.Registers.Q2L) / 32768;
			  WT901_Out_msg.Quaternion.Quaternion_3 = ((WT901_Out_msg.Quaternion.Registers.Q3H << 8) | WT901_Out_msg.Quaternion.Registers.Q3L) / 32768;
			break;

		  case Satellite_Positioning_Accuracy:
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.Msg_Begin 	 = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.Msg_Addr  	 = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.SNL		 = Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.SNH		 = Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.PDOPL		 = Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.PDOPH		 = Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.HDOPL		 = Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.HDOPH		 = Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.VDOPL 		 = Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.VDOPH 		 = Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.SUM 		 = Receive_Message_Temporary_Buffer.Checksum;

			  WT901_Out_msg.Satellite_Positioning_Accuracy.SN = ((WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.SNH << 8) | WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.SNL);
			  WT901_Out_msg.Satellite_Positioning_Accuracy.PDOP = ((WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.PDOPH << 8) | WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.PDOPL) / 32768;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.HDOP = ((WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.HDOPH << 8) | WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.HDOPL) / 32768;
			  WT901_Out_msg.Satellite_Positioning_Accuracy.VDOP = ((WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.VDOPH << 8) | WT901_Out_msg.Satellite_Positioning_Accuracy.Registers.VDOPL) / 32768;
			break;

		  default:
		// code block
    	}
	}
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


