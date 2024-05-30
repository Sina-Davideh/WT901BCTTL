/*
 * WT901BCTTL.c
 *
 *  Created on: May 22, 2024
 *      Author: SINA
 */

#include "AHRS_Module.h"

//Transmit_Messages_Structure_t Transmit_Messages;
AHRS_Structure_t AHRS_Module;
extern WT901T_Output_Data_Struct_t WT901T;

void WT901_Init(void){
//	WT901_AHRS.Config.Have_GPS = Flase;
//
//	if(WT901_AHRS.Config.Have_GPS == Flase){
//		WT901_AHRS.Config.WT_I2C.I2C_BUS_Handler = NULL;
//		WT901_AHRS.Config.WT_I2C.I2C_Communication_Mode = NULL;
//		WT901_AHRS.Config.WT_I2C.I2C_Device_Address = NULL;
//	}
//
//	WT901_AHRS.Config.WT_UART.UART_BUS_Handler = &huart3;
//	WT901_AHRS.Config.WT_UART.UART_Communication_Mode = Direct_Memory_Access_Mode;
//	WT901_AHRS.Receive_Messages.Acceleration.X_Acceleration
}

void AHRS_Update_Message(void){

	AHRS_Module.Receive_Messages.Time.Millisecond = ((WT901T.Time.msH << 8) | WT901T.Time.msL);

	AHRS_Module.Receive_Messages.Acceleration.X = ((WT901T.Acceleration.AxH << 8) | WT901T.Acceleration.AxL) / 32768 * 16 * 9.8;
	AHRS_Module.Receive_Messages.Acceleration.Y = ((WT901T.Acceleration.AyH << 8) | WT901T.Acceleration.AyL) / 32768 * 16 * 9.8;
	AHRS_Module.Receive_Messages.Acceleration.Z = ((WT901T.Acceleration.AzH << 8) | WT901T.Acceleration.AzL) / 32768 * 16 * 9.8;
	AHRS_Module.Receive_Messages.Acceleration.Temperature = ((WT901T.Acceleration.TH << 8) | WT901T.Acceleration.TL) / 100;

	AHRS_Module.Receive_Messages.Angular_Velocity.X = ((WT901T.Angular_Velocity.WxH << 8) | WT901T.Angular_Velocity.WxL) / 32768 * 2000;
	AHRS_Module.Receive_Messages.Angular_Velocity.Y = ((WT901T.Angular_Velocity.WyH << 8) | WT901T.Angular_Velocity.WyL) / 32768 * 2000;
	AHRS_Module.Receive_Messages.Angular_Velocity.Z = ((WT901T.Angular_Velocity.WzH << 8) | WT901T.Angular_Velocity.WzL) / 32768 * 2000;
	AHRS_Module.Receive_Messages.Angular_Velocity.Temperature = ((WT901T.Angular_Velocity.TH << 8) | WT901T.Angular_Velocity.TL) / 100;

	AHRS_Module.Receive_Messages.Angle.Roll  = ((WT901T.Angle.RollH << 8)  | WT901T.Angle.RollL)  / 32768 * 180;
	AHRS_Module.Receive_Messages.Angle.Pitch = ((WT901T.Angle.PitchH << 8) | WT901T.Angle.PitchL) / 32768 * 180;
	AHRS_Module.Receive_Messages.Angle.Yaw   = ((WT901T.Angle.YawH << 8)   | WT901T.Angle.YawL)   / 32768 * 180;
	AHRS_Module.Receive_Messages.Angle.Temperature = ((WT901T.Angle.TH << 8) | WT901T.Angle.TL) / 100;

	AHRS_Module.Receive_Messages.Magnetic.X = ((WT901T.Magnetic.HxH << 8) | WT901T.Magnetic.HxL);
	AHRS_Module.Receive_Messages.Magnetic.Y = ((WT901T.Magnetic.HyH << 8) | WT901T.Magnetic.HyL);
	AHRS_Module.Receive_Messages.Magnetic.Z = ((WT901T.Magnetic.HzH << 8) | WT901T.Magnetic.HzL);
	AHRS_Module.Receive_Messages.Magnetic.Temperature = ((WT901T.Magnetic.TH << 8) | WT901T.Magnetic.TL) / 100;

	AHRS_Module.Receive_Messages.Data_Port_Status.Port_0 = (WT901T.Data_Port_Status.D0H << 8) | WT901T.Data_Port_Status.D0L;
	AHRS_Module.Receive_Messages.Data_Port_Status.Port_1 = (WT901T.Data_Port_Status.D1H << 8) | WT901T.Data_Port_Status.D1L;
	AHRS_Module.Receive_Messages.Data_Port_Status.Port_2 = (WT901T.Data_Port_Status.D2H << 8) | WT901T.Data_Port_Status.D2L;
	AHRS_Module.Receive_Messages.Data_Port_Status.Port_3 = (WT901T.Data_Port_Status.D3H << 8) | WT901T.Data_Port_Status.D3L;

AHRS_Module.Receive_Messages.Atmospheric_Pressure_Height.Pressure = ((WT901T.Atmospheric_Pressure_Height.P3 << 24) | \
																	 (WT901T.Atmospheric_Pressure_Height.P2 << 16) | \
																	 (WT901T.Atmospheric_Pressure_Height.P1 << 8)  | \
																	  WT901T.Atmospheric_Pressure_Height.P0);

AHRS_Module.Receive_Messages.Atmospheric_Pressure_Height.Height = ((WT901T.Atmospheric_Pressure_Height.H3 << 24) | \
																   (WT901T.Atmospheric_Pressure_Height.H2 << 16) | \
																   (WT901T.Atmospheric_Pressure_Height.H1 << 8)  | \
																    WT901T.Atmospheric_Pressure_Height.H0);

AHRS_Module.Receive_Messages.Longitude_Latitude.Longitude = ((WT901T.Longitude_Latitude.Lon3 << 24) | \
															 (WT901T.Longitude_Latitude.Lon2 << 16) | \
															 (WT901T.Longitude_Latitude.Lon1 << 8)  | \
															  WT901T.Longitude_Latitude.Lon0);

AHRS_Module.Receive_Messages.Longitude_Latitude.Latitude = ((WT901T.Longitude_Latitude.Lat3 << 24) | \
															(WT901T.Longitude_Latitude.Lat2 << 16) | \
															(WT901T.Longitude_Latitude.Lat1 << 8)  | \
															 WT901T.Longitude_Latitude.Lat0);

AHRS_Module.Receive_Messages.Ground_Speed.GPS_Height = ((WT901T.Ground_Speed.GPSHeightH << 8) | WT901T.Ground_Speed.GPSHeightL) / 10;
AHRS_Module.Receive_Messages.Ground_Speed.GPS_Yaw 	 = ((WT901T.Ground_Speed.GPSYawH << 8) | WT901T.Ground_Speed.GPSYawL) / 10;
AHRS_Module.Receive_Messages.Ground_Speed.GPS_Velocity = ((WT901T.Ground_Speed.GPSV3 << 24) | \
														  (WT901T.Ground_Speed.GPSV2 << 16) | \
														  (WT901T.Ground_Speed.GPSV1 << 8)  | \
														   WT901T.Ground_Speed.GPSV0) / 1000;

	AHRS_Module.Receive_Messages.Quaternion.Quaternion_0 = ((WT901T.Quaternion.Q0H << 8) | WT901T.Quaternion.Q0L) / 32768;
	AHRS_Module.Receive_Messages.Quaternion.Quaternion_1 = ((WT901T.Quaternion.Q1H << 8) | WT901T.Quaternion.Q1L) / 32768;
	AHRS_Module.Receive_Messages.Quaternion.Quaternion_2 = ((WT901T.Quaternion.Q2H << 8) | WT901T.Quaternion.Q2L) / 32768;
	AHRS_Module.Receive_Messages.Quaternion.Quaternion_3 = ((WT901T.Quaternion.Q3H << 8) | WT901T.Quaternion.Q3L) / 32768;

	AHRS_Module.Receive_Messages.Satellite_Positioning_Accuracy.SN = ((WT901T.Satellite_Positioning_Accuracy.SNH << 8) | WT901T.Satellite_Positioning_Accuracy.SNL);
	AHRS_Module.Receive_Messages.Satellite_Positioning_Accuracy.PDOP = ((WT901T.Satellite_Positioning_Accuracy.PDOPH << 8) | WT901T.Satellite_Positioning_Accuracy.PDOPL) / 32768;
	AHRS_Module.Receive_Messages.Satellite_Positioning_Accuracy.HDOP = ((WT901T.Satellite_Positioning_Accuracy.HDOPH << 8) | WT901T.Satellite_Positioning_Accuracy.HDOPL) / 32768;
	AHRS_Module.Receive_Messages.Satellite_Positioning_Accuracy.VDOP = ((WT901T.Satellite_Positioning_Accuracy.VDOPH << 8) | WT901T.Satellite_Positioning_Accuracy.VDOPL) / 32768;
}


//void WT901_Get_Time(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Acceleration(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Angular_Velocity(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Angle(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Magnetic(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Data_Port_Status(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Atmospheric_Pressure_Height(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Longitude_Latitude(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Ground_Speed(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Quaternion(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}
//
//void WT901_Get_Satellite_Positioning_Accuracy(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg){}


