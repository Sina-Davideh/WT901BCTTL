/*
 * Config.h
 *
 *  Created on: May 30, 2024
 *      Author: FTP
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

extern UART_HandleTypeDef huart3;

//Zero_e Zero_Byte = Zero;

//#define	S1 		(I2C_Device_Type){.I2C_BUS_Handler = &hi2c1, .I2C_Device_Address = ACS37800_DEFAULT_I2C_ADDRESS }

#define 	Snd_Flag	(WT901T_Transmit_Messages_Flag_Structure_t){.fSave = False, 		\
																	.fCALSW = False, 		\
																	.fDIRECTION = False, 	\
																	.fSleep_WakeUp = False, \
																	.fALG = False, 			\
																	.fGYRO = False, 		\
																	.fRSW = False, 			\
																	.fRATE = False, 		\
																	.fBAUD = False, 		\
																	.fAXOFFSET = False, 	\
																	.fAYOFFSET = False, 	\
																	.fAZOFFSET = False, 	\
																	.fGXOFFSET = False, 	\
																	.fGYOFFSET = False, 	\
																	.fGZOFFSET = False, 	\
																	.fHXOFFSET = False, 	\
																	.fHYOFFSET = False, 	\
																	.fHZOFFSET = False, 	\
																	.fD0MODE = False, 		\
																	.fD1MODE = False, 		\
																	.fD2MODE = False, 		\
																	.fD3MODE = False, 		\
																	.fD0PWMH = False, 		\
																	.fD1PWMH = False, 		\
																	.fD2PWMH = False, 		\
																	.fD3PWMH = False, 		\
																	.fD0PWMT = False, 		\
																	.fD1PWMT = False, 		\
																	.fD2PWMT = False, 		\
																	.fD3PWMT = False, 		\
																	.fIICADDR = False, 		\
																	.fLED = False, 			\
																	.fGPSBAUD = False}

//#define	S1 		(I2C_Device_Type){.I2C_BUS_Handler = &hi2c1, .I2C_Device_Address = ACS37800_DEFAULT_I2C_ADDRESS }
#define		Time_Pack_Init								(Time_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,				\
																					  .Msg_Addr = Time, 										\
																					  .YY = 0x00, 												\
																					  .MM = 0x00, 												\
																					  .DD = 0x00, 												\
																					  .hh = 0x00, 												\
																					  .mm = 0x00, 												\
																					  .ss = 0x00, 												\
																					  .msL = 0x00, 												\
																					  .msH = 0x00, 												\
																					  .SUM = 0xA5 }

#define		Acceleration_Pack_Init						(Acceleration_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,		\
																							  .Msg_Addr = Acceleration, 						\
																							  .AxL = 0x00, 										\
																							  .AxH = 0x00, 										\
																							  .AyL = 0x00, 										\
																							  .AyH = 0x00, 										\
																							  .AzL = 0x00, 										\
																							  .AzH = 0x00, 										\
																							  .TL = 0x00, 										\
																							  .TH = 0x00, 										\
																							  .SUM = 0xA6 }

#define		Angular_Velocity_Pack_Init					(Angular_Velocity_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,	\
																								  .Msg_Addr = Angular_Velocity, 				\
																								  .WxL = 0x00, 									\
																								  .WxH = 0x00, 									\
																								  .WyL = 0x00, 									\
																								  .WyH = 0x00, 									\
																								  .WzL = 0x00, 									\
																								  .WzH = 0x00, 									\
																								  .TL = 0x00, 									\
																								  .TH = 0x00, 									\
																								  .SUM = 0xA7 }

#define		Angle_Pack_Init								(Angle_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,				\
																					   .Msg_Addr = Angle, 										\
																					   .RollL = 0x00, 											\
																					   .RollH = 0x00, 											\
																					   .PitchL = 0x00, 											\
																					   .PitchH = 0x00, 											\
																					   .YawL = 0x00, 											\
																					   .YawH = 0x00, 											\
																					   .TL = 0x00, 												\
																					   .TH = 0x00, 												\
																					   .SUM = 0xA8 }

#define		Magnetic_Pack_Init							(Magnetic_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,			\
																						  .Msg_Addr = Magnetic, 								\
																						  .HxL = 0x00, 											\
																						  .HxH = 0x00, 											\
																						  .HyL = 0x00, 											\
																						  .HyH = 0x00, 											\
																						  .HzL = 0x00, 											\
																						  .HzH = 0x00, 											\
																						  .TL = 0x00, 											\
																						  .TH = 0x00, 											\
																						  .SUM = 0xA9 }

#define		Port_Pack_Init								(Data_Output_Port_Status_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,	\
																								  .Msg_Addr = Data_Port_Status, 				\
																								  .D0L = 0x00, 									\
																								  .D0H = 0x00, 									\
																								  .D1L = 0x00, 									\
																								  .D1H = 0x00, 									\
																								  .D2L = 0x00, 									\
																								  .D2H = 0x00, 									\
																								  .D3L = 0x00, 									\
																								  .D3H = 0x00, 									\
																								  .SUM = 0xAA }

#define		Atmospheric_Pressure_Height_Pack_Init		(Atmospheric_Pressure_Height_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,	\
																											 .Msg_Addr = Atmospheric_Pressure_Height, 	\
																											 .P0 = 0x00, 								\
																											 .P1 = 0x00, 								\
																											 .P2 = 0x00, 								\
																											 .P3 = 0x00, 								\
																											 .H0 = 0x00, 								\
																											 .H1 = 0x00, 								\
																											 .H2 = 0x00, 								\
																											 .H3 = 0x00, 								\
																											 .SUM = 0xAB }

#define		Longitude_Latitude_Pack_Init				(Longitude_Latitude_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,	\
																									.Msg_Addr = Longitude_Latitude, 			\
																									.Lon0 = 0x00, 								\
																									.Lon1 = 0x00, 								\
																									.Lon2 = 0x00, 								\
																									.Lon3 = 0x00, 								\
																									.Lat0 = 0x00, 								\
																									.Lat1 = 0x00, 								\
																									.Lat2 = 0x00, 								\
																									.Lat3 = 0x00, 								\
																									.SUM = 0xAC }

#define		Ground_Speed_Pack_Init						(Ground_Speed_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,		\
																							  .Msg_Addr = Ground_Speed, 						\
																							  .GPSHeightL = 0x00, 								\
																							  .GPSHeightH = 0x00, 								\
																							  .GPSYawL = 0x00, 									\
																							  .GPSYawH = 0x00, 									\
																							  .GPSV0 = 0x00, 									\
																							  .GPSV1 = 0x00, 									\
																							  .GPSV2 = 0x00, 									\
																							  .GPSV3 = 0x00, 									\
																							  .SUM = 0xAD }

#define		Quaternion_Pack_Init						(Quaternion_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,					\
																					 .Msg_Addr = Quaternion, 									\
																					 .Q0L = 0x00, 												\
																					 .Q0H = 0x00, 												\
																					 .Q1L = 0x00, 												\
																					 .Q1H = 0x00, 												\
																					 .Q2L = 0x00, 												\
																					 .Q2H = 0x00, 												\
																					 .Q3L = 0x00, 												\
																					 .Q3H = 0x00, 												\
																					 .SUM = 0xAE }

#define		Satellite_Positioning_Accuracy_Pack_Init	(Satellite_Positioning_Accuracy_Output_Packet_Struct_t){.Msg_Begin = First_Byte_Receive_Data_Pack,	\
																												.Msg_Addr = Satellite_Positioning_Accuracy, \
																												.SNL = 0x00, 								\
																												.SNH = 0x00, 								\
																												.PDOPL = 0x00, 								\
																												.PDOPH = 0x00, 								\
																												.HDOPL = 0x00, 								\
																												.HDOPH = 0x00, 								\
																												.VDOPL = 0x00, 								\
																												.VDOPH = 0x00, 								\
																												.SUM = 0xAF }


#define		WT901T_Receive_Messages_Pack_Init			(WT901T_Receive_Messages_Struct_t){.Time = Time_Pack_Init, 													\
																						   .Acceleration = Acceleration_Pack_Init, 									\
																						   .Angular_Velocity = Angular_Velocity_Pack_Init, 							\
																						   .Angle = Angle_Pack_Init, 												\
																						   .Magnetic = Magnetic_Pack_Init, 											\
																						   .Data_Port_Status = Port_Pack_Init, 										\
																						   .Atmospheric_Pressure_Height = Atmospheric_Pressure_Height_Pack_Init, 	\
																						   .Longitude_Latitude = Longitude_Latitude_Pack_Init, 						\
																						   .Ground_Speed = Ground_Speed_Pack_Init, 									\
																						   .Quaternion = Quaternion_Pack_Init, 										\
																						   .Satellite_Positioning_Accuracy = Satellite_Positioning_Accuracy_Pack_Init }

/* Local constant variable definition */
//static const uint8_t First_Byte_Receive_Data_Pack   = 0x55;
//static const uint8_t First_Byte_Transmit_Data_Pack  = 0xFF;
//static const uint8_t Second_Byte_Transmit_Data_Pack = 0xAA;




#endif /* INC_CONFIG_H_ */
