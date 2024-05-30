/*
 * AHRS_Module.h
 *
 *  Created on: May 22, 2024
 *      Author: SINA
 */

#ifndef INC_AHRS_MODULE_H_
#define INC_AHRS_MODULE_H_

/*
 *   Include required library
 */
#include "wt901t.h"
#include "Config.h"


/*
 *   Variable definition
 */

/* External variable definition */
extern UART_HandleTypeDef huart3;

/* Global variable definition */
//WT901_AHRS_Structure_t WT901_AHRS;


/*
 *   Enumeration definition
 */

typedef enum
{
	Normal_Mode = 0,
	Interrupt_Mode,
	Direct_Memory_Access_Mode
} Communication_Mode_e;


/* 
 *   Data structure definition
 */


/* 
 * Receive Message Structure
 */

/* Time Output structure */
/* Calculation formula:
 * Millisecond：ms=((msH<<8)|msL).
 * Checksum: Sum=0x55+0x51+YY+MM+DD+hh+mm+ss+ms+TL.
 */
typedef struct{
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint16_t Millisecond;
	float Seconds;
} Time_Output_Struct_t;


/* Acceleration Output structure */
/* Calculation formular:
 * X-Axis Acceleration：Ax=((AxH<<8)|AxL)/32768*16g(g is Gravity acceleration，9.8m/s2).
 * Y-Axis Acceleration：Ay=((AyH<<8)|AyL)/32768*16g(g is Gravity acceleration，9.8m/s2).
 * Z-Axis Acceleration：Az=((AzH<<8)|AzL)/32768*16g(g is Gravity acceleration，9.8m/s2).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x51+AxH+AxL+AyH+AyL+AzH+AzL+TH+TL.
 */
typedef struct{
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
	uint16_t Temperature;
} Acceleration_Output_Struct_t;


/* Angular Velocity Output structure */
/* Calculation formular:
 * X-Axis Angular Velocity：Wx=((wxH<<8)|wxL)/32768*2000(°/s).
 * Y-Axis Angular Velocity：Wy=((wyH<<8)|wyL)/32768*2000(°/s).
 * Z-Axis Angular Velocity：Wz=((wzH<<8)|wzL)/32768*2000(°/s).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x52+wxH+wxL+wyH+wyL+wzH+wzL+TH+TL.
 */
typedef struct{
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
	uint16_t Temperature;
} Angular_Velocity_Output_Struct_t;


/* Angle Output structure */
/* Calculation formular:
 * Roll Angle  (around X-Axis direction)：Roll=((RollH<<8)|RollL)/32768*180(°).
 * Pitch Angle (around Y-Axis direction): Pitch=((PitchH<<8)|PitchL)/32768*180(°).
 * Yaw Angle   (around Z-Axis direction)：Yaw=((YawH<<8)|YawL)/32768*180(°).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+TH+TL.
 */
typedef struct{
	uint16_t Roll;
	uint16_t Pitch;
	uint16_t Yaw;
	uint16_t Temperature;
} Angle_Output_Struct_t;


/* Magnetic Output structure */
/* Calculation formular:
 * X-Axis Magnetic：Hx=((HxH<<8)|HxL).
 * Y-Axis Magnetic: Hy=((HyH<<8)|HyL).
 * Z-Axis Magnetic：Hz=((HzH<<8)|HzL).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x54+HxH+HxL+HyH+HyL+HzH+HzL+TH+TL.
 */
typedef struct{
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
	uint16_t Temperature;
} Magnetic_Output_Struct_t;


/* Data output port status structure */
/* Calculation formular:
 * Data Output Port 0 Status：D0=(D0H<<8)|D0L.
 * Data Output Port 1 Status: D1=(D1H<<8)|D1L.
 * Data Output Port 2 Status：D2=(D2H<<8)|D2L.
 * Data Output Port 3 Status：D3=(D3H<<8)|D3L.
 * Checksum: Sum=0x55+0x55+D0L+D0H+D1L+D1H+D2L+D2H+D3L+D3H.
 */

/* Note:
 * Analog input port mode:
 * U=DxStatus/1024*Uvcc
 * Uvcc is the power supply voltage of the module, because the module has LDO, if the module
 * power supply voltage is greater than 3.5V, Uvcc is 3.3V. If the module supply voltage is less than
 * 3.5V, Uvcc equal to the supply voltage minus 0.2V.
 * 
 * Digital input mode:
 * Voltage level is high, the data is 1.
 * Voltage level is low, the data is 0.
 * 
 * Digital output mode:
 * Output is high，the data is 1.
 * Output is low，the data is 0.
 * 
 * PWM output mode:
 * When the port is set to PWM output mode, port status data indicates high level width,
 * the unit is us.
*/

typedef struct{
	uint16_t Port_0;
	uint16_t Port_1;
	uint16_t Port_2;
	uint16_t Port_3;
} Data_Output_Port_Status_Struct_t;


/* Atmospheric Pressure and Height Output structure */
/* Calculation formular:
 * Atmospheric Pressure：P=((P3<<24)|(P2<<16)|(P1<<8)|P0 （Pa).
 * Height：H=((H3<<24)|(H2<<16)|(H1<<8)|H0 （cm).
 * Checksum: Sum=0x55+0x56+P0+P1+P2+P3+H0+H1+H2+H3.
 */
typedef struct{
	uint32_t Pressure;
	uint32_t Height;
} Atmospheric_Pressure_Height_Output_Struct_t;


/* Longitude and Latitude Output structure */
/* Calculation formular:
 * Longitude: Lon=((Lon3<<24)|(Lon2<<16)|(Lon1<<8)|Lon0.
 * Latitude:  Lat=((Lat3<<24)|(Lat2<<16)|(Lat1<<8)|Lat0（cm）.
 * Checksum: Sum=0x55+0x57+Lon0+Lon1+Lon2+Lon3+Lat0+Lat1+Lat2+Lat3.
 */

 /* Note:
 * In NMEA0183 standard, GPS output format is ddmm.mmmmm (dd for the degree,mm.mmmmm is after decimal point),
 * WT901 removed output decimal point, so the degree of longitude can be calculated:
 * dd=Lon/100000000;
 * mm.mmmmm=(Lon%10000000)/100000；(% calculate Remainder)
 * 
 * In NMEA0183 standard , GPS output format is ddmm.mmmmm (dd for the degree,mm.mmmmm is after decimal point ),
 * WT901 removed output decimal point, so the degree of longitude can be calculated:
 * dd=Lat/100000000;
 * mm.mmmmm=(Lat%10000000)/100000；(% calculate Remainder)
 */

typedef struct{
	uint32_t Longitude;
	uint32_t Latitude;
} Longitude_Latitude_Output_Struct_t;


/* Ground Speed Output structure */
/* Calculation formular:
 * GPS Height:   GPSHeight=((GPSHeightH<<8)|GPSHeightL)/10 （m).
 * GPS Yaw:      GPSYaw=((GPSYawH<<8)|GPSYawL)/10 （°).
 * GPS Velocity: GPSV=(((Lat3<<24)|(Lat2<<16)|(Lat1<<8)|Lat0)/1000（km/h).
 * Checksum: Sum=0x55+0x58+GPSHeightL+GPSHeightH+GPSYawL+GPSYawH+GPSV0+GPSV1+GPSV2+GPSV3.
 */
typedef struct{
	uint16_t GPS_Height;
	uint16_t GPS_Yaw;
	uint32_t GPS_Velocity;
} Ground_Speed_Output_Struct_t;


/* Quaternion structure */
/* Calculation formular:
 * Quaternion: Q0=((Q0H<<8)|Q0L)/32768.
 * Quaternion: Q1=((Q1H<<8)|Q1L)/32768.
 * Quaternion: Q2=((Q2H<<8)|Q2L)/32768.
 * Quaternion: Q3=((Q3H<<8)|Q3L)/32768.
 * Checksum: Sum=0x55+0x59+Q0L+Q0H+Q1L+Q1H+Q2L+Q2H+Q3L+Q3H.
 */
typedef struct{
	uint16_t Quaternion_0;
	uint16_t Quaternion_1;
	uint16_t Quaternion_2;
	uint16_t Quaternion_3;
} Quaternion_Struct_t;


/* Satellite Positioning Accuracy Output structure */
/* Calculation formular:
 * Satellite Quantity：SN=((SNH<<8)|SNL)
 * Location Positioning Accuracy：  PDOP=((PDOPH<<8)|PDOPL)/32768.
 * Horizontal Positioning Accuracy：HDOP=((HDOPH<<8)|HDOPL)/32768.
 * Vertical Positioning Accuracy：  VDOP=((VDOPH<<8)|VDOPL)/32768.
 * Checksum: Sum=0x55+0x5A+SNL+SNH+PDOPL+PDOPH+HDOPL+HDOPH+VDOPL+VDOPH.
 */
typedef struct{
	uint16_t SN;
	uint16_t PDOP;
	uint16_t HDOP;
	uint16_t VDOP;
} Satellite_Positioning_Accuracy_Output_Struct_t;


/* Integration all output data in unit Structure for easy accessibility */
typedef struct{
	Time_Output_Struct_t Time;
	Acceleration_Output_Struct_t Acceleration;
	Angular_Velocity_Output_Struct_t Angular_Velocity;
	Angle_Output_Struct_t Angle;
	Magnetic_Output_Struct_t Magnetic;
	Data_Output_Port_Status_Struct_t Data_Port_Status;
	Atmospheric_Pressure_Height_Output_Struct_t Atmospheric_Pressure_Height;
	Longitude_Latitude_Output_Struct_t Longitude_Latitude;
	Ground_Speed_Output_Struct_t Ground_Speed;
	Quaternion_Struct_t Quaternion;
	Satellite_Positioning_Accuracy_Output_Struct_t Satellite_Positioning_Accuracy;
} Module_Output_Data_Struct_t;


/*
 * Transmit Message Structure
 */

/*
 * Offset Axis parameter structure
 */

/* Acceleration Offset Structures */
typedef struct{
    AGHxOFFSET_Struct_t     AXOFFSET;
    AGHxOFFSET_Struct_t     AYOFFSET;
    AGHxOFFSET_Struct_t     AZOFFSET;
} Acceleration_Offset_Structure_t;

/* Angular Velocity Offset Structure */
typedef struct{
    AGHxOFFSET_Struct_t     GXOFFSET;
    AGHxOFFSET_Struct_t     GYOFFSET;
    AGHxOFFSET_Struct_t     GZOFFSET;
} Angular_Velocity_Offset_Structure_t;

/* Magnetic Offset Structure */
typedef struct{
    AGHxOFFSET_Struct_t     HXOFFSET;
    AGHxOFFSET_Struct_t     HYOFFSET;
    AGHxOFFSET_Struct_t     HZOFFSET;
} Magnetic_Offset_Structure_t;

/* Axis Offset Structure */
typedef struct
{
	Acceleration_Offset_Structure_t Acceleration;
	Angular_Velocity_Offset_Structure_t Angular_Velocity;
	Magnetic_Offset_Structure_t Magnetic;
} Axis_Offset_Structure_t;


/*
 * External Port configuration structures
 */

/* External Port Mode */
typedef struct
{
    DxMODE_Struct_t         D0MODE;
    DxMODE_Struct_t         D1MODE;
    DxMODE_Struct_t         D2MODE;
    DxMODE_Struct_t         D3MODE;
} External_Port_Mode_Structure_t;

/* External Port PWM High-Level width structure */
typedef struct
{
    DxPWMH_Struct_t         D0PWMH;
    DxPWMH_Struct_t         D1PWMH;
    DxPWMH_Struct_t         D2PWMH;
    DxPWMH_Struct_t         D3PWMH;
} External_Port_PWM_Width_Structure_t;

/* External Port PWM Period structure */
typedef struct
{
    DxPWMT_Struct_t         D0PWMT;
    DxPWMT_Struct_t         D1PWMT;
    DxPWMT_Struct_t         D2PWMT;
    DxPWMT_Struct_t         D3PWMT;
} External_Port_PWM_Period_Structure_t;

/* External Port configuration structure */
typedef struct
{
	External_Port_Mode_Structure_t Mode;
	External_Port_PWM_Width_Structure_t PWM_Width;
	External_Port_PWM_Period_Structure_t PWM_Period;
} External_Port_Config_Structure_t;


/*
 * Module configuration structure
 */

typedef struct
{
    SAVE_Struct_t           Save;
    CALSW_Struct_t          CALSW;
    DIRECTION_Struct_t      DIRECTION;
    Sleep_WakeUp_Struct_t   Sleep_WakeUp;
    ALG_Struct_t            ALG;
    GYRO_Struct_t           GYRO;
    RSW_Struct_t            RSW;
    RATE_Struct_t           RATE;
    BAUD_Struct_t           BAUD;
    IICADDR_Struct_t        IICADDR;
    LED_Struct_t            LED;
    GPSBAUD_Struct_t        GPSBAUD;
} Module_General_Config_Structure_t;


/*
 * Module configuration Structure
 */

typedef struct{
	Module_General_Config_Structure_t General_Config;
	External_Port_Config_Structure_t Port_Config;
	Axis_Offset_Structure_t Axis_Offset;
} Module_Input_Data_Struct_t;

/*
 * Module configuration Structure
 */

typedef struct
{
	uint8_t	WT901_RX_Buffer[11];
	uint8_t	WT901_TX_Buffer[5];
}Data_Buffer_Struct_t;


/*
 * Module configuration Structure
 */

typedef struct
{
//	I2C_HandleTypeDef		*I2C_BUS_Handler;
	Communication_Mode_e	I2C_Communication_Mode;
	uint8_t					I2C_Device_Address;
} I2C_Communication_Struct_t;

typedef struct
{
	UART_HandleTypeDef		*UART_BUS_Handler;
	Communication_Mode_e	UART_Communication_Mode;
} UART_Communication_Struct_t;

typedef struct{
	I2C_Communication_Struct_t	WT_I2C;
	UART_Communication_Struct_t	WT_UART;
	Bool Have_GPS;
} Module_Com_Config_Struct_t;

typedef struct{
	Module_Input_Data_Struct_t Transmit_Messages;
	Module_Output_Data_Struct_t Receive_Messages;
	Module_Com_Config_Struct_t Config;
	Data_Buffer_Struct_t Buffer;
} AHRS_Structure_t;

/*
 *   Function's structure definition
 */
void WT901_Init(void);
//void WT901_Transmit_Message(UART_HandleTypeDef *huart, Transmit_Message_Struct_t *Transmit_msg);
//void WT901_Receive_Message(UART_HandleTypeDef *huart, Transmit_Message_Struct_t *Transmit_msg, Receive_Message_Struct_t *Receive_msg);
//void AHRS_Update_Message(void);
//
//void WT901_Get_Time(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Acceleration(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Angular_Velocity(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Angle(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Magnetic(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Data_Port_Status(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Atmospheric_Pressure_Height(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Longitude_Latitude(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Ground_Speed(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Quaternion(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);
//void WT901_Get_Satellite_Positioning_Accuracy(UART_HandleTypeDef *huart, Receive_Message_Struct_t *Receive_msg);

/**
  * @brief  Enable the TIM peripheral.
  * @param  __HANDLE__ TIM handle
  * @retval None
  */
//#define __HAL_TIM_ENABLE(__HANDLE__)                 ((__HANDLE__)->Receive_Messages->Acceleration->X_Acceleration|=(TIM_CR1_CEN))

#endif /* INC_AHRS_MODULE_H_ */
