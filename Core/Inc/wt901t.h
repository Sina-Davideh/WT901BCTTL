/*
 * WT901T.h
 *
 *  Created on: May 22, 2024
 *      Author: SINA DAVIDEH
 */

#ifndef INC_WT901T_H_
#define INC_WT901T_H_

/*
 *   Include required library
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Config.h"

/*
 *   Variable definition
 */
#define I2C_COMMUNICATION
#define UART_COMMUNICATION
#define HAVE_GPS



/*
 *   Variable definition
 */

/* External variable definition */
extern UART_HandleTypeDef huart3;

/* Local constant variable definition */
static const uint8_t First_Byte_Receive_Data_Pack   = 0x55;
static const uint8_t First_Byte_Transmit_Data_Pack  = 0xFF;
static const uint8_t Second_Byte_Transmit_Data_Pack = 0xAA;


/*
 *   Enumeration definition
 */

/* Receive Message Address */
typedef enum
{
    Time = 0x50,                    /* Time Output */
    Acceleration,                   /* Acceleration Output */
    Angular_Velocity,               /* Angular Velocity Output */
    Angle,                          /* Angle Output */
    Magnetic,                       /* Magnetic Output */
    Data_Port_Status,               /* Data Output Port Status */
    Atmospheric_Pressure_Height,    /* Atmospheric Pressure Height Output */
    Longitude_Latitude,             /* Longitude Latitude Output */
    Ground_Speed,                   /* Ground Speed Output */
    Quaternion,                     /* Quaternion */
    Satellite_Positioning_Accuracy  /* Satellite Positioning Accuracy Output */
} Receive_Message_Addr_e;


/* Transmit Message Address */
typedef enum
{
    SAVE = 0x00,   /* Save */
    CALSW,         /* Calibration */
    RSW,           /* Return data content */
    RATE,          /* Return data Speed */
    BAUD,          /* Baud rate */
    AXOFFSET,      /* X-Axis Acceleration bias */
    AYOFFSET,      /* Y-Axis Acceleration bias */
    AZOFFSET,      /* Z-Axis Acceleration bias */
    GXOFFSET,      /* X-Axis angular velocity bias */
    GYOFFSET,      /* Y-Axis angular velocity bias */
    GZOFFSET,      /* Z-Axis angular velocity bias */
    HXOFFSET,      /* X-Axis Magnetic bias */
    HYOFFSET,      /* Y-Axis Magnetic bias */
    HZOFFSET,      /* Z-Axis Magnetic bias */
    D0MODE,        /* D0 mode */
    D1MODE,        /* D1 mode */
    D2MODE,        /* D2 mode */
    D3MODE,        /* D3 mode */
    D0PWMH,        /* D0PWM High-level width */
    D1PWMH,        /* D1PWM High-level width */
    D2PWMH,        /* D2PWM High-level width */
    D3PWMH,        /* D3PWM High-level width */
    D0PWMT,        /* D0PWM Period */
    D1PWMT,        /* D1PWM Period */
    D2PWMT,        /* D2PWM Period */
    D3PWMT,        /* D3PWM Period */
    IICADDR,       /* IIC address */
    LEDOFF,        /* Turn off LED */
    GPSBAUD,       /* GPS baud rate */

    SLEEP = 0x22,  /*Sleep-Wake up */
    DIRECTION,     /* Installation direction */
    ALG,           /* Algorithm transition */

    YYMM = 0x30,   /* Year, Month */
    DDHH,          /* Day, Hour */
    MMSS,          /* Minute, Second */
    MS,            /* Millisecond */
    AX,            /* X-Axis Acceleration */
    AY,            /* Y-Axis Acceleration */
    AZ,            /* Z-Axis Acceleration */
    GX,            /* X-Axis angular velocity */
    GY,            /* Y-Axis angular velocity */
    GZ,            /* Z-Axis angular velocity */
    HX,            /* X-Axis Magnetic */
    HY,            /* Y-Axis Magnetic */
    HZ,            /* Z-Axis Magnetic */
    Roll,          /* X-Axis Angle */
    Pitch,         /* Y-Axis Angle */
    Yaw,           /* Z-Axis Angle */
    TEMP,          /* Temperature */
    D0Status,      /* D0Status */
    D1Status,      /* D1Status */
    D2Status,      /* D2Status */
    D3Status,      /* D3Status */
    PressureL,     /* Pressure Low Byte */
    PressureH,     /* Pressure High Byte */
    HeightL,       /* Height Low Byte */
    HeightH,       /* Height High Byte */
    LonL,          /* Longitude Low Byte */
    LonH,          /* Longitude High Byte */
    LatL,          /* Latitude Low Byte */
    LatH,          /* Latitude High Byte */
    GPSHeight,     /* GPS Height */
    GPSYaw,        /* GPS Yaw */
    GPSVL,         /* GPS speed Low byte */
    GPSVH,         /* GPS speed High byte */
    Q0,            /* Quaternion Q0 */
    Q1,            /* Quaternion Q1 */
    Q2,            /* Quaternion Q2 */
    Q3,            /* Quaternion Q3 */
    GYRO = 0x63    /* Gyroscope Automatic Calibration */
} Transmit_Message_Addr_e;

/* Zero */
typedef enum
{
    Zero = 0x00
} Zero_e;

/* Configuration
 * Save Configuration.
 */
typedef enum
{
    Save_Current_Configuration = 0,
    Set_to_Default_Setting
} Save_Configuration_e;

/* Calibrate
 * Set calibration mode.
 */
typedef enum
{
    Exit_Calibration_Mode = 0,
    Enter_Gyroscope_and_Accelerometer_Calibration_Mode,
    Enter_Magnetic_Calibration_Mode,
    Set_Height_to_Zero
} Calibrate_e;

/* Direction
 * Set Installation direction.
 */
typedef enum
{
    Set_to_Horizontal_Installation = 0,
    Set_to_Vertical_Installation
} Set_Installation_Direction_e;

/* Sleep/Wake Up
 * Sent this instruction to enter Sleep state,
 * Sent it once again, module enter the working state from the standby state.
*/
typedef enum
{
    Sleep_WakeUp = 0x01,
} Sleep_WakeUp_e;

/* Algorithm transition
 * 6-Axis / 9-Axis algorithm transition
 */
typedef enum
{
    Set_to_9_Axis_Algorithm = 0,
    Set_to_6_Axis_Algorithm
} Algorithm_Transition_e;

/* Gyroscope automatic calibration
 * gyroscope automatic calibration
*/
typedef enum
{
    Set_to_Gyroscope_Automatic_Calibration = 0,
    Removed_to_Gyroscope_Automatic_Calibration
} Gyroscope_Automatic_Calibration_e;

/* Set return content
 * Set enable every data pack you need returning. 
 */
typedef enum
{
    Reset_Output_Pack = 0,
    Set_Output_Pack
} Set_Return_Content_e;

/* Set return rate
 * After the setup is complete, need to save and re-power the module to take effect.
 */
typedef enum
{
    Rate_01Hz = 0x01,
    Rate_05Hz,
    Rate_1Hz,
    Rate_2Hz,
    Rate_5Hz,
    Rate_10Hz,          /* default */
    Rate_20Hz,
    Rate_50Hz,
    Rate_100Hz,
    Rate_125Hz,
    Rate_200Hz,
    Rate_Single,
    Not_Output,
} Set_Return_Rate_e;

/* Set baud rate
 */
typedef enum
{
    Baud_Rate_2400 = 0,
    Baud_Rate_4800,
    Baud_Rate_9600,     /* default */
    Baud_Rate_19200,
    Baud_Rate_38400,
    Baud_Rate_57600,
    Baud_Rate_115200,
    Baud_Rate_230400,
    Baud_Rate_460800,
    Baud_Rate_921600,
} Set_Baud_Rate_e;

/* Set port Dx mode
 */
typedef enum
{
    Analog_Input = 0,   /* default */
    Digital_Input,
    Digital_Output_High,
    Digital_Output_Low,
    PWM_Output,
    Connect_to_TX_of_GPS
} Set_Port_Dx_Mode_e;

/* Set LED */
typedef enum
{
    Turn_ON_LED = 0,
    Turn_OFF_LED
} Set_LED_State_e;

/* Set GPS baud rate
 * After set it up, you need to save the configuration and then restart the module.
 */
typedef enum
{
    GPS_Baud_Rate_2400 = 0,
    GPS_Baud_Rate_4800,
    GPS_Baud_Rate_9600, /* default */
    GPS_Baud_Rate_19200,
    GPS_Baud_Rate_38400,
    GPS_Baud_Rate_57600,
    GPS_Baud_Rate_115200,
    GPS_Baud_Rate_230400,
    GPS_Baud_Rate_460800,
    GPS_Baud_Rate_921600,
} Set_GPS_Baud_Rate_e;


/* 
 *   Data structure definition
 */


/* 
 * General form of message receiving structure
 */

typedef struct
{
    uint8_t First_Byte_Receive;
    uint8_t Message_Address_Receive;
    uint8_t Data_Byte_0;
    uint8_t Data_Byte_1;
    uint8_t Data_Byte_2;
    uint8_t Data_Byte_3;
    uint8_t Data_Byte_4;
    uint8_t Data_Byte_5;
    uint8_t Data_Byte_6;
    uint8_t Data_Byte_7;
    uint8_t Checksum;
} Receive_Message_Struct_t;


/* Time Output structure */
/* Calculation formula:
 * Millisecond：ms=((msH<<8)|msL).
 * Checksum: Sum=0x55+0x51+YY+MM+DD+hh+mm+ss+ms+TL.
 */
typedef struct
{
    uint8_t Msg_Begin;  				/* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;   	/* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t YY;                         /* YY：Year，20YY Year */
    uint8_t MM;                         /* MM：Month */
    uint8_t DD;                         /* DD：Day */
    uint8_t hh;                         /* hh：hour */
    uint8_t mm;                         /* mm：minute */
    uint8_t ss;                         /* ss：Second */
    uint8_t msL;                        /* ms：Millisecond Low byte data */
    uint8_t msH;                        /* ms：Millisecond High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Time_Output_Packet_Struct_t;


/* Acceleration Output structure */
/* Calculation formular:
 * X-Axis Acceleration：Ax=((AxH<<8)|AxL)/32768*16g(g is Gravity acceleration，9.8m/s2).
 * Y-Axis Acceleration：Ay=((AyH<<8)|AyL)/32768*16g(g is Gravity acceleration，9.8m/s2).
 * Z-Axis Acceleration：Az=((AzH<<8)|AzL)/32768*16g(g is Gravity acceleration，9.8m/s2).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x51+AxH+AxL+AyH+AyL+AzH+AzL+TH+TL.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t AxL;                        /* AxL：Acceleration in X-Axis direction Low byte data */
    uint8_t AxH;                        /* AxH：Acceleration in X-Axis direction High byte data */
    uint8_t AyL;                        /* AyL：Acceleration in Y-Axis direction Low byte data */
    uint8_t AyH;                        /* AyH：Acceleration in Y-Axis direction High byte data */
    uint8_t AzL;                        /* AzL：Acceleration in Z-Axis direction Low byte data */
    uint8_t AzH;                        /* AzH:Acceleration in Z-Axis direction High byte data */
    uint8_t TL;                         /* TL：Temperature Low byte data */
    uint8_t TH;                         /* TH：Temperature High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Acceleration_Output_Packet_Struct_t;


/* Angular Velocity Output structure */
/* Calculation formular:
 * X-Axis Angular Velocity：Wx=((wxH<<8)|wxL)/32768*2000(°/s).
 * Y-Axis Angular Velocity：Wy=((wyH<<8)|wyL)/32768*2000(°/s).
 * Z-Axis Angular Velocity：Wz=((wzH<<8)|wzL)/32768*2000(°/s).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x52+wxH+wxL+wyH+wyL+wzH+wzL+TH+TL.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t WxL;                        /* WxL：Angular Velocity in X-Axis direction Low byte data */
    uint8_t WxH;                        /* WxH：Angular Velocity in X-Axis direction High byte data */
    uint8_t WyL;                        /* WyL：Angular Velocity in Y-Axis direction Low byte data */
    uint8_t WyH;                        /* WyH：Angular Velocity in Y-Axis direction High byte data */
    uint8_t WzL;                        /* WzL：Angular Velocity in Z-Axis direction Low byte data */
    uint8_t WzH;                        /* WzH:Angular Velocity in Z-Axis direction High byte data */
    uint8_t TL;                         /* TL：Temperature Low byte data */
    uint8_t TH;                         /* TH：Temperature High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Angular_Velocity_Output_Packet_Struct_t;


/* Angle Output structure */
/* Calculation formular:
 * Roll Angle  (around X-Axis direction)：Roll=((RollH<<8)|RollL)/32768*180(°).
 * Pitch Angle (around Y-Axis direction): Pitch=((PitchH<<8)|PitchL)/32768*180(°).
 * Yaw Angle   (around Z-Axis direction)：Yaw=((YawH<<8)|YawL)/32768*180(°).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+TH+TL.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t RollL;                      /* RollL：Roll Angle Low byte data */
    uint8_t RollH;                      /* RollH：Roll Angle High byte data */
    uint8_t PitchL;                     /* PitchL：Pitch Angle Low byte data */
    uint8_t PitchH;                     /* PitchH：Pitch Angle High byte data */
    uint8_t YawL;                       /* YawL：Yaw Angle Low byte data */
    uint8_t YawH;                       /* YawH:Yaw Angle High byte data */
    uint8_t TL;                         /* TL：Temperature Low byte data */
    uint8_t TH;                         /* TH：Temperature High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Angle_Output_Packet_Struct_t;


/* Magnetic Output structure */
/* Calculation formular:
 * X-Axis Magnetic：Hx=((HxH<<8)|HxL).
 * Y-Axis Magnetic: Hy=((HyH<<8)|HyL).
 * Z-Axis Magnetic：Hz=((HzH<<8)|HzL).
 * Temperature：T=((TH<<8)|TL) /100 ℃.
 * Checksum: Sum=0x55+0x54+HxH+HxL+HyH+HyL+HzH+HzL+TH+TL.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t HxL;                        /* HxL：Magnetic field in X-Axis direction Low byte data */
    uint8_t HxH;                        /* HxH：Magnetic field in X-Axis direction High byte data */
    uint8_t HyL;                        /* HyL：Magnetic field in Y-Axis direction Low byte data */
    uint8_t HyH;                        /* HyH：Magnetic field in Y-Axis direction High byte data */
    uint8_t HzL;                        /* HzL：Magnetic field in Z-Axis direction Low byte data */
    uint8_t HzH;                        /* HzH:Magnetic field in Z-Axis direction High byte data */
    uint8_t TL;                         /* TL：Temperature Low byte data */
    uint8_t TH;                         /* TH：Temperature High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Magnetic_Output_Packet_Struct_t;


/* Data output port status structure */
/* Calculation formular:
 * Data Output Port 0 Status：D0=(D0H<<8)|D0L.
 * Data Output Port 1 Status: D1=(D1H<<8)|D1L.
 * Data Output Port 2 Status：D2=(D2H<<8)|D2L.
 * Data Output Port 3 Status：D3=(D3H<<8)|D3L.
 * Checksum: Sum=0x55+0x55+D0L+D0H+D1L+D1H+D2L+D2H+D3L+D3H.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t D0L;                        /* D0L：Data Output Port 0 Status Low byte data */
    uint8_t D0H;                        /* D0H：Data Output Port 0 Status High byte data */
    uint8_t D1L;                        /* D1L：Data Output Port 1 Status Low byte data */
    uint8_t D1H;                        /* D1H：Data Output Port 1 Status High byte data */
    uint8_t D2L;                        /* D2L：Data Output Port 2 Status Low byte data */
    uint8_t D2H;                        /* D2H:Data Output Port 2 Status High byte data */
    uint8_t D3L;                        /* D3L：Data Output Port 3 Status Low byte data */
    uint8_t D3H;                        /* D3H：Data Output Port 3 Status High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Data_Output_Port_Status_Packet_Struct_t;

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


/* Atmospheric Pressure and Height Output structure */
/* Calculation formular:
 * Atmospheric Pressure：P=((P3<<24)|(P2<<16)|(P1<<8)|P0 （Pa).
 * Height：H=((H3<<24)|(H2<<16)|(H1<<8)|H0 （cm).
 * Checksum: Sum=0x55+0x56+P0+P1+P2+P3+H0+H1+H2+H3.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t P0;                         /* P0：Atmospheric Pressure byte 1 of 4-byte data */
    uint8_t P1;                         /* P1：Atmospheric Pressure byte 2 of 4-byte data */
    uint8_t P2;                         /* P2：Atmospheric Pressure byte 3 of 4-byte data */
    uint8_t P3;                         /* P3：Atmospheric Pressure byte 4 of 4-byte data */
    uint8_t H0;                         /* H0：Height byte 1 of 4-byte data */
    uint8_t H1;                         /* H1:Height byte 2 of 4-byte data */
    uint8_t H2;                         /* H2：Height byte 3 of 4-byte data */
    uint8_t H3;                         /* H3：Height byte 4 of 4-byte data */
    uint8_t SUM;                        /* Data Checksum */
} Atmospheric_Pressure_Height_Output_Packet_Struct_t;


/* Longitude and Latitude Output structure */
/* Calculation formular:
 * Longitude: Lon=((Lon3<<24)|(Lon2<<16)|(Lon1<<8)|Lon0.
 * Latitude:  Lat=((Lat3<<24)|(Lat2<<16)|(Lat1<<8)|Lat0（cm）.
 * Checksum: Sum=0x55+0x57+Lon0+Lon1+Lon2+Lon3+Lat0+Lat1+Lat2+Lat3.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t Lon0;                       /* P0：Longitude byte 1 of 4-byte data */
    uint8_t Lon1;                       /* P1：Longitude byte 2 of 4-byte data */
    uint8_t Lon2;                       /* P2：Longitude byte 3 of 4-byte data */
    uint8_t Lon3;                       /* P3：Longitude byte 4 of 4-byte data */
    uint8_t Lat0;                       /* H0：Latitude byte 1 of 4-byte data */
    uint8_t Lat1;                       /* H1:Latitude byte 2 of 4-byte data */
    uint8_t Lat2;                       /* H2：Latitude byte 3 of 4-byte data */
    uint8_t Lat3;                       /* H3：Latitude byte 4 of 4-byte data */
    uint8_t SUM;                        /* Data Checksum */
} Longitude_Latitude_Output_Packet_Struct_t;

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


/* Ground Speed Output structure */
/* Calculation formular:
 * GPS Height:   GPSHeight=((GPSHeightH<<8)|GPSHeightL)/10 （m).
 * GPS Yaw:      GPSYaw=((GPSYawH<<8)|GPSYawL)/10 （°).
 * GPS Velocity: GPSV=(((Lat3<<24)|(Lat2<<16)|(Lat1<<8)|Lat0)/1000（km/h).
 * Checksum: Sum=0x55+0x58+GPSHeightL+GPSHeightH+GPSYawL+GPSYawH+GPSV0+GPSV1+GPSV2+GPSV3.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t GPSHeightL;                 /* GPSHeightL：GPS Height Low byte data */
    uint8_t GPSHeightH;                 /* GPSHeightH：GPS Height High byte data */
    uint8_t GPSYawL;                    /* GPSYawL：GPS Yaw Low byte data */
    uint8_t GPSYawH;                    /* GPSYawH：GPS Yaw High byte data */
    uint8_t GPSV0;                      /* GPSV0：GPS Velocity byte 1 of 4-byte data */
    uint8_t GPSV1;                      /* GPSV1:GPS Velocity byte 2 of 4-byte data */
    uint8_t GPSV2;                      /* GPSV2：GPS Velocity byte 3 of 4-byte data */
    uint8_t GPSV3;                      /* GPSV3：GPS Velocity byte 4 of 4-byte data */
    uint8_t SUM;                        /* Data Checksum */
} Ground_Speed_Output_Packet_Struct_t;


/* Quaternion structure */
/* Calculation formular:
 * Quaternion: Q0=((Q0H<<8)|Q0L)/32768.
 * Quaternion: Q1=((Q1H<<8)|Q1L)/32768.
 * Quaternion: Q2=((Q2H<<8)|Q2L)/32768.
 * Quaternion: Q3=((Q3H<<8)|Q3L)/32768.
 * Checksum: Sum=0x55+0x59+Q0L+Q0H+Q1L+Q1H+Q2L+Q2H+Q3L+Q3H.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t Q0L;                        /* Q0L：Quaternion 0 Low byte data */
    uint8_t Q0H;                        /* Q0H：Quaternion 0 High byte data */
    uint8_t Q1L;                        /* Q1L：Quaternion 1 Low byte data */
    uint8_t Q1H;                        /* Q1H：Quaternion 1 High byte data */
    uint8_t Q2L;                        /* Q2L：Quaternion 2 Low byte data */
    uint8_t Q2H;                        /* Q2H:Quaternion 2 High byte data */
    uint8_t Q3L;                        /* Q3L：Quaternion 3 Low byte data */
    uint8_t Q3H;                        /* Q3H：Quaternion 3 High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Quaternion_Packet_Struct_t;


/* Satellite Positioning Accuracy Output structure */
/* Calculation formular:
 * Satellite Quantity：SN=((SNH<<8)|SNL)
 * Location Positioning Accuracy：  PDOP=((PDOPH<<8)|PDOPL)/32768.
 * Horizontal Positioning Accuracy：HDOP=((HDOPH<<8)|HDOPL)/32768.
 * Vertical Positioning Accuracy：  VDOP=((VDOPH<<8)|VDOPL)/32768.
 * Checksum: Sum=0x55+0x5A+SNL+SNH+PDOPL+PDOPH+HDOPL+HDOPH+VDOPL+VDOPH.
 */
typedef struct
{
    uint8_t Msg_Begin;                  /* Start message acknowledge */
    Receive_Message_Addr_e Msg_Addr;    /* Message Address for data package recognition. This message address using to recognize which type data package received. */
    uint8_t SNL;                        /* SNL：Satellite quantity Low byte data */
    uint8_t SNH;                        /* SNH：Satellite quantity High byte data */
    uint8_t PDOPL;                      /* PDOPL：Location positioning accuracy Low byte data */
    uint8_t PDOPH;                      /* PDOPH：Location positioning accuracy High byte data */
    uint8_t HDOPL;                      /* HDOPL：Horizontal positioning accuracy Low byte data */
    uint8_t HDOPH;                      /* HDOPH:Horizontal positioning accuracy High byte data */
    uint8_t VDOPL;                      /* VDOPL：Vertical positioning accuracy Low byte data */
    uint8_t VDOPH;                      /* VDOPH：Vertical positioning accuracy High byte data */
    uint8_t SUM;                        /* Data Checksum */
} Satellite_Positioning_Accuracy_Output_Packet_Struct_t;


/*
 * WT901T integrated AHRS Sensor, output Data Structure.
 * This structure consist of all output message receive from the AHRS sensor.
 */
typedef struct
{
	Time_Output_Packet_Struct_t Time;
	Acceleration_Output_Packet_Struct_t Acceleration;
	Angular_Velocity_Output_Packet_Struct_t Angular_Velocity;
	Angle_Output_Packet_Struct_t Angle;
	Magnetic_Output_Packet_Struct_t Magnetic;
	Data_Output_Port_Status_Packet_Struct_t Data_Port_Status;
	Atmospheric_Pressure_Height_Output_Packet_Struct_t Atmospheric_Pressure_Height;
	Longitude_Latitude_Output_Packet_Struct_t Longitude_Latitude;
	Ground_Speed_Output_Packet_Struct_t Ground_Speed;
	Quaternion_Packet_Struct_t Quaternion;
	Satellite_Positioning_Accuracy_Output_Packet_Struct_t Satellite_Positioning_Accuracy;
} WT901T_Receive_Messages_Struct_t;



/*
 * Transmit Messages Structures
 */

/*
 * General form of Message Transmission Structure
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    uint8_t DataL;
    uint8_t DataH;
} Transmit_Message_Struct_t;


/*
 * Save Configuration messages structures.
 * 0： Save current configuration.
 * 1： Set to default setting.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Save_Configuration_e Save;
    Zero_e Empty;
} SAVE_Struct_t;

/*
 * Calibration selector messages structures.
 * 0： Exit calibration mode.
 * 1： Enter Gyroscope and Accelerometer calibration mode.
 * 2： Enter magnetic calibration mode.
 * 3： Set height to 0.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Calibrate_e CALSW;
    Zero_e Empty;
} CALSW_Struct_t;

/*
 * Set Installation direction messages structuresl.
 * 0： Set to Horizontal installation.
 * 1： Set to Vertical installation.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Set_Installation_Direction_e DIRECTION;
    Zero_e Empty;
} DIRECTION_Struct_t;

/*
 * Sleep/ Wake-up messages structures.
 *
 * Notice:
 * Sent this instruction to enter sleep state,
 * sent it once again, module enter the working state from the standby state.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Sleep_WakeUp_e Sleep_WakeUp;
    Zero_e Empty;
} Sleep_WakeUp_Struct_t;

/*
 * Algorithm transition messages structures.
 * 6-axis/ 9-axis algorithm transition.
 * 0： Set to 9-axis algorithm.
 * 1： Set to 6-axis algorithm.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Algorithm_Transition_e ALG;
    Zero_e Empty;
} ALG_Struct_t;

/*
 * Gyroscope automatic calibration messages structures.
 * 0： Set to gyroscope automatic calibration.
 * 1： Removed to gyroscope automatic calibration.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Gyroscope_Automatic_Calibration_e GYRO;
    Zero_e Empty;
} GYRO_Struct_t;

/*
 * Return switch low byte structures.
 * 0x50 pack： time pack.
 * 0x51 pack： Acceleration pack.
 * 0x52 pack： Angular velocity pack.
 * 0x53 pack： Angle Pack.
 * 0x54 pack： Magnetic Pack.
 * 0x55 pack： Port status pack.
 * 0x56 pack： Atmospheric pressure &Height Pack.
 * 0x57 pack： Longitude and Latitude Output Pack.
 */
typedef struct
{
    union
    {
        uint8_t all;
        struct
        {
            Set_Return_Content_e Time_Pack : 1;
            Set_Return_Content_e Acceleration_Pack : 1;
            Set_Return_Content_e Angular_Velocity_Pack : 1;
            Set_Return_Content_e Angle_Pack : 1;
            Set_Return_Content_e Magnetic_Pack : 1;
            Set_Return_Content_e Port_Status_Pack : 1;
            Set_Return_Content_e Atmospheric_Pressure_Height_Pack : 1;
            Set_Return_Content_e Longitude_Latitude_Output_Pack : 1;
        } bits;
    } data;
} RSWL_Struct_t;

/*
 * Return switch high byte structures.
 * 0x58 pack： GPS speed Pack.
 * 0x59 pack： Quaternion Pack.
 * 0x5A pack： Satellite position accuracy.
 */
typedef struct
{
    union
    {
        uint8_t all;
        struct
        {
            Set_Return_Content_e GPS_Speed_Pack : 1;
            Set_Return_Content_e Quaternion_Pack : 1;
            Set_Return_Content_e Satellite_Position_Accuracy_Pack : 1;
            Set_Return_Content_e : 5;
        } bits;
    } data;
} RSWH_Struct_t;

/*
 * Set return content messages structures.
 * 0： Output pack Disabled.
 * 1： Output pack Enable.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    RSWL_Struct_t RSWL;
    RSWH_Struct_t RSWH;
} RSW_Struct_t;

/*
 * Set return rate messages structures.
 * 0x01： 0.1Hz
 * 0x02： 0.5Hz
 * 0x03： 1Hz
 * 0x04： 2Hz
 * 0x05： 5Hz
 * 0x06： 10Hz	（default）
 * 0x07： 20Hz
 * 0x08： 50Hz
 * 0x09： 100Hz
 * 0x0a： 125Hz
 * 0x0b： 200Hz
 * 0x0c： Single
 * 0x0d: Not output
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Set_Return_Rate_e RATE;
    Zero_e Empty;
} RATE_Struct_t;

/*
 * Set baud rate messages structures.
 * 0x00： 2400
 * 0x01： 4800
 * 0x02： 9600	（default）
 * 0x03： 19200
 * 0x04： 38400
 * 0x05： 57600
 * 0x06： 115200
 * 0x07： 230400
 * 0x08： 460800
 * 0x09： 921600
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Set_Baud_Rate_e BAUD;
    Zero_e Empty;
} BAUD_Struct_t;

/*
 * Set Offset messages structures
 * Set Acceleration, Angular Velocity, Magnetic field offset in X-Y-Z Axis.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    uint8_t AGHxOFFSETL;
    uint8_t AGHxOFFSETH;
} AGHxOFFSET_Struct_t;

/*
 * Set Dx port mode messages structures.
 * 0x00：Analog Input（default）.
 * 0x01：Digital Input.
 * 0x02：Digital Output high.
 * 0x03：Digital Output low.
 * 0x04：PWM Output.
 * 0x05：Connect to TX of GPS (Available just for Port D1).
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Set_Port_Dx_Mode_e DxMODE;
    Zero_e Empty;
} DxMODE_Struct_t;

/*
 * Set Dx port PWM width messages structures
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    uint8_t DxPWMHL;
    uint8_t DxPWMHH;
} DxPWMH_Struct_t;

/*
 * Set Dx period of Port messages structures
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    uint8_t DxPWMTL;
    uint8_t DxPWMTH;
} DxPWMT_Struct_t;

/*
 * Set I2C Address messages structures
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    uint8_t IICADDR;
    Zero_e Empty;
} IICADDR_Struct_t;

/*
 * Set LED state messages structures.
 * 0x00： Turn on LED.
 * 0x01： Turn off LED.
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Set_LED_State_e LED;
    Zero_e Empty;
} LED_Struct_t;

/*
 * Set GPS baud rate messages structures.
 * 0x00：2400
 * 0x01： 4800
 * 0x02： 9600	（default）
 * 0x03： 19200
 * 0x04： 38400
 * 0x05： 57600
 * 0x06： 115200
 * 0x07： 230400
 * 0x08： 460800
 * 0x09： 921600
 */
typedef struct
{
    uint8_t First_Byte_Transmit;
    uint8_t Second_Byte_Transmit;
    Transmit_Message_Addr_e Address;
    Set_GPS_Baud_Rate_e GPSBAUD;
    Zero_e Empty;
} GPSBAUD_Struct_t;


/*
 * WT901T integrated AHRS Sensor, Transmit Messages Structure.
 * This structure consist of all setting message transmit to the AHRS sensor.
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
    AGHxOFFSET_Struct_t     AXOFFSET;
    AGHxOFFSET_Struct_t     AYOFFSET;
    AGHxOFFSET_Struct_t     AZOFFSET;
    AGHxOFFSET_Struct_t     GXOFFSET;
    AGHxOFFSET_Struct_t     GYOFFSET;
    AGHxOFFSET_Struct_t     GZOFFSET;
    AGHxOFFSET_Struct_t     HXOFFSET;
    AGHxOFFSET_Struct_t     HYOFFSET;
    AGHxOFFSET_Struct_t     HZOFFSET;
    DxMODE_Struct_t         D0MODE;
    DxMODE_Struct_t         D1MODE;
    DxMODE_Struct_t         D2MODE;
    DxMODE_Struct_t         D3MODE;
    DxPWMH_Struct_t         D0PWMH;
    DxPWMH_Struct_t         D1PWMH;
    DxPWMH_Struct_t         D2PWMH;
    DxPWMH_Struct_t         D3PWMH;
    DxPWMT_Struct_t         D0PWMT;
    DxPWMT_Struct_t         D1PWMT;
    DxPWMT_Struct_t         D2PWMT;
    DxPWMT_Struct_t         D3PWMT;
    IICADDR_Struct_t        IICADDR;
    LED_Struct_t            LED;
    GPSBAUD_Struct_t        GPSBAUD;
} WT901T_Transmit_Messages_Structure_t;

/*
 * Flag Structure
 * This structure is not a official.
 * This structure is custom struct and just used for auto sending data to module.
 */
typedef struct
{
    Bool    fSave;
    Bool    fCALSW;
    Bool    fDIRECTION;
    Bool    fSleep_WakeUp;
    Bool    fALG;
    Bool    fGYRO;
    Bool    fRSW;
    Bool    fRATE;
    Bool    fBAUD;
    Bool    fAXOFFSET;
    Bool    fAYOFFSET;
    Bool    fAZOFFSET;
    Bool    fGXOFFSET;
    Bool    fGYOFFSET;
    Bool    fGZOFFSET;
    Bool    fHXOFFSET;
    Bool    fHYOFFSET;
    Bool    fHZOFFSET;
    Bool    fD0MODE;
    Bool    fD1MODE;
    Bool    fD2MODE;
    Bool    fD3MODE;
    Bool    fD0PWMH;
    Bool    fD1PWMH;
    Bool    fD2PWMH;
    Bool    fD3PWMH;
    Bool    fD0PWMT;
    Bool    fD1PWMT;
    Bool    fD2PWMT;
    Bool    fD3PWMT;
    Bool    fIICADDR;
    Bool    fLED;
    Bool    fGPSBAUD;
} WT901T_Transmit_Messages_Flag_Structure_t;

/*
 * WT901T Messages Structures
 */
typedef struct
{
	WT901T_Receive_Messages_Struct_t		Receive_Messages;
	WT901T_Transmit_Messages_Structure_t	Transmit_Messages;
} WT901T_Message_Struct_t;

/*
 *   Function's structure definition
 */
static void WT901_Init(void);
void WT901_Transmit_Message(void);
void WT901_Receive_Message(void);
void WT901_Update_Message(void);


#endif /* INC_WT901T_H_ */
