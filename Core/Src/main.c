/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "WT901BCTTL.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t First_Byte;
	uint8_t Second_Byte;
	uint8_t YY;
	uint8_t MM;
	uint8_t DD;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
	uint8_t msL;
	uint8_t msH;
	uint8_t SUM;
} WT901_Time_Str_t;

typedef struct{
	uint8_t First_Byte;
	uint8_t Second_Byte;
	uint8_t AxL;
	uint8_t AxH;
	uint8_t AyL;
	uint8_t AyH;
	uint8_t AzL;
	uint8_t AzH;
	uint8_t TL;
	uint8_t TH;
	uint8_t SUM;
} WT901_Acceleration_Str_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
//uint8_t CH340_TX_Buffer[64] = "ch340 test message\n";
uint8_t CH340_TX_Buffer[64] = {0x00};
uint8_t CH340_RX_Buffer[64];

//uint8_t WT901_TX_Buffer[5] = {0xff, 0xaa, 0x02, 0x01, 0x00};
uint8_t WT901_TX_Buffer[5] = {0xff, 0xaa, 0x02, 0x02, 0x00};
uint8_t WT901_RX_Buffer[11];


uint8_t WT901_TX_Save[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t WT901_TX_ALG[5] = {0xFF, 0xAA, 0x24, 0x00, 0x00};
uint8_t WT901_TX_GYRO[5] = {0xFF, 0xAA, 0x63, 0x00, 0x00};
uint8_t WT901_TX_Ret[5] = {0xFF, 0xAA, 0x02, 0x5E, 0x00};

//uint8_t CDC_TX_Buffer[64];
//uint8_t CDC_RX_Buffer[64];

WT901_Time_Str_t WT901_Time;
WT901_Acceleration_Str_t WT901_Acceleration;
Module_Output_Data_Struct_t WT901_Out_msg;

uint16_t Validate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
static void WT901_Update_Message(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}

	if(huart->Instance == USART3){
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		WT901_Update_Message();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}

	if(huart->Instance == USART3){
		WT901_Update_Message();
		for(uint8_t i = 0; i < 11; i++){
			CH340_TX_Buffer[i] = WT901_RX_Buffer[i];
		}
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}
}

//void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
//	if(huart->Instance == USART1){}
//	if(huart->Instance == USART3){}
//}

//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
//	if(huart->Instance == USART1){}
//	if(huart->Instance == USART3){}
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer));

  HAL_UART_Transmit_DMA(&huart3, WT901_TX_ALG, sizeof(WT901_TX_ALG));
  HAL_UART_Transmit_DMA(&huart3, WT901_TX_GYRO, sizeof(WT901_TX_GYRO));
  HAL_UART_Transmit_DMA(&huart3, WT901_TX_Ret, sizeof(WT901_TX_Ret));
  HAL_UART_Transmit_DMA(&huart3, WT901_TX_Save, sizeof(WT901_TX_Save));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
	  HAL_UART_Transmit_DMA(&huart1, CH340_TX_Buffer, sizeof(CH340_TX_Buffer));
//	  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

static void WT901_Update_Message(void){
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : KEY1_Pin KEY0_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
