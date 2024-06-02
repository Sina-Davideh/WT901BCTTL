/*
 * WT901T.c
 *
 *  Created on: May 22, 2024
 *      Author: SINA
 */

#include "wt901t.h"

//Transmit_Messages_Structure_t Transmit_Messages;
WT901T_Message_Struct_t WT901T;
WT901T_Transmit_Messages_Flag_Structure_t WT901T_Flag;

static void WT901_Init(void){
	WT901T_Flag = Snd_Flag;
}

void WT901_Transmit_Message(void){
	WT901_Init();
    uint8_t WT901_TX_Buffer[5] = {0x00};
//    Transmit_Message_Addr_e Transmit_Message_Addr;

//    Transmit_Buffer[1] = First_Byte_Transmit_Data_Pack;
//    Transmit_Buffer[2] = Second_Byte_Transmit_Data_Pack;

//    WT901_TX_Buffer[1] = Transmit_msg->First_Byte_Transmit;
//    WT901_TX_Buffer[2] = Transmit_msg->Second_Byte_Transmit;
//    WT901_TX_Buffer[3] = Transmit_msg->Address;
//    WT901_TX_Buffer[4] = Transmit_msg->DataL;
//    WT901_TX_Buffer[5] = Transmit_msg->DataH;

    while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);

    if(WT901T_Flag.fCALSW == True){
    	WT901T_Flag.fSave == True;
    	WT901_TX_Buffer[3] = WT901T.Transmit_Messages.CALSW.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.CALSW.CALSW;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.CALSW.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fCALSW == False;
    }

    if(WT901T_Flag.fDIRECTION == True){
    	WT901T_Flag.fSave == True;
    	WT901_TX_Buffer[3] = WT901T.Transmit_Messages.DIRECTION.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.DIRECTION.DIRECTION;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.DIRECTION.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fDIRECTION == False;
    }

    if(WT901T_Flag.fSleep_WakeUp == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.Sleep_WakeUp.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.Sleep_WakeUp.Sleep_WakeUp;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.Sleep_WakeUp.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fSleep_WakeUp == False;
	}

    if(WT901T_Flag.fALG == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.ALG.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.ALG.ALG;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.ALG.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fALG == False;
	}

    if(WT901T_Flag.fGYRO == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.GYRO.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.GYRO.GYRO;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.GYRO.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fGYRO == False;
	}

    if(WT901T_Flag.fRSW == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.RSW.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.RSW.RSWL.data.all;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.RSW.RSWH.data.all;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fRSW == False;
	}

    if(WT901T_Flag.fRATE == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.RATE.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.RATE.RATE;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.RATE.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fRATE == False;
	}

    if(WT901T_Flag.fBAUD == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.BAUD.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.BAUD.BAUD;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.BAUD.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fBAUD == False;
	}

    if(WT901T_Flag.fAXOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.AXOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.AXOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.AXOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fAXOFFSET == False;
	}

    if(WT901T_Flag.fAYOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.AYOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.AYOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.AYOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fAYOFFSET == False;
	}

    if(WT901T_Flag.fAZOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.AZOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.AZOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.AZOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fAZOFFSET == False;
	}

    if(WT901T_Flag.fGXOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.GXOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.GXOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.GXOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fGXOFFSET == False;
	}
    
	if(WT901T_Flag.fGYOFFSET == True){
		WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.GYOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.GYOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.GYOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fGYOFFSET == False;
	}

    if(WT901T_Flag.fGZOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.GZOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.GZOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.GZOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fGZOFFSET == False;
	}

    if(WT901T_Flag.fHXOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.HXOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.HXOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.HXOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fHXOFFSET == False;
	}

    if(WT901T_Flag.fHYOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.HYOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.HYOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.HYOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fHYOFFSET == False;
	}

    if(WT901T_Flag.fHZOFFSET == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.HZOFFSET.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.HZOFFSET.AGHxOFFSETL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.HZOFFSET.AGHxOFFSETH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fHZOFFSET == False;
	}

    if(WT901T_Flag.fD0MODE == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D0MODE.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D0MODE.DxMODE;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D0MODE.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD0MODE == False;
	}

    if(WT901T_Flag.fD1MODE == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D1MODE.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D1MODE.DxMODE;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D1MODE.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD1MODE == False;
	}

    if(WT901T_Flag.fD2MODE == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D2MODE.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D2MODE.DxMODE;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D2MODE.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD2MODE == False;
	}

    if(WT901T_Flag.fD3MODE == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D3MODE.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D3MODE.DxMODE;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D3MODE.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD3MODE == False;
	}

    if(WT901T_Flag.fD0PWMH == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D0PWMH.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D0PWMH.DxPWMHL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D0PWMH.DxPWMHH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD0PWMH == False;
	}

    if(WT901T_Flag.fD1PWMH == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D1PWMH.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D1PWMH.DxPWMHL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D1PWMH.DxPWMHH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD1PWMH == False;
	}

    if(WT901T_Flag.fD2PWMH == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D2PWMH.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D2PWMH.DxPWMHL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D2PWMH.DxPWMHH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD2PWMH == False;
	}

    if(WT901T_Flag.fD3PWMH == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D3PWMH.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D3PWMH.DxPWMHL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D3PWMH.DxPWMHH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD3PWMH == False;
	}

    if(WT901T_Flag.fD0PWMT == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D0PWMT.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D0PWMT.DxPWMTL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D0PWMT.DxPWMTH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD0PWMT == False;
	}
	
    if(WT901T_Flag.fD1PWMT == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D1PWMT.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D1PWMT.DxPWMTL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D1PWMT.DxPWMTH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
		WT901T_Flag.fD1PWMT == False;
	}

    if(WT901T_Flag.fD2PWMT == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D2PWMT.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D2PWMT.DxPWMTL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D2PWMT.DxPWMTH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fD2PWMT == False;
	}

    if(WT901T_Flag.fD3PWMT == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.D3PWMT.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.D3PWMT.DxPWMTL;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.D3PWMT.DxPWMTH;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fD3PWMT == False;
	}

    if(WT901T_Flag.fIICADDR == True){
    	WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.IICADDR.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.IICADDR.IICADDR;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.IICADDR.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fIICADDR == False;
	}
    
	if(WT901T_Flag.fLED == True){
		WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.LED.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.LED.LED;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.LED.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fLED == False;
	}
    
	if(WT901T_Flag.fGPSBAUD == True){
		WT901T_Flag.fSave == True;
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.GPSBAUD.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.GPSBAUD.GPSBAUD;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.GPSBAUD.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fGPSBAUD == False;
	}
    
	if(WT901T_Flag.fSave == True){
		WT901_TX_Buffer[3] = WT901T.Transmit_Messages.Save.Address;
    	WT901_TX_Buffer[4] = WT901T.Transmit_Messages.Save.Save;
    	WT901_TX_Buffer[5] = WT901T.Transmit_Messages.Save.Empty;
    	while(HAL_UART_Transmit_DMA(&huart3, WT901_TX_Buffer, sizeof(WT901_TX_Buffer)) != HAL_OK);
    	WT901T_Flag.fSave == False;
	}
}

void WT901_Receive_Message(void){
//	uint8_t Transmit_Buffer[5] = {0x00};
//	uint8_t Receive_Buffer[11] = {0x00};
//
//	while(HAL_UART_Transmit_DMA(huart, Transmit_Buffer, 5) != HAL_OK);
//
//	while(HAL_UART_Receive_DMA(huart, Receive_Buffer, 11) != HAL_OK);
//
//	Receive_msg->First_Byte_Receive = Receive_Buffer[0];
//	Receive_msg->Message_Address_Receive = Receive_Buffer[1];
//	Receive_msg->Data_Byte_0 = Receive_Buffer[2];
//	Receive_msg->Data_Byte_1 = Receive_Buffer[3];
//	Receive_msg->Data_Byte_2 = Receive_Buffer[4];
//	Receive_msg->Data_Byte_3 = Receive_Buffer[5];
//	Receive_msg->Data_Byte_4 = Receive_Buffer[6];
//	Receive_msg->Data_Byte_5 = Receive_Buffer[7];
//	Receive_msg->Data_Byte_6 = Receive_Buffer[8];
//	Receive_msg->Data_Byte_7 = Receive_Buffer[9];
//	Receive_msg->Checksum = Receive_Buffer[10];
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
			  WT901T.Receive_Messages.Time.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Time.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Time.YY 			= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Time.MM 			= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Time.DD 			= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Time.hh 			= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Time.mm 			= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Time.ss 			= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Time.msL 			= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Time.msH 			= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Time.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Acceleration:
				WT901T.Receive_Messages.Acceleration.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
				WT901T.Receive_Messages.Acceleration.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
				WT901T.Receive_Messages.Acceleration.AxL		= Receive_Message_Temporary_Buffer.Data_Byte_0;
				WT901T.Receive_Messages.Acceleration.AxH		= Receive_Message_Temporary_Buffer.Data_Byte_1;
				WT901T.Receive_Messages.Acceleration.AyL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
				WT901T.Receive_Messages.Acceleration.AyH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
				WT901T.Receive_Messages.Acceleration.AzL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
				WT901T.Receive_Messages.Acceleration.AzH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
				WT901T.Receive_Messages.Acceleration.TL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
				WT901T.Receive_Messages.Acceleration.TH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
				WT901T.Receive_Messages.Acceleration.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Angular_Velocity:
			  WT901T.Receive_Messages.Angular_Velocity.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Angular_Velocity.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Angular_Velocity.WxL			= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Angular_Velocity.WxH			= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Angular_Velocity.WyL			= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Angular_Velocity.WyH			= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Angular_Velocity.WzL			= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Angular_Velocity.WzH			= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Angular_Velocity.TL 			= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Angular_Velocity.TH 			= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Angular_Velocity.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Angle:
			  WT901T.Receive_Messages.Angle.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Angle.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Angle.RollL		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Angle.RollH		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Angle.PitchL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Angle.PitchH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Angle.YawL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Angle.YawH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Angle.TL 			= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Angle.TH 			= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Angle.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Magnetic:
			  WT901T.Receive_Messages.Magnetic.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Magnetic.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Magnetic.HxL			= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Magnetic.HxH			= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Magnetic.HyL			= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Magnetic.HyH			= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Magnetic.HzL			= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Magnetic.HzH			= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Magnetic.TL 			= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Magnetic.TH 			= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Magnetic.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Data_Port_Status:
			  WT901T.Receive_Messages.Data_Port_Status.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Data_Port_Status.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Data_Port_Status.D0L			= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Data_Port_Status.D0H			= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Data_Port_Status.D1L			= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Data_Port_Status.D1H			= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Data_Port_Status.D2L			= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Data_Port_Status.D2H			= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Data_Port_Status.D3L 			= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Data_Port_Status.D3H 			= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Data_Port_Status.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Atmospheric_Pressure_Height:
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.Msg_Begin		= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.Msg_Addr   	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.P0		 	= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.P1		 	= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.P2		 	= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.P3		 	= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.H0		 	= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.H1		 	= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.H2 		 	= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.H3 		 	= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Atmospheric_Pressure_Height.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Longitude_Latitude:
			  WT901T.Receive_Messages.Longitude_Latitude.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Longitude_Latitude.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Longitude_Latitude.Lon0		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Longitude_Latitude.Lon1		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Longitude_Latitude.Lon2		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Longitude_Latitude.Lon3		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Longitude_Latitude.Lat0		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Longitude_Latitude.Lat1		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Longitude_Latitude.Lat2		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Longitude_Latitude.Lat3 	 	= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Longitude_Latitude.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Ground_Speed:
			  WT901T.Receive_Messages.Ground_Speed.Msg_Begin 	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Ground_Speed.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Ground_Speed.GPSHeightL	= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Ground_Speed.GPSHeightH	= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Ground_Speed.GPSYawL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Ground_Speed.GPSYawH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Ground_Speed.GPSV0		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Ground_Speed.GPSV1		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Ground_Speed.GPSV2 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Ground_Speed.GPSV3 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Ground_Speed.SUM 			= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Quaternion:
			  WT901T.Receive_Messages.Quaternion.Msg_Begin  = Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Quaternion.Msg_Addr   = Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Quaternion.Q0L		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Quaternion.Q0H		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Quaternion.Q1L		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Quaternion.Q1H		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Quaternion.Q2L		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Quaternion.Q2H		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Quaternion.Q3L 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Quaternion.Q3H 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Quaternion.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  case Satellite_Positioning_Accuracy:
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.Msg_Begin	= Receive_Message_Temporary_Buffer.First_Byte_Receive;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.Msg_Addr  	= Receive_Message_Temporary_Buffer.Message_Address_Receive;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.SNL		= Receive_Message_Temporary_Buffer.Data_Byte_0;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.SNH		= Receive_Message_Temporary_Buffer.Data_Byte_1;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.PDOPL		= Receive_Message_Temporary_Buffer.Data_Byte_2;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.PDOPH		= Receive_Message_Temporary_Buffer.Data_Byte_3;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.HDOPL		= Receive_Message_Temporary_Buffer.Data_Byte_4;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.HDOPH		= Receive_Message_Temporary_Buffer.Data_Byte_5;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.VDOPL 		= Receive_Message_Temporary_Buffer.Data_Byte_6;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.VDOPH 		= Receive_Message_Temporary_Buffer.Data_Byte_7;
			  WT901T.Receive_Messages.Satellite_Positioning_Accuracy.SUM 		= Receive_Message_Temporary_Buffer.Checksum;
			break;

		  default:
		// code block
    	}
	}
}

