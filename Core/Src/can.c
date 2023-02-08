/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "usr_moto.h"








//筛选器配置
void CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef CAN1_FilerConf;

    CAN1_FilerConf.FilterIdHigh=0X0000;					   //具体Id要求高16位
    CAN1_FilerConf.FilterIdLow=0X0000;					   //具体Id要求低16位
    CAN1_FilerConf.FilterMaskIdHigh=0X0000;                //掩码高16位全设置为0，表示对所有位报文Id高16位都不关心
    CAN1_FilerConf.FilterMaskIdLow=0X0000;                 //掩码低16位全设置为0，表示对所有位报文Id低16位都不关心
    CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;  //筛选器接收到的报文放入到FIFO0中，即为接收邮箱0
    CAN1_FilerConf.FilterActivation=ENABLE;                //筛选器使能（开启）
    CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;       //筛选器掩码模式
    CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;      //掩码用32位表示

    CAN1_FilerConf.FilterBank=0;
    CAN1_FilerConf.SlaveStartFilterBank=14;

    if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK)
    {
        Error_Handler();
    }
}

void usr_can_Tx(unsigned char canid, unsigned char * pdata, unsigned char len){
    uint32_t pTxMailbox = 0 ;
    CAN_TxHeaderTypeDef  pTxMsg;
    pTxMsg.StdId = canid;
    pTxMsg.IDE = CAN_ID_STD;
    pTxMsg.RTR = CAN_RTR_DATA;
    pTxMsg.DLC = len;
    HAL_CAN_AddTxMessage(&hcan1,&pTxMsg, pdata, &pTxMailbox);
}

CAN_RxHeaderTypeDef    RxMessage ;
uint8_t can_data_left[8], can_data_right[8] ;  // 存放伺服电机回的数据
short int right_rpm, left_rpm,right_get_flag, left_get_flag;
uint8_t  aRxData[8] ;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    uint8_t  i;
    if(hcan->Instance == CAN1)	 {
        if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, aRxData) == HAL_OK) {
            if(RxMessage.StdId == MOTO1) {  //MOTO1
                if(aRxData[1] == WRITERET  && aRxData[2] == MOTOSTARTSTOP && aRxData[5] == SPEEDSET) {
                    for(i=0; i<8; i++) {
                        can_data_left[i] = aRxData[i];
                    }
                }//if(aRxData[1] == REPORT
                else if(aRxData[1] == REPORT && aRxData[2] == AMPFEEDBACK && aRxData[5] == SPEEDFEEDBACK){
                    left_rpm = ((aRxData[7] | (aRxData[6] << 8)));//left_rpm = left_rpm;// * 3000 / 8192;						left_rpm = 0 - left_rpm; //return data=16384, rotate speed is 6000RMP.  (3000/8192=0.366)  RMP --> r/s
                    left_get_flag = 1;
                }
            }//if(RxMessage.StdId == 0x01
            else if(RxMessage.StdId == MOTO2) { //MOTO2
                if(aRxData[1] == WRITERET  && aRxData[2] == MOTOSTARTSTOP && aRxData[5] == SPEEDSET) {	//write ret
                    for(i=0; i<8; i++) {
                        can_data_right[i] = aRxData[i];
                    }
                }//if(aRxData[1] == REPORT
                else if(aRxData[1] == REPORT && aRxData[2] == AMPFEEDBACK && aRxData[5] == SPEEDFEEDBACK){
                    right_rpm = ((aRxData[7] | (aRxData[6] << 8)));//right_rpm = right_rpm;// * 3000 / 8192;//转速读出值转成实际值
                    right_get_flag = 1;
//						HAL_GPIO_TogglePin(Beep_GPIO_Port, Beep_Pin);
                }
            }//else if(RxMessage.StdId == 0x02
        }//if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, aRxData) == HAL_OK)
    }//if(hcan->Instance == CAN1
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
