/**********************************************************************************************************
 * @文件     can.c
 * @说明     通过can总线发送和接收数据
 * @版本  	 V1.0
 * @作者     刘垚
 * @日期     2019.01
**********************************************************************************************************/
#include "can.h"
#include "string.h"

extern CAN_HandleTypeDef hcan;

/**********************************************************************************************************
*函 数 名:  CANSendMag
*功能说明:  通过can总线发送数据
*形    参:  data1 --- 发送数据前4位
			data2 --- 发送数据后4位
			id ------ can ID
*返 回 值:  1 --- 成功
			0 --- 失败
**********************************************************************************************************/
unsigned char CANSendMag(float *data1, float *data2, uint32_t id)
{	
    hcan.pTxMsg->StdId = id;        //标准标识符
    hcan.pTxMsg->ExtId = id;        //扩展标识符(29位)
    hcan.pTxMsg->IDE = CAN_ID_STD;    //使用标准帧
    hcan.pTxMsg->RTR = CAN_RTR_DATA;  //数据帧
    hcan.pTxMsg->DLC = 8;
   
    memcpy(hcan.pTxMsg->Data,data1,4);	
	memcpy(&hcan.pTxMsg->Data[4],data2,4);

    if(HAL_CAN_Transmit(&hcan,100)!=HAL_OK) return 1;     //发送
    return 0;		
}
/**********************************************************************************************************
*函 数 名:  CAN1_Receive_Msg
*功能说明:  can口接收数据查询
*形    参:  buf 接受到的数据
*返 回 值:  
**********************************************************************************************************/
unsigned char CAN1_Receive_Msg(unsigned char *buf)
{		   		   
 	unsigned char i;
    if(HAL_CAN_Receive(&hcan,CAN_FIFO0,0)!=HAL_OK) return 0;//接收数据，超时时间设置为0	
    for(i=0;i<hcan.pRxMsg->DLC;i++)
    buf[i]=hcan.pRxMsg->Data[i];
	
	return hcan.pRxMsg->DLC;	
}
