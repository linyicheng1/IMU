/**********************************************************************************************************
 * @�ļ�     can.c
 * @˵��     ͨ��can���߷��ͺͽ�������
 * @�汾  	 V1.0
 * @����     ����
 * @����     2019.01
**********************************************************************************************************/
#include "can.h"
#include "string.h"

extern CAN_HandleTypeDef hcan;

/**********************************************************************************************************
*�� �� ��:  CANSendMag
*����˵��:  ͨ��can���߷�������
*��    ��:  data1 --- ��������ǰ4λ
			data2 --- �������ݺ�4λ
			id ------ can ID
*�� �� ֵ:  1 --- �ɹ�
			0 --- ʧ��
**********************************************************************************************************/
unsigned char CANSendMag(float *data1, float *data2, uint32_t id)
{	
    hcan.pTxMsg->StdId = id;        //��׼��ʶ��
    hcan.pTxMsg->ExtId = id;        //��չ��ʶ��(29λ)
    hcan.pTxMsg->IDE = CAN_ID_STD;    //ʹ�ñ�׼֡
    hcan.pTxMsg->RTR = CAN_RTR_DATA;  //����֡
    hcan.pTxMsg->DLC = 8;
   
    memcpy(hcan.pTxMsg->Data,data1,4);	
	memcpy(&hcan.pTxMsg->Data[4],data2,4);

    if(HAL_CAN_Transmit(&hcan,100)!=HAL_OK) return 1;     //����
    return 0;		
}
/**********************************************************************************************************
*�� �� ��:  CAN1_Receive_Msg
*����˵��:  can�ڽ������ݲ�ѯ
*��    ��:  buf ���ܵ�������
*�� �� ֵ:  
**********************************************************************************************************/
unsigned char CAN1_Receive_Msg(unsigned char *buf)
{		   		   
 	unsigned char i;
    if(HAL_CAN_Receive(&hcan,CAN_FIFO0,0)!=HAL_OK) return 0;//�������ݣ���ʱʱ������Ϊ0	
    for(i=0;i<hcan.pRxMsg->DLC;i++)
    buf[i]=hcan.pRxMsg->Data[i];
	
	return hcan.pRxMsg->DLC;	
}
