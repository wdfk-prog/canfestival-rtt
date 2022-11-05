/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  * @author
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGV_CANOPEN_H__
#define __AGV_CANOPEN_H__
/* Includes ------------------------------------------------------------------*/
#include "master402_od.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  �ڵ�ID
  */  
typedef enum
{
  SERVO_NODEID_1 = 0x02,
  SERVO_NODEID_2,
  MAX_NODE_COUNT,         //���ڵ�����
}NODEID_NUM;
/* Exported constants --------------------------------------------------------*/
#define CONTROLLER_NODEID 	  1//������ID
#define PDO_TRANSMISSION_TYPE 1//PDO��������

#define SDO_REPLY_TIMEOUT     50//50ms�ȴ���ʱ
#define PRODUCER_HEARTBEAT_TIME 500 //�������������
#define CONSUMER_HEARTBEAT_TIME 1000//�������������

#define ELECTRONIC_GEAR_RATIO_NUMERATOR 100000                      //���ӳ��ֱȷ���
#define ENCODER_RES           (16777216)                            //���ӳ��ֱȷ�ĸ �������ֱ��� 16,777,216
//���ӳ��ֱ� 6093h sub1/6093h sub2
#define ELECTRONIC_GEAR_RATIO (ENCODER_RES / ELECTRONIC_GEAR_RATIO_numerator)//���ӳ��ֱ�
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern CO_Data *OD_Data;
/* Exported functions prototypes ---------------------------------------------*/
extern void config_node(uint8_t nodeId);
extern void canopen_start_thread_entry(void *parameter);
extern UNS8 Write_SLAVE_control_word(UNS8 nodeId,UNS16 value);
extern UNS8 Write_SLAVE_Modes_of_operation(UNS8 nodeId,INTEGER8 mode);
extern UNS8 Write_SLAVE_profile_position_speed_set(UNS8 nodeId,UNS32 speed);
extern UNS8 Write_SLAVE_Interpolation_time_period(UNS8 nodeId);
extern UNS8 Write_SLAVE_Homing_set(UNS8 nodeId,UNS32 offset,UNS8 method,float switch_speed,float zero_speed);
#endif /* __AGV_CANOPEN_H__ */