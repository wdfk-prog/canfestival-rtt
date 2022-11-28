/**
 * @file master402_canopen.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention 
 * @par �޸���־:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MASTER402_CANOPEN_H
#define __MASTER402_CANOPEN_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "master402_od.h"
#include "motor.h"
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
/** 
  * @brief  �ڵ�������
  */  
typedef enum
{
  NODEID_CONFIG_SUCCESS,      //�ڵ����óɹ�
  NODEID_BACK_ONLINE,         //�ڵ���������
  NODEID_CONFIG_NO_RESPOND,   //���ûظ�δ��Ӧ���ڵ��ֵ����
  NODEID_CONFIG_NO_SEND,      //����δ����,�����ֵ����
}NODEID_ERRCODE;

/* Exported constants --------------------------------------------------------*/
#define MASTER_NODEID 	  1//������ID
#define PDO_TRANSMISSION_TYPE 1//PDO��������

#define SDO_REPLY_TIMEOUT     50//50ms�ȴ���ʱ
#define PRODUCER_HEARTBEAT_TIME 500 //�������������
#define CONSUMER_HEARTBEAT_TIME 1000//�������������
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern CO_Data *OD_Data;
/* Exported functions prototypes ---------------------------------------------*/
extern void config_node(uint8_t nodeId);
extern void canopen_start_thread_entry(void *parameter);
extern UNS8 Write_SLAVE_control_word(UNS8 nodeId,UNS16 value);
extern UNS8 Write_SLAVE_Modes_of_operation(UNS8 nodeId,INTEGER8 mode);
extern UNS8 Write_SLAVE_profile_position_speed_set(UNS8 nodeId,float speed);
extern UNS8 Write_SLAVE_Interpolation_time_period(UNS8 nodeId);
extern UNS8 Write_SLAVE_Homing_set(UNS8 nodeId,UNS32 offset,UNS8 method,float switch_speed,float zero_speed);
/********************�ڵ���Ϣ��ѯ�����****************************************/
extern char *nodeID_get_name(char* des,uint8_t nodeid);
extern e_nodeState nodeID_get_nmt(uint8_t nodeid);
extern UNS8 nodeID_set_errcode(uint8_t nodeid,uint16_t errcode);
extern uint16_t nodeID_get_errcode(uint8_t nodeid);
extern UNS8 nodeID_set_errSpec(uint8_t nodeID,const uint8_t errSpec[5]);
extern char* nodeID_get_errSpec(char* des,uint8_t nodeID);
extern motor_config* nodeID_get_config(motor_config* des,uint8_t nodeID);

#ifdef __cplusplus
}
#endif

#endif /* __MASTER402_CANOPEN_H */