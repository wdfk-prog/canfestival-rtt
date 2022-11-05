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
  * @brief  节点ID
  */  
typedef enum
{
  SERVO_NODEID_1 = 0x02,
  SERVO_NODEID_2,
  MAX_NODE_COUNT,         //最大节点数量
}NODEID_NUM;
/* Exported constants --------------------------------------------------------*/
#define CONTROLLER_NODEID 	  1//控制器ID
#define PDO_TRANSMISSION_TYPE 1//PDO传输类型

#define SDO_REPLY_TIMEOUT     50//50ms等待超时
#define PRODUCER_HEARTBEAT_TIME 500 //生产者心跳间隔
#define CONSUMER_HEARTBEAT_TIME 1000//消费者心跳间隔

#define ELECTRONIC_GEAR_RATIO_NUMERATOR 100000                      //电子齿轮比分子
#define ENCODER_RES           (16777216)                            //电子齿轮比分母 编码器分辨率 16,777,216
//电子齿轮比 6093h sub1/6093h sub2
#define ELECTRONIC_GEAR_RATIO (ENCODER_RES / ELECTRONIC_GEAR_RATIO_numerator)//电子齿轮比
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