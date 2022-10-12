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

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define CONTROLLER_NODEID 	  1//控制器ID
#define SERVO_NODEID 		      2//伺服ID
#define PDO_TRANSMISSION_TYPE 1//PDO传输类型

#define PRODUCER_HEARTBEAT_TIME 500 //生产者心跳间隔
#define CONSUMER_HEARTBEAT_TIME 1000//消费者心跳间隔
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern CO_Data *OD_Data;
/* Exported functions prototypes ---------------------------------------------*/
extern void canopen_start_thread_entry(void *parameter);
#endif /* __AGV_CANOPEN_H__ */