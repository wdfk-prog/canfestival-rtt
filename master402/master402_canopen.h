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
#define CONTROLLER_NODEID 	  1//������ID
#define SERVO_NODEID 		      2//�ŷ�ID
#define PDO_TRANSMISSION_TYPE 1//PDO��������

#define PRODUCER_HEARTBEAT_TIME 500 //�������������
#define CONSUMER_HEARTBEAT_TIME 1000//�������������
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern CO_Data *OD_Data;
/* Exported functions prototypes ---------------------------------------------*/
extern void canopen_start_thread_entry(void *parameter);
#endif /* __AGV_CANOPEN_H__ */