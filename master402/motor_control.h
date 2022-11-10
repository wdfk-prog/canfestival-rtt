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
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "canfestival.h"
/* Exported types ------------------------------------------------------------*/
/*
 * INTEGER8地图变量结构体
*/
typedef struct
{
  INTEGER8 *map_val; //变量地址
  UNS16 index;   //变量索引
}Map_Val_INTEGER8;
/*
 * UNS16地图变量结构体
*/
typedef struct
{
  UNS16 *map_val; //变量地址
  UNS16 index;   //变量索引
}Map_Val_UNS16;
/*
 * UNS32地图变量结构体
*/
typedef struct
{
  UNS32 *map_val; //变量地址
  UNS16 index;   //变量索引
}Map_Val_UNS32;
/*
 * INTEGER32地图变量结构体
*/
typedef struct
{
  INTEGER32 *map_val; //变量地址
  UNS16 index;   //变量索引
}Map_Val_INTEGER32;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern Map_Val_UNS16 Controlword_Node[];
extern Map_Val_INTEGER8 Modes_of_operation_Node[];
/* Exported functions prototypes ---------------------------------------------*/
 
#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
