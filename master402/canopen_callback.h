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
#ifndef __CANOPEN_CALLBACK_H
#define __CANOPEN_CALLBACK_H
/* Includes ------------------------------------------------------------------*/
#include "data.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
extern void master402_heartbeatError(CO_Data* d, UNS8 heartbeatID);
extern void master402_initialisation(CO_Data* d);
extern void master402_preOperational(CO_Data* d);
extern void master402_operational(CO_Data* d);
extern void master402_stopped(CO_Data* d);

extern void master402_post_sync(CO_Data* d);
extern void master402_post_TPDO(CO_Data* d);
extern void master402_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex);
extern void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5]);

extern void master402_fix_config_err(UNS8 nodeId);
#endif /* __CANOPEN_CALLBACK_H */


