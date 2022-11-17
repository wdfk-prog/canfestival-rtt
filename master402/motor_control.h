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
#include <stdbool.h>
/* Exported types ------------------------------------------------------------*/
/*0x6040����ָ�� Controlword ״̬λ����*/
typedef enum
{
  WRITE_SWITCH_ON             = 1 << 0,//���¿���
  EN_VOLTAGE                  = 1 << 1,//ʹ�ܵ�Դ
  QUICK_STOP                  = 1 << 2,//��ͣ
  EN_OPERATION                = 1 << 3,//����ʹ��
  FAULT_RESET                 = 1 << 7,//��������
  HALT                        = 1 << 8,//��ͣ
}CONTROL_WORD;
/*0x6041״̬λ Statusword����*/
typedef enum
{
  READY_TO_SWITCH_ON          = 0,//׼����������
  READ_SWITCH_ON              = 1,//�ŷ�׼�����
  OPERATION_ENABLED           = 2,//�ŷ�ʹ��
  FAULT                       = 3,//�쳣�ź�
  VOLTAGE_ENABLED             = 4,//�ŷ�������ѹ���
  READ_QUICK_STOP             = 5,//����ֹͣ [0:������ͣ 1:�رռ�ͣ]
  SWITCH_ON_DISABLED          = 6,//�ŷ�׼�����ܹر�
  WARNING                     = 7,//�����ź�
  REMOTE                      = 9,//Զ�̿���
  TARGET_REACHED              = 10,//Ŀ�굽��
  POSITIVE_LIMIT              = 14,//������ת��ֹ����
  NEGATIVE_LIMIT              = 15,//������ת��ֹ����
}STATUS_WORD;
/*0X6060 ģʽ�趨Modes of operation����*/
typedef enum
{
  PROFILE_POSITION_MODE       = 1,//λ�ù滮ģʽ
  PROFILE_VELOCITY_MODE       = 3,//�ٶȹ滮ģʽ
  PROFILE_TORQUE_MODE         = 4,//Ť�ع滮ģʽ
  HOMING_MODE                 = 6,//ԭ�㸴��ģʽ
  INTERPOLATED_POSITION_MODE  = 7,//�岹λ��ģʽ
}MODE_OPERATION;
/*λ�ù滮ģʽ��Controlword����ģʽ�ض�λ*/
typedef enum
{
  NEW_SET_POINT               = 1 << 4,//�����(��Ե����)
  CHANGE_SET_IMMEDIATELY      = 1 << 5,//����������Чָ��  ��Ϊ 0���ر�����������Чָ��
  ABS_REL                     = 1 << 6,//����Ϊ���Զ�λ����Զ�λ  0��Ŀ��λ����һ������ֵ 1��Ŀ��λ����һ�����ֵ
}PROFILE_POSITION_CONTROLWORD;
/*λ�ù滮ģʽ��Statusword����*/
typedef enum
{
  SET_POINT_ACKNOWLEDGE       = 12,//�ŷ��յ������ź�
  FOLLOWING_ERROR             = 13,//׷�����
}PROFILE_POSITION_STATUSWORD;
/*�岹λ��ģʽ��Controlword����ģʽ�ض�λ
  Name          Value   Description
Enable ip mode    0     Interpolated position mode inactive 
                  1     Interpolated position mode active*/
typedef enum
{
   ENABLE_IP_MODE             = 1 << 4,//ʹ��IPģʽ
}Interpolated_Position_CONTROLWORD;
/*�岹λ��ģʽ��Statusword����
    Name        Value   Description
ip mode active    1     Interpolated position mode active
*/
typedef enum
{
  IP_MODE_ACTIVE              = 12,//�岹λ��ģʽ�Ƿ���Ч
}Interpolated_Position_STATUSWORD;
/*ԭ�㸴��ģʽ��Controlword����ģʽ�ض�λ
  Name          Value   Description
Homing          0       Homing mode inactive
operation       0 �� 1  Start homing mode
star            1       Homing mode active
                1 �� 0  Interrupt homing mode*/
typedef enum
{
  HOMING_OPERATION_STAR       = 1 << 4,//ʹ��IPģʽ
}HOMING_CONTROLWORD;
/*ԭ�㸴��ģʽ��Statusword����
    Name          Value   Description
Homing attained    0      Homing mode not yet completed.
                   1      Homing mode carried out successfully

Homing error       0      No homing error
                   1      Homing error occurred;Homing mode carried out not successfully;The error cause is found by reading the error code
*/
typedef enum
{
  HOMING_ATTAINED              = 12,//�ص�ԭ��
  HOMING_ERROR                 = 13,//��ԭ����
}HOMING_STATUSWORD;

/*
 * INTEGER8��ͼ�����ṹ��
*/
typedef struct
{
  INTEGER8 *map_val; //������ַ
  UNS16 index;   //��������
}Map_Val_INTEGER8;
/*
 * UNS16��ͼ�����ṹ��
*/
typedef struct
{
  UNS16 *map_val; //������ַ
  UNS16 index;   //��������
}Map_Val_UNS16;
/*
 * UNS32��ͼ�����ṹ��
*/
typedef struct
{
  UNS32 *map_val; //������ַ
  UNS16 index;   //��������
}Map_Val_UNS32;
/*
 * INTEGER32��ͼ�����ṹ��
*/
typedef struct
{
  INTEGER32 *map_val; //������ַ
  UNS16 index;   //��������
}Map_Val_INTEGER32;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern Map_Val_UNS16 Controlword_Node[];
extern Map_Val_INTEGER8 Modes_of_operation_Node[];
/* Exported functions prototypes ---------------------------------------------*/
extern UNS8 motor_on_profile_position(UNS8 nodeId);
extern UNS8 motor_on_interpolated_position(UNS8 nodeId);
extern UNS8 motor_on_homing_mode(int32_t offset,uint8_t method,float switch_speed,float zero_speed,UNS8 nodeId);
extern UNS8 motor_on_profile_velocity(UNS8 nodeId);
/******************************�˶�ģʽ��������******************************************************************/
extern UNS8 motor_profile_position(int32_t position,int16_t speed,bool abs_rel,bool immediately,UNS8 nodeId);
extern UNS8 motor_interpolation_position (UNS8 nodeId);
extern UNS8 motor_homing_mode (bool zero_flag,int16_t speed,UNS8 nodeId);
extern UNS8 motor_profile_velocity(int16_t speed,UNS8 nodeId);
/******************************�˶��رռ���ѯ����******************************************************************/
extern UNS8 motor_off(UNS8 nodeId);
extern UNS16 motor_get_controlword(UNS8 nodeId);
extern UNS16 motor_get_statusword(UNS8 nodeId);
extern INTEGER32 *motor_get_position(INTEGER32* des,UNS8 nodeId);
extern INTEGER32 *motor_get_velocity(INTEGER32* des,UNS8 nodeId);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
