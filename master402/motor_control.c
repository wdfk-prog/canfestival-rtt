/**
 * @file motor_control.c
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
/* Private includes ----------------------------------------------------------*/
#include <stdlib.h>
#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#include "canfestival.h"
#include "timers_driver.h"
#include "master402_od.h"
#include "master402_canopen.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/*ʹ�������ӳ�ȥ���Ĳ��������ܱ�֤�����Ѿ�����
���Ĳ���ģʽ������ʹ��0X6061�鿴ģʽ��ȡ��֤���ĳɹ�
�����̣߳��жϳ�ʱ���ָ�����
*/
#define SYNC_DELAY            rt_thread_mdelay(20)//������ʱ
#define MAX_WAIT_TIME         5000                //ms

/*0X6040 ����ָ�� Controlword ״̬��*/
//�ŷ� Servo Off
#define CONTROL_WORD_SHUTDOWN         (EN_VOLTAGE | QUICK_STOP) & (~WRITE_SWITCH_ON & ~FAULT_RESET)
//�ŷ�Servo On
#define CONTROL_WORD_SWITCH_ON        (WRITE_SWITCH_ON | EN_VOLTAGE | QUICK_STOP) & (~EN_OPERATION)  
//ִ���˶�ģʽ
#define CONTROL_WORD_ENABLE_OPERATION (WRITE_SWITCH_ON | EN_VOLTAGE | QUICK_STOP | EN_OPERATION & ~FAULT_RESET)
//�رչ���
#define CONTROL_WORD_DISABLE_VOLTAGE  (0 & ~EN_VOLTAGE & ~FAULT_RESET) 
//��ͣ
#define CONTROL_WORD_QUICK_STOP       (EN_VOLTAGE & ~QUICK_STOP & ~FAULT_RESET)
/* Private macro -------------------------------------------------------------*/
#define	CANOPEN_GET_BIT(x, bit)	  ((x &   (1 << bit)) >> bit)	/* ��ȡ��bitλ */
/*�ж�����ֵ��Ϊ0X00�˳���ǰ������*/
#define FAILED_EXIT(CODE){  \
if(CODE != 0X00)            \
   return 0XFF;             \
}
/*�жϵ�ǰ�����ڵ��Ƿ�������״̬��*/
#define NODE_DECISION {  \
if(OD_Data->NMTable[nodeId] != Operational){ \
  LOG_E("The current node %d is not in operation and is in 0X%02X state", \
  nodeId,OD_Data->NMTable[nodeId]); \
  return 0XFF; \
}}
/* Private variables ---------------------------------------------------------*/
Map_Val_UNS16 Controlword_Node[] = {
{&Controlword           ,0x6040},
{&NODE3_Controlword_6040,0x2001},};		/* Mapped at index 0x6040, subindex 0x00*/
Map_Val_UNS16 Statusword_Node[] =  {
{&Statusword            ,0x6041},
{&NODE3_Statusword_6041 ,0x2002},};		/* Mapped at index 0x6041, subindex 0x00 */
Map_Val_INTEGER8 Modes_of_operation_Node[] = {
{&Modes_of_operation    ,0x6060},
{&NODE3_Modes_of_operation_6060 ,0x2003},};		/* Mapped at index 0x6060, subindex 0x00 */
Map_Val_INTEGER32 Target_position_Node[] = {
{&Target_position,0x607A},
{&NODE3_Target_position_607A,0x2006},};		/* Mapped at index 0x607A, subindex 0x00 */
Map_Val_INTEGER32 Target_velocity_Node[] = {
{&Target_velocity,0x60FF},
{&NODE3_Target_velocity_60FF,0x2007},};		/* Mapped at index 0x60FF, subindex 0x00 */
Map_Val_INTEGER32 Position_actual_value_Node[] = {
{&Position_actual_value,0x6064},
{&NODE3_Position_actual_value_6064,0x2004},};		/* Mapped at index 0x6064, subindex 0x00*/
Map_Val_INTEGER32 Velocity_actual_value_Node[] = {
{&Velocity_actual_value,0x606C},
{&NODE3_Velocity_actual_value_0x606C,0x2005},};		/* Mapped at index 0x606C, subindex 0x00 */
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  �����̲߳�ѯֵ�ض�λ�Ƿ���һ
  * @param  value:��Ҫ��ѯ�ı���
  * @param  bit:��ѯ��λ
  * @param  timeout:��ʱ�˳�ʱ�䣬��λms
  * @param  block_time:����ʱ�䣬��λms
  * @retval None
  * @note   ������ѯ
  *         ���ж��ض�λ�Ƿ���һ�������ж�ֵ�Ƿ���ȡ�
  *         ��ֹ��������£�����λ��һ�޷��ж�Ϊ�ı�ɹ�
*/
static UNS8 block_query_BIT_change(UNS16 *value,UNS8 bit,uint16_t timeout,uint16_t block_time)
{
  uint16_t cnt = 0;

  while(!CANOPEN_GET_BIT(*value,bit))
  {
    if((cnt * block_time) < timeout)
    {
      cnt++;
      rt_thread_mdelay(block_time);
    }
    else
    {
      return 0xFF;
    }
  }
  return 0x00; 
}  
/******************************�˶�ģʽѡ����******************************************************************/
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��λ�ù滮ģʽ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
*/
UNS8 motor_on_profile_position(UNS8 nodeId)
{
  NODE_DECISION;
  *Target_position_Node[nodeId - 2].map_val = 0;
  SYNC_DELAY;
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,PROFILE_POSITION_MODE));
  FAILED_EXIT(Write_SLAVE_profile_position_speed_set(nodeId,0));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

  return 0x00;
}
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��岹λ��ģʽ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
*/
UNS8 motor_on_interpolated_position(UNS8 nodeId)
{
  NODE_DECISION;
  pos_cmd1 = *Position_actual_value_Node[nodeId - 2].map_val;
  FAILED_EXIT(Write_SLAVE_Interpolation_time_period(nodeId));
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,INTERPOLATED_POSITION_MODE));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
  /*State Transition 1: NO IP-MODE SELECTED => IP-MODE INACTIVE
  Event: Enter in the state OPERATION ENABLE with controlword and select ip 
  mode with modes of operation*/
//  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));
  return 0X00;
}
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��ԭ�㸴λģʽ
  * @param  offest:ԭ��ƫ��ֵ ��λ:PUU [ע��:ֻ�ǰ�ԭ��ƫ��ֵ�㵱Ϊ0����㣬�������˶���0����㴦]
  * @param  method:��ԭ��ʽ   ��Χ:0 ~ 35
  * @param  switch_speed:Ѱ��ԭ�㿪���ٶ� �趨��Χ 0.1 ~ 200 Ĭ��ֵ 10  ��λrpm ����:С�����һλ
  * @param  zero_speed:Ѱ�� Z�����ٶ�     �趨��Χ 0.1 ~ 50  Ĭ��ֵ 2   ��λrpm ����:С�����һλ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
*/
UNS8 motor_on_homing_mode(int32_t offset,uint8_t method,float switch_speed,float zero_speed,UNS8 nodeId)
{
  NODE_DECISION;
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,HOMING_MODE));
  FAILED_EXIT(Write_SLAVE_Homing_set(nodeId,offset,method,switch_speed,zero_speed));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));
  SYNC_DELAY;//��ʱ����������Ӧʱ�䣬����̫�췢�ʹ��������������δ��Ӧ
  return 0x00;
}
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ���ٶȹ滮ģʽ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
            2. �趨����ʱ��б�� OD 6083h�� 
            3. �趨����ʱ��б�� OD 6084h��
            606Fh�����ٶ�׼λ (Velocity threshold) 
            �˶����趨���ٶ��źŵ������Χ�����������ת�ٶ� (����ֵ) ���ڴ��趨ֵʱ��DO: 0x03(ZSPD) ����� 1��
Name          Value                   Description
rfg enable      0     Velocity reference value is controlled in any other (manufacturer specific) way, e.g. by a test function generator or manufacturer specific halt function.
                1     Velocity reference value accords to ramp output value. 
rfg unlock      0     Ramp output value is locked to current output value.
                1     Ramp output value follows ramp input value.
rfg use ref     0     Ramp input value is set to zero.
                1     Ramp input value accords to ramp reference.*/
UNS8 motor_on_profile_velocity(UNS8 nodeId)
{
  NODE_DECISION;
  *Target_velocity_Node[nodeId - 2].map_val = 0;
  SYNC_DELAY;
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,PROFILE_VELOCITY_MODE));
  FAILED_EXIT(Write_SLAVE_profile_position_speed_set(nodeId,0));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));
  
  return 0x00;
}
/******************************�˶�ģʽ��������******************************************************************/
/**
  * @brief  ���Ƶ����λ�ù滮ģʽ�˶�
  * @param  position:   λ��    ��λPUU �������
  * @param  speed:      �ٶ�    ��λRPM
  * @param  abs_rel:    �˶�ģʽ�� ��Ϊ0�������˶�ģʽ����Ϊ1������˶�ģʽ
  * @param  immediately:����������Чָ� ��Ϊ 0���ر�����������Чָ�� 1��������Ч����δ����Ŀ��λ��Ҳ��ִ���´��˶�
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ģʽ���󷵻�0XFF.��ʱ����0XFE
  * @note   �����ظ��˺����������Ƶ���˶���ͬλ�á� ��һod 0x2124����S�ͼӼ���
  ����ٶ�����      607Fh Ĭ��ֵ 3000rpm
  ���������      607Dh Ĭ��ֵ 2147483647
  ���������      607Dh Ĭ��ֵ -2147483647
  ���ٶ�ʱ��б��    6083h Ĭ��ֵ 200ms [0    rpm���ٵ� 3000 rpm����Ҫ��ʱ��]
  ���ٶ�ʱ��б��    6084h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
  ��ͣ����ʱ��б��  6085h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
  ��߼��ٶ�        60C5h Ĭ��ֵ 1  ms [0    rpm���ٵ� 3000 rpm����Ҫ��ʱ��]
  ��߼��ٶ�        60C6h Ĭ��ֵ 1  ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]

  ������OD 607Ah �����λ�� (OD 6064h) ֮������ֵС�ڴ˶���ʱ��
  ��ʱ��ά�ִ��� OD 6068h (λ�õ��ﷶΧʱ��)��״̬λStatusword 6041h�� Bit10Ŀ�굽�Ｔ�����
  λ�õ��ﷶΧ      6067H:Ĭ��ֵ 100PUU
  λ�õ��ﷶΧʱ��  6068h:Ĭ��ֵ 0  ms

  ��λ����� (60F4h) �������趨��Χʱ���ŷ������쾯 AL009λ�������� 
  λ����������  6065h:Ĭ��ֵ50331648PUU //50331648 / 16777216 = 3
  */
UNS8 motor_profile_position(int32_t position,int16_t speed,bool abs_rel,bool immediately,UNS8 nodeId)
{
  NODE_DECISION;
  UNS16 value = 0;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != PROFILE_POSITION_MODE)
  {
    LOG_D("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
    return 0XFF;
  }

  *Target_position_Node[nodeId - 2].map_val = position;
  SYNC_DELAY;
  FAILED_EXIT(Write_SLAVE_profile_position_speed_set(nodeId,speed));
  //�������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

  if(immediately == false)//����Ϊ�����ģʽ����������Ч
  {
    value = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }
  else//����Ϊ�����ģʽ��������Ч
  {
    value = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }

  if(abs_rel == false)//����Ϊ�����˶�
  {
    value &= ~ABS_REL;
  }
  else//����Ϊ����˶�
  {
    value |=  ABS_REL;
  }

  FAILED_EXIT(Write_SLAVE_control_word(nodeId,value));

  *Statusword_Node[nodeId - 2].map_val = 0;//�����������
  if(block_query_BIT_change(Statusword_Node[nodeId - 2].map_val,TARGET_REACHED,MAX_WAIT_TIME,1) != 0x00)
  {
    LOG_W("Motor runing time out");
    return 0XFE;
  }
  else
  {
    LOG_I("Completion of motor movement");
    return 0X00;
  }
}
/**
  * @brief  ���Ƶ���Բ岹λ��ģʽ�˶�
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF.
  * @note   None
*/
UNS8 motor_interpolation_position (UNS8 nodeId)
{
  NODE_DECISION;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != INTERPOLATED_POSITION_MODE)
  {
    LOG_D("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
    return 0XFF;
  }
  /* State Transition 3: IP-MODE INACTIVE => IP-MODE ACTIVE
  Event: Set bit enable ip mode (bit4) of the controlword while in ip mode and 
  OPERATION ENABLE*/
//  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | ENABLE_IP_MODE));//Bit 4���� on�� 
//  PDOEnable(OD_Data,1);
  pos_cmd1 += 100;
  return 0X00;
}
/**
  * @brief  ���Ƶ������ԭ�㸴��ģʽ
  * @param  zero_flag��0�����践��0��λ�á� zero_flag��1������0��λ��
  * @param  speed: �����ٶ�    ��λRPM
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,
  * ģʽ���󷵻�0XFF.
  * ��ʱ����0XFE.
  * ���û���δ����ƫ��ֵ����0XFD
  * @note   None
*/
UNS8 motor_homing_mode (bool zero_flag,int16_t speed,UNS8 nodeId)
{
  NODE_DECISION;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != HOMING_MODE)
  {
    LOG_D("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
    return 0xFF;
  }
  //�������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT));//�������������Ե������Bit 4��Ϊ������ on�� 
  LOG_I("Motor runing homing");

  *Statusword_Node[nodeId - 2].map_val = 0;//�����������
  if(block_query_BIT_change(Statusword_Node[nodeId - 2].map_val,HOMING_ATTAINED,MAX_WAIT_TIME,1) != 0x00)
  {
    LOG_W("Motor runing homing time out");
    return 0XFE;
  }
  else
  {
    LOG_I("Motor return to home is complete");
  }

  if(zero_flag == true && Home_offset != 0)
  {
    LOG_I("The motor is returning to zero");
    motor_on_profile_position(nodeId);
    FAILED_EXIT(motor_profile_position(0,speed,0,0,nodeId));

    *Statusword_Node[nodeId - 2].map_val = 0;//�����������
    if(block_query_BIT_change(Statusword_Node[nodeId - 2].map_val,TARGET_REACHED,MAX_WAIT_TIME,1) != 0x00)
    {
      LOG_W("Motor runing zero time out");
      return 0XFE;
    }
    else
    {
      LOG_I("Motor return to zero is complete");
    }
  }
  else if (zero_flag == true && Home_offset == 0)
  {
      LOG_W("The offset value is not set");
      return 0XFD;
  }

  return 0;
}
/**
  * @brief  ���Ƶ�����ٶȹ滮ģʽ�˶�
  * @param  speed:      �ٶ�    ��λRPM
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   �����ظ��˺����������Ƶ���˶���ͬλ�á� ��һod 0x2124����S�ͼӼ���
  */
UNS8 motor_profile_velocity(int16_t speed,UNS8 nodeId)
{
  NODE_DECISION;
  UNS16 value = 0;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != PROFILE_VELOCITY_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
    return 0XFF;
  }

  *Target_velocity_Node[nodeId - 2].map_val  = speed * 10;//Target_velocity ��λΪ0.1 rpm,��Ҫ*10

  return 0X00;
}
/******************************�˶��رռ���ѯ����******************************************************************/
/**
  * @brief  ���Ƶ���ر�.
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF.
  * @note   ����������ͣ
  ��ͣ����ʱ��б��  6085h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
*/
UNS8 motor_off(UNS8 nodeId)
{
  NODE_DECISION;
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_DISABLE_VOLTAGE));
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,0));//���ģʽѡ��
  return 0x00;
}
/**
  * @brief  ��ȡ��ǰ�ڵ�������ָ��Controlword
  * @param  nodeId:�ڵ�ID
  * @retval ��ȡ�ɹ�����Controlword��ʧ�ܷ���0��
  * @note   None.
*/
UNS16 motor_get_controlword(UNS8 nodeId)
{
  if(nodeId == MASTER_NODEID || nodeId > MAX_NODE_COUNT || nodeId == 0)
  {
    return 0;
  }
  return *Controlword_Node[nodeId - 2].map_val;
}
/**
  * @brief  ��ȡ��ǰ�ڵ���״̬λstatusword
  * @param  nodeId:�ڵ�ID
  * @retval ��ȡ�ɹ�����statusword��ʧ�ܷ���0��
  * @note   None.
*/
UNS16 motor_get_statusword(UNS8 nodeId)
{
  if(nodeId == MASTER_NODEID || nodeId > MAX_NODE_COUNT || nodeId == 0)
  {
    return 0;
  }
  return *Statusword_Node[nodeId - 2].map_val;
}
/**
  * @brief  ��ȡ��ǰ�ڵ�������λ��
  * @param  des:Ŀ���ַ
  * @param  nodeId:�ڵ�ID
  * @retval Ŀ���ַ
  * @note   ������ڵ�ID����ȷ�������ؿ�ָ��
*/
INTEGER32 *motor_get_position(INTEGER32* des,UNS8 nodeId)
{ 
  if(des == NULL)
    return RT_NULL;

  if(nodeId == MASTER_NODEID || nodeId > MAX_NODE_COUNT || nodeId == 0)
  {
    return RT_NULL;
  }
  else
  {
    *des = *Position_actual_value_Node[nodeId - 2].map_val;
    return des;
  }
}
/**
  * @brief  ��ȡ��ǰ�ڵ��������ٶ�
  * @param  des:Ŀ���ַ
  * @param  nodeId:�ڵ�ID
  * @retval Ŀ���ַ
  * @note   ������ڵ�ID����ȷ�������ؿ�ָ��
*/
INTEGER32 *motor_get_velocity(INTEGER32* des,UNS8 nodeId)
{
  if(des == NULL)
    return RT_NULL;

  if(nodeId == MASTER_NODEID || nodeId > MAX_NODE_COUNT || nodeId == 0)
  {
    return RT_NULL;
  }
  else
  {
    *des = *Velocity_actual_value_Node[nodeId - 2].map_val;
    return des;
  }
}
/**
  * @brief  ��ѯ���״̬.
  * @param  None.
  * @retval None.
  * @note   
  ������
  ״̬��
  �����������
  ��ǰλ��
  ��ǰ�ٶ�

  Alarm_code��AL180 �����쳣
              AL3E3 ͨѶͬ���źų�ʱ[IPģʽδ�յ�����]
              AL022 ����·��Դ�쳣[����������]
              AL009 λ�ÿ��������� [��Ҫ�ҷ�λ������]
*/
static void motor_state(UNS8 nodeId)
{
  LOG_I("Mode operation:%d",*Modes_of_operation_Node[nodeId - 2].map_val);
	LOG_I("ControlWord 0x%4.4X", *Controlword_Node[nodeId - 2].map_val);
  LOG_I("StatusWord 0x%4.4X", *Statusword_Node[nodeId - 2].map_val);
  
  if(CANOPEN_GET_BIT(*Statusword_Node[nodeId - 2].map_val , FAULT))
    LOG_E("motor fault!");
  if(CANOPEN_GET_BIT(*Statusword_Node[nodeId - 2].map_val , WARNING))
    LOG_W("motor warning!");
  if(CANOPEN_GET_BIT(*Statusword_Node[nodeId - 2].map_val , FOLLOWING_ERROR))
  {
    if(*Modes_of_operation_Node[nodeId - 2].map_val == PROFILE_POSITION_MODE)
      LOG_E("motor following error!");
  }
  if(CANOPEN_GET_BIT(*Statusword_Node[nodeId - 2].map_val , POSITIVE_LIMIT))
    LOG_W("motor touch  positive limit!");
  if(!CANOPEN_GET_BIT(*Statusword_Node[nodeId - 2].map_val , TARGET_REACHED))
    LOG_W("The node did not receive the target arrival command!");
  
  if(CANOPEN_GET_BIT(*Statusword_Node[nodeId - 2].map_val , 13))
  {
    if(*Modes_of_operation_Node[nodeId - 2].map_val == PROFILE_POSITION_MODE)
    {
      LOG_E("motor following error!");
    }
    else if(*Modes_of_operation_Node[nodeId - 2].map_val == HOMING_MODE)
    {
      LOG_E("motor Homing error occurred!");
    }
  }

	LOG_I("current position %d PUU", *Position_actual_value_Node[nodeId - 2].map_val);  //ע��Ϊ0X6064��0X6063Ϊ����ʽλ��
	LOG_I("current speed %.1f RPM",  *Velocity_actual_value_Node[nodeId - 2].map_val / 10.0f);//ע�ⵥλΪ0.1rpm
}
#ifdef RT_USING_MSH
#define DEFAULT_NODE SERVO_NODEID_1//MSH����Ĭ�ϲ����ڵ�
/**
  * @brief  MSH���Ƶ���˶�
  * @param  None
  * @retval None
  * @note   None
*/
static void cmd_motor(uint8_t argc, char **argv) 
{
#define MOTOR_CMD_ON                    0
#define MOTOR_CMD_OFF                   1
#define MOTOR_CMD_STATE                 2
#define MOTOR_CMD_PPMOVE                3
#define MOTOR_CMD_IPMOVE                4
#define MOTOR_CMD_HMMOVE                5
#define MOTOR_CMD_PVMOVE                6
  size_t i = 0;

  const char* help_info[] =
    {
            [MOTOR_CMD_ON]             = "motor on pp/ip/hm/pv                                      - Enable motor control",
            [MOTOR_CMD_OFF]            = "motor off                                                 - Disenable motor control",
            [MOTOR_CMD_STATE]          = "motor state                                               - Display motor status.",
            [MOTOR_CMD_PPMOVE]         = "motor pp_move                                             - Setting motor pp motion position",
            [MOTOR_CMD_IPMOVE]         = "motor ip_move                                             - Setting motor ip motion position.",
            [MOTOR_CMD_HMMOVE]         = "motor hm_move                                             - Setting motor into homing mode.",
            [MOTOR_CMD_PVMOVE]         = "motor pv_move                                             - Setting motor pv motion position.",
    };

    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
        {
            rt_kprintf("%s\n", help_info[i]);
        }
        rt_kprintf("\n");
    }
    else
    {
        const char *operator = argv[1];

        if (!strcmp(operator, "on"))
        {
          if(argc <= 2) 
          {
              rt_kprintf("Usage: motor [mode] <nodeid> -pp ip hm pv pt\n");
              return;
          }
          if (!strcmp(argv[2], "pp"))
          {
            UNS8 nodeId = DEFAULT_NODE;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_on_profile_position(nodeId);
          }
          else if (!strcmp(argv[2], "ip"))
          {
            UNS8 nodeId = DEFAULT_NODE;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_on_interpolated_position(nodeId);
          }
          else if (!strcmp(argv[2], "hm"))
          {
            int32_t offset = 0;
            uint8_t method = 34;//34 ��ʼ������Ѱ��Z���� 33 ��ʼ������Ѱ��Z���� 
            float switch_speed = 100;//��λRPM
            float zero_speed = 20;//��λRPM
            UNS8 nodeId = DEFAULT_NODE;

            if(argc <= 3) 
            {
                rt_kprintf("Usage: motor on hm [offset] <nodeId> <switch_speed> <zero_speed> <method> \n");
                return;
            }
            offset = atoi(argv[3]);
            
            if(argc > 4) 
            {
              nodeId = atoi(argv[4]);
            }   
            if(argc > 5) 
            {
              switch_speed = atoi(argv[5]);
            }
            if(argc > 6) 
            {zero_speed = atoi(argv[6]);
            }
            if(argc > 7) 
            {
              method = atoi(argv[7]);
            }
            rt_kprintf("homing mode set offset: %d, nodeId: %d, method: %d\n", offset, nodeId, method);
            rt_kprintf("switch_speed: %.1f, zero_speed: %.1f\n", switch_speed, zero_speed);
            motor_on_homing_mode(offset,method,switch_speed,zero_speed,nodeId);
          }
          else if (!strcmp(argv[2], "pv"))
          {
            UNS8 nodeId = DEFAULT_NODE;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_on_profile_velocity(nodeId);
          }
          else
          {
              rt_kprintf("Usage: motor [mode] <nodeId> -pp ip hm pv pt\n");
              return;
          }
        }
        else if (!strcmp(operator, "off"))
        {
            UNS8 nodeId = DEFAULT_NODE;
            if(argc > 2) 
            {
              nodeId = atoi(argv[2]);
            }   
            motor_off(nodeId);
        }
        else if (!strcmp(operator, "state"))
        {
            UNS8 nodeId = DEFAULT_NODE;
            if(argc > 2) 
            {
              nodeId = atoi(argv[2]);
            }
            motor_state(nodeId);
        }
        else if (!strcmp(operator, "pp_move"))
        {
            int32_t position;
            int32_t speed = 60;//RPM:60
            bool immediately = false;
            bool abs_rel     = false;
            UNS8 nodeId = DEFAULT_NODE;
            if(argc <= 2) 
            {
                rt_kprintf("Usage: motor pp_move [position] <nodeId> <speed> <0:abs/1:rel> <immediately> \n");
                return;
            }

            position = atoi(argv[2]);
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }  
            if(argc > 4) 
            {
                speed = atoi(argv[4]);
            }
            if(argc > 5) 
            {
                abs_rel = atoi(argv[5]);
            }
            if(argc > 6) 
            {
                immediately = atoi(argv[6]);
            }

            rt_kprintf("nodeId %d,move to position: %d, speed: %d\n",nodeId, position, speed);
            rt_kprintf("abs_rel: %d, immediately: %d\n", abs_rel, immediately);
            motor_profile_position(position,speed,abs_rel,immediately,nodeId);
        }
        else if (!strcmp(operator, "ip_move"))
        {
          UNS8 nodeId = DEFAULT_NODE;
          if(argc > 2) 
          {
            nodeId = atoi(argv[2]);
          }  
          motor_interpolation_position(nodeId);
        }
        else if (!strcmp(operator, "hm_move"))
        {
            bool zero_flag = false;
            UNS16 nodeId = DEFAULT_NODE;
            int16_t speed = 60;

            if(argc <= 2) 
            {
                rt_kprintf("Usage: motor hm_move [zero] <nodeId> <speed> --homing move zero:1,run zero postion and speed\n");
                return;
            }
            zero_flag = atoi(argv[2]);
            if(argc > 3) 
            {
                nodeId = atoi(argv[3]);
            }
            if(argc > 4) 
            {
                speed = atoi(argv[4]);
            }
            motor_homing_mode(zero_flag,speed,nodeId);
        }
        else if (!strcmp(operator, "pv_move"))
        {
          int32_t speed = 0;//RPM:0
          UNS8 nodeId = DEFAULT_NODE;
          if(argc <= 2) 
          {
            rt_kprintf("Usage: motor pv_move [speed] <nodeId>\n");
            return;
          }
          speed = atoi(argv[2]);
          if(argc > 3) 
          {
              nodeId = atoi(argv[3]);
          }
          motor_profile_velocity(speed,nodeId);
        }
        else
        {
          rt_kprintf("Usage:\n");
          for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
          {
              rt_kprintf("%s\n", help_info[i]);
          }
          rt_kprintf("\n");
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_motor,motor,motor command.);
#endif