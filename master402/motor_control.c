/**
 * @file motor_control.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
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
#endif /*RT_USING_FINSH*/

#include "canfestival.h"
#include "timers_driver.h"
#include "master402_od.h"
#include "master402_canopen.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/*使用命令延迟去更改操作，不能保证命令已经更改
更改操作模式，可以使用0X6061查看模式读取保证更改成功
阻塞线程，判断超时。恢复运行
*/
#define SYNC_DELAY            rt_thread_mdelay(20)//命令延时
#define MAX_WAIT_TIME         5000                //ms

/*0X6040 控制指令 Controlword 状态宏*/
//伺服 Servo Off
#define CONTROL_WORD_SHUTDOWN         (EN_VOLTAGE | QUICK_STOP) & (~WRITE_SWITCH_ON & ~FAULT_RESET)
//伺服Servo On
#define CONTROL_WORD_SWITCH_ON        (WRITE_SWITCH_ON | EN_VOLTAGE | QUICK_STOP) & (~EN_OPERATION)  
//执行运动模式
#define CONTROL_WORD_ENABLE_OPERATION (WRITE_SWITCH_ON | EN_VOLTAGE | QUICK_STOP | EN_OPERATION & ~FAULT_RESET)
//关闭供电
#define CONTROL_WORD_DISABLE_VOLTAGE  (0 & ~EN_VOLTAGE & ~FAULT_RESET) 
//急停
#define CONTROL_WORD_QUICK_STOP       (EN_VOLTAGE & ~QUICK_STOP & ~FAULT_RESET)
/* Private macro -------------------------------------------------------------*/
#define	CANOPEN_GET_BIT(x, bit)	  ((x &   (1 << bit)) >> bit)	/* 获取第bit位 */
/*判读返回值不为0X00退出当前函数宏*/
#define FAILED_EXIT(CODE){  \
if(CODE != 0X00)            \
   return 0XFF;             \
}
/*判断当前操作节点是否处于运行状态宏*/
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
  * @brief  阻塞线程查询值特定位是否置一
  * @param  value:需要查询的变量
  * @param  bit:查询的位
  * @param  timeout:超时退出时间，单位ms
  * @param  block_time:阻塞时间，单位ms
  * @retval None
  * @note   阻塞查询
  *         仅判断特定位是否置一，并不判断值是否相等。
  *         防止其他情况下，其他位置一无法判断为改变成功
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
/******************************运动模式选择函数******************************************************************/
/**
  * @brief  控制电机使能并选择位置规划模式
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
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
  SYNC_DELAY;//延时给驱动器响应时间，以免太快发送触发命令导致驱动器未响应
  return 0x00;
}
/**
  * @brief  控制电机使能并选择插补位置模式
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
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
  SYNC_DELAY;//延时给驱动器响应时间，以免太快发送触发命令导致驱动器未响应
  return 0X00;
}
/**
  * @brief  控制电机使能并选择原点复位模式
  * @param  offest:原点偏移值 单位:PUU [注意:只是把原点偏移值点当为0坐标点，并不会运动到0坐标点处]
  * @param  method:回原方式   范围:0 ~ 35
  * @param  switch_speed:寻找原点开关速度 设定范围 0.1 ~ 200 默认值 100  单位rpm 精度:小数点后一位
  * @param  zero_speed:寻找 Z脉冲速度     设定范围 0.1 ~ 50  默认值 20   单位rpm 精度:小数点后一位
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
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
  SYNC_DELAY;//延时给驱动器响应时间，以免太快发送触发命令导致驱动器未响应
  return 0x00;
}
/**
  * @brief  控制电机使能并选择速度规划模式
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
  * @note   None
            2. 设定加速时间斜率 OD 6083h。 
            3. 设定减速时间斜率 OD 6084h。
            606Fh：零速度准位 (Velocity threshold) 
            此对象设定零速度信号的输出范围。当电机正反转速度 (绝对值) 低于此设定值时，DO: 0x03(ZSPD) 将输出 1。
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
  SYNC_DELAY;//延时给驱动器响应时间，以免太快发送触发命令导致驱动器未响应
  return 0x00;
}
/******************************运动模式操作函数******************************************************************/
/**
  * @brief  控制电机以位置规划模式运动
  * @param  position:   位置    单位PUU 脉冲个数
  * @param  speed:      速度    单位RPM
  * @param  abs_rel:    运动模式。 设为0，绝对运动模式；设为1，相对运动模式
  * @param  immediately:命令立即生效指令。 设为 0，关闭命令立即生效指令 1，立刻生效，即未到达目标位置也可执行下次运动
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE
  * @note   可以重复此函数用来控制电机运动不同位置。 置一od 0x2124开启S型加减速
  最大速度限制      607Fh 默认值 3000rpm
  软件正向极限      607Dh 默认值 2147483647
  软件反向极限      607Dh 默认值 -2147483647
  加速度时间斜率    6083h 默认值 200ms [0    rpm加速到 3000 rpm所需要的时间]
  减速度时间斜率    6084h 默认值 200ms [3000 rpm减速到 0    rpm所需要的时间]
  急停减速时间斜率  6085h 默认值 200ms [3000 rpm减速到 0    rpm所需要的时间]
  最高加速度        60C5h 默认值 1  ms [0    rpm加速到 3000 rpm所需要的时间]
  最高减速度        60C6h 默认值 1  ms [3000 rpm减速到 0    rpm所需要的时间]

  当命令OD 607Ah 与回授位置 (OD 6064h) 之间的误差值小于此对象时，
  且时间维持大于 OD 6068h (位置到达范围时间)，状态位Statusword 6041h的 Bit10目标到达即输出。
  位置到达范围      6067H:默认值 100PUU
  位置到达范围时间  6068h:默认值 0  ms

  当位置误差 (60F4h) 超过此设定范围时，伺服即跳异警 AL009位置误差过大。 
  位置误差警告条件  6065h:默认值50331648PUU //50331648 / 16777216 = 3
  */
UNS8 motor_profile_position(int32_t position,float speed,bool abs_rel,bool immediately,UNS8 nodeId)
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
  //由于命令触发是正缘触发，因此必须先将 Bit 4切为 off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

  if(immediately == false)//设置为命令触发模式，不立刻生效
  {
    value = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  }
  else//设置为命令触发模式，立刻生效
  {
    value = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  }

  if(abs_rel == false)//设置为绝对运动
  {
    value &= ~ABS_REL;
  }
  else//设置为相对运动
  {
    value |=  ABS_REL;
  }

  FAILED_EXIT(Write_SLAVE_control_word(nodeId,value));

  if(immediately == false)
  {
    *Statusword_Node[nodeId - 2].map_val = 0;//清除本地数据
    if(block_query_BIT_change(Statusword_Node[nodeId - 2].map_val,TARGET_REACHED,MAX_WAIT_TIME,1) != 0x00)
    {
      LOG_W("Motor runing time out");
      return 0XFE;
    }
    else
    {
      LOG_D("Completion of motor movement");
      return 0X00;
    }
  }
  return 0X00;
}
/**
  * @brief  控制电机以插补位置模式运动
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF.
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
//  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | ENABLE_IP_MODE));//Bit 4切至 on。 
//  PDOEnable(OD_Data,1);
  pos_cmd1 += 100;
  return 0X00;
}
/**
  * @brief  控制电机进入原点复归模式
  * @param  zero_flag：0，无需返回0点位置。 zero_flag：1，返回0点位置
  * @param  speed: 回零速度    单位RPM
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,
  * 模式错误返回0XFF.
  * 超时返回0XFE.
  * 设置回零未设置偏移值返回0XFD
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
  //由于命令触发是正缘触发，因此必须先将 Bit 4切为 off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT));//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  LOG_I("Motor runing homing");

  *Statusword_Node[nodeId - 2].map_val = 0;//清除本地数据
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

    *Statusword_Node[nodeId - 2].map_val = 0;//清除本地数据
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
  * @brief  控制电机以速度规划模式运动
  * @param  speed:      速度    单位RPM
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
  * @note   可以重复此函数用来控制电机运动不同位置。 置一od 0x2124开启S型加减速
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

  *Target_velocity_Node[nodeId - 2].map_val  = speed * 10;//Target_velocity 单位为0.1 rpm,需要*10

  return 0X00;
}
/******************************运动关闭及查询函数******************************************************************/
/**
  * @brief  控制电机关闭.
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF.
  * @note   可以用来急停
  急停减速时间斜率  6085h 默认值 200ms [3000 rpm减速到 0    rpm所需要的时间]
*/
UNS8 motor_off(UNS8 nodeId)
{
  NODE_DECISION;
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_DISABLE_VOLTAGE));
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,0));//清除模式选择
  return 0x00;
}
/**
  * @brief  获取当前节点电机控制指令Controlword
  * @param  nodeId:节点ID
  * @retval 获取成功返回Controlword，失败返回0。
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
  * @brief  获取当前节点电机状态位statusword
  * @param  nodeId:节点ID
  * @retval 获取成功返回statusword，失败返回0。
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
  * @brief  获取当前节点电机反馈位置
  * @param  des:目标地址
  * @param  nodeId:节点ID
  * @retval 目标地址
  * @note   若输入节点ID不正确，将返回空指针
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
  * @brief  获取当前节点电机反馈速度
  * @param  des:目标地址
  * @param  nodeId:节点ID
  * @retval 目标地址
  * @note   若输入节点ID不正确，将返回空指针
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
  * @brief  查询电机状态.
  * @param  None.
  * @retval None.
  * @note   
  控制字
  状态字
  电机报警代码
  当前位置
  当前速度

  Alarm_code：AL180 心跳异常
              AL3E3 通讯同步信号超时[IP模式未收到命令]
              AL022 主回路电源异常[驱动器掉电]
              AL009 位置控制误差过大 [不要乱发位置命令]
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

	LOG_I("current position %d PUU", *Position_actual_value_Node[nodeId - 2].map_val);  //注意为0X6064，0X6063为增量式位置
	LOG_I("current speed %.1f RPM",  *Velocity_actual_value_Node[nodeId - 2].map_val / 10.0f);//注意单位为0.1rpm
}
#ifdef RT_USING_MSH
#define DEFAULT_NODE SERVO_NODEID_1//MSH命令默认操作节点
/**
  * @brief  MSH控制电机运动
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
            uint8_t method = 34;//34 起始点往右寻找Z脉冲 33 起始点往左寻找Z脉冲 
            float switch_speed = 100;//单位RPM
            float zero_speed = 20;//单位RPM
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
#endif /*RT_USING_MSH*/