#include <stdint.h>
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
1  Profile Position Mode (位置规划模式) 
  1. 设定模式，OD 6060h = 01h，为位置控制模式。 
  2. 设定目标位置，OD 607Ah (单位：PUU)。 
  PUU(Pulse of User Unit),此单位为经过电子齿轮比例缩放的量
  3. 设定速度命令，OD 6081h (单位：PUU/sec)。 //有默认值
  4. 设定加速度时间斜率，OD 6083h (单位：ms)。//有默认值
  5. 设定减速度时间斜率，OD 6084h (单位：ms)。//有默认值
  6. 设定控制指令，OD 6040h，请依照以下步骤操作。步骤 6.1与 6.2是为了使驱动器 
  的状态机 (state machine) 进入准备状态。状态机说明请详见章节 12.4的 OD 6040h
  说明。 
  步骤  说明 
  6.1  Shutdown (关闭) 
  6.2  Switch on (伺服 Servo On准备) 
  6.3  Enable Operation (伺服 Servo On) 
  6.4  命令触发 (正缘触发) 1 1 1 1 1【bit0~bit4】
  bit位  名称          值   说明
  4      新设定点      0     没有承担目标位置
                       1     假设目标位置
  5      立即更改设定  0     完成实际定位，然后开始下一个定位
                       1     中断实际定位，开始下一个定位
  6      abs / rel     0     目标位置是一个绝对值
                       1     目标位置是一个相对值
  7      停止          0     执行定位
                       1     带轮廓减速的停止轴(如果没有轮廓加速支持)
  7. 在完成第一段运动命令后，若需要执行下一段运动命令需再设定目标位置、速度等 
  条件。 
  8. 设定控制指令，OD 6040h。由于命令触发是正缘触发，因此必须先将 Bit 4切为 off
  再切至 on。 
  步骤  说明 
  8.1   Enable Operation (伺服 Servo On) 
  8.2   命令触发 (正缘触发) 
  读取驱动器信息： 
  1. 读取 OD 6064h取得目前电机回授位置。 
  2. 读取 OD 6041h取得驱动器的状态，包括 following error (追随误差)、set-point 
  acknowledge (收到命令通知) 与 target reached (到达目标通知)
  * @author
  台达A3电机设置为CANOPEN模式
  1.恢复出厂设置
  用户可依下列步骤连接 CANopen上位机与 ASDA-A3伺服驱动器： 
  1. 设定 CANopen模式：将参数 P1.001设为 0x0C。 
  2. 设定节点 ID，将 P3.000范围设为 01h ~ 7Fh。 
  3. 将参数 P3.001设为 0403h，P3.001.Z设定鲍率 1 Mbps (0：125 Kbps； 
  1：250 Kbps；2：500 Kbps；3：750 Kbps；4：1 Mbps)。 
  4. 建议将 P3.012设定为0x0100，以实现将下表参数断电保持的功能。 
  在驱动器重新上下电或是进行通讯重置后，下表的 P参数会维持本来的设定，
  不会加载 CANopen / DMCNET / EtherCAT参数的数值。
  5. 建议开启动态抱闸功能，P1.032 = 0x0000。
  6. 重新设置电子齿轮比 16777216 / 100000
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#include "canfestival.h"
#include "timers_driver.h"
#include "master402_od.h"
#include "master402_canopen.h"

/* Private typedef -----------------------------------------------------------*/
/*0x6040控制指令 Controlword 状态位定义*/
typedef enum
{
  WRITE_SWITCH_ON             = 1 << 0,//按下开关
  EN_VOLTAGE                  = 1 << 1,//使能电源
  QUICK_STOP                  = 1 << 2,//急停
  EN_OPERATION                = 1 << 3,//操作使能
  FAULT_RESET                 = 1 << 7,//错误重置
  HALT                        = 1 << 8,//暂停
}CONTROL_WORD;
/*0x6041状态位 Statusword定义*/
typedef enum
{
  READY_TO_SWITCH_ON          = 0,//准备功能启动
  READ_SWITCH_ON              = 1,//伺服准备完成
  OPERATION_ENABLED           = 2,//伺服使能
  FAULT                       = 3,//异常信号
  VOLTAGE_ENABLED             = 4,//伺服输入侧已供电
  READ_QUICK_STOP             = 5,//紧急停止 [0:开启急停 1:关闭急停]
  SWITCH_ON_DISABLED          = 6,//伺服准备功能关闭
  WARNING                     = 7,//警告信号
  REMOTE                      = 9,//远程控制
  TARGET_REACHED              = 10,//目标到达
  POSITIVE_LIMIT              = 14,//正向运转禁止极限
  NEGATIVE_LIMIT              = 15,//负向运转禁止极限
}STATUS_WORD;
/*0X6060 模式设定Modes of operation定义*/
typedef enum
{
  PROFILE_POSITION_MODE       = 1,//位置规划模式
  PROFILE_VELOCITY_MODE       = 3,//速度规划模式
  PROFILE_TORQUE_MODE         = 4,//扭矩规划模式
  HOMING_MODE                 = 6,//原点复归模式
  INTERPOLATED_POSITION_MODE  = 7,//插补位置模式
}MODE_OPERATION;
/*位置规划模式下Controlword操作模式特定位*/
typedef enum
{
  NEW_SET_POINT               = 1 << 4,//命令触发(正缘触发)
  CHANGE_SET_IMMEDIATELY      = 1 << 5,//命令立即生效指令  设为 0，关闭命令立即生效指令
  ABS_REL                     = 1 << 6,//更改为绝对定位或相对定位  0，目标位置是一个绝对值 1，目标位置是一个相对值
}PROFILE_POSITION_CONTROLWORD;
/*位置规划模式下Statusword定义*/
typedef enum
{
  SET_POINT_ACKNOWLEDGE       = 12,//伺服收到命令信号
  FOLLOWING_ERROR             = 13,//追随错误
}PROFILE_POSITION_STATUSWORD;
/*插补位置模式下Controlword操作模式特定位
  Name          Value   Description
Enable ip mode    0     Interpolated position mode inactive 
                  1     Interpolated position mode active*/
typedef enum
{
   ENABLE_IP_MODE             = 1 << 4,//使能IP模式
}Interpolated_Position_CONTROLWORD;
/*插补位置模式下Statusword定义
    Name        Value   Description
ip mode active    1     Interpolated position mode active
*/
typedef enum
{
  IP_MODE_ACTIVE              = 12,//插补位置模式是否生效
}Interpolated_Position_STATUSWORD;
/*原点复归模式下Controlword操作模式特定位
  Name          Value   Description
Homing          0       Homing mode inactive
operation       0 → 1  Start homing mode
star            1       Homing mode active
                1 → 0  Interrupt homing mode*/
typedef enum
{
  HOMING_OPERATION_STAR       = 1 << 4,//使能IP模式
}HOMING_CONTROLWORD;
/*原点复归模式下Statusword定义
    Name          Value   Description
Homing attained    0      Homing mode not yet completed.
                   1      Homing mode carried out successfully

Homing error       0      No homing error
                   1      Homing error occurred;Homing mode carried out not successfully;The error cause is found by reading the error code
*/
typedef enum
{
  HOMING_ATTAINED              = 12,//回到原点
  HOMING_ERROR                 = 13,//回原错误
}HOMING_STATUSWORD;
/* Private define ------------------------------------------------------------*/
/*使用命令延迟去更改操作，不能保证命令已经更改
更改操作模式，可以使用0X6061查看模式读取保证更改成功
阻塞线程，判断超时。恢复运行
*/
#define SYNC_DELAY            rt_thread_mdelay(20)//命令延时
#define MAX_WAIT_TIME         50                                    //ms

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
{&Velocity_actual_value,0x6064},
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
static UNS8 motor_on_profile_position(UNS8 nodeId)
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
  * @brief  控制电机使能并选择插补位置模式
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
  * @note   None
*/
static UNS8 motor_on_interpolated_position(UNS8 nodeId)
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
  * @brief  控制电机使能并选择原点复位模式
  * @param  offest:原点偏移值 单位:PUU [注意:只是把原点偏移值点当为0坐标点，并不会运动到0坐标点处]
  * @param  method:回原方式   范围:0 ~ 35
  * @param  switch_speed:寻找原点开关速度 设定范围 0.1 ~ 200 默认值 10  单位rpm 精度:小数点后一位
  * @param  zero_speed:寻找 Z脉冲速度     设定范围 0.1 ~ 50  默认值 2   单位rpm 精度:小数点后一位
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
  * @note   None
*/
static UNS8 motor_on_homing_mode(int32_t offset,uint8_t method,float switch_speed,float zero_speed,UNS8 nodeId)
{
  NODE_DECISION;
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,HOMING_MODE));
  FAILED_EXIT(Write_SLAVE_Homing_set(nodeId,offset,method,switch_speed,zero_speed));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

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
static UNS8 motor_on_profile_velocity(UNS8 nodeId)
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
/******************************运动模式操作函数******************************************************************/
/**
  * @brief  控制电机以位置规划模式运动
  * @param  position:   位置    单位PUU 脉冲个数
  * @param  speed:      速度    单位RPM
  * @param  abs_rel:    运动模式。 设为0，绝对运动模式；设为1，相对运动模式
  * @param  immediately:命令立即生效指令。 设为 0，关闭命令立即生效指令 1，立刻生效，即未到达目标位置也可执行下次运动
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
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
static UNS8 motor_profile_position(int32_t position,uint32_t speed,bool abs_rel,bool immediately,UNS8 nodeId)
{
  NODE_DECISION;
  UNS16 value = 0;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != PROFILE_POSITION_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
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

  return 0X00;
}
/**
  * @brief  控制电机以插补位置模式运动
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF.
  * @note   None
*/
static UNS8 motor_interpolation_position (UNS8 nodeId)
{
  NODE_DECISION;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != INTERPOLATED_POSITION_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
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
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF.
  * @note   None
*/
static UNS8 motor_homing_mode (bool zero_flag,UNS8 nodeId)
{
  NODE_DECISION;
  if(*Modes_of_operation_Node[nodeId - 2].map_val != HOMING_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",*Modes_of_operation_Node[nodeId - 2].map_val);
    return 0xFF;
  }
  //由于命令触发是正缘触发，因此必须先将 Bit 4切为 off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT));//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  LOG_I("Motor runing homing");
  if(block_query_BIT_change(Statusword_Node[nodeId - 2].map_val,HOMING_ATTAINED,5000,1) != 0x00)
  {
    LOG_W("Motor runing homing time out");
    return 0XFF;
  }
  else
  {
    LOG_I("Motor return to home is complete");
  }
  //回到零点
  if(zero_flag == true && Home_offset != 0)
  {
    LOG_I("The motor is returning to zero");
    motor_on_profile_position(nodeId);
    FAILED_EXIT(motor_profile_position(0,60,0,0,nodeId));

    if(block_query_BIT_change(Statusword_Node[nodeId - 2].map_val,TARGET_REACHED,5000,1) != 0x00)
    {
      LOG_W("Motor runing zero time out");
    }
    else
    {
      LOG_I("Motor return to zero is complete");
    }
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
static UNS8 motor_profile_velocity(uint32_t speed,UNS8 nodeId)
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
static UNS8 motor_off(UNS8 nodeId)
{
  NODE_DECISION;
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN | FAULT_RESET));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_DISABLE_VOLTAGE));
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,0));//清除模式选择
  return 0x00;
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
	LOG_I("ControlWord 0x%0X", *Controlword_Node[nodeId - 2].map_val);
  LOG_I("StatusWord 0x%0X", *Statusword_Node[nodeId - 2].map_val);
  
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

            rt_kprintf("move to position: %d, speed: %d\n", position, speed);
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
            if(argc <= 2) 
            {
                rt_kprintf("Usage: motor hm_move [zero] <nodeId> --homing move zero:1,run zero postion\n");
                return;
            }
            zero_flag = atoi(argv[2]);
            if(argc > 3) 
            {
                nodeId = atoi(argv[3]);
            }
            motor_homing_mode(zero_flag,nodeId);
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
static void cmd_CONTROL_WORD(uint8_t argc, char **argv)
{
#define CW_CMD_ON                  0
#define CW_CMD_EN                  1
#define CW_CMD_DIS                 2
#define CW_CMD_RESET               3
    size_t i = 0;

    const char* help_info[] =
    {
            [CW_CMD_ON]             = "canoepn_cw [nodeID] on                       --SWITCH ON",
            [CW_CMD_EN]             = "canoepn_cw [nodeID] en                       --ENABLE OPERATION",
            [CW_CMD_DIS]            = "canoepn_cw [nodeID] dis                      --DISABLE VOLTAGE",
            [CW_CMD_RESET]          = "canoepn_cw [nodeID] reset                    --FAULT_RESET",
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
        UNS8 nodeId = DEFAULT_NODE;
        if(argc > 1) 
        {
            nodeId = atoi(argv[1]);
        }
        const char *operator = argv[2];
        if (!strcmp(operator, "shutdown"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN);
        }
        else if(!strcmp(operator, "on"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON);
        }
        else if(!strcmp(operator, "en"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION);
        }
        else if(!strcmp(operator, "dis"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_DISABLE_VOLTAGE);
        }
        else if(!strcmp(operator, "reset"))
        {
            Write_SLAVE_control_word(nodeId,FAULT_RESET);
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
MSH_CMD_EXPORT_ALIAS(cmd_CONTROL_WORD,canopen_cw,canopen control word.);
#endif