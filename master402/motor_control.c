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
  4. 建议将 P3.012.Z设定为 1，以实现将下表参数断电保持的功能。 
  在驱动器重新上下电或是进行通讯重置后，下表的 P参数会维持本来的设定，
  不会加载 CANopen / DMCNET / EtherCAT参数的数值。
  5. 建议开启动态抱闸功能，P1.032 = 0x0000。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

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
  FAULT_RESET                 = 1 << 7,//复位
  HALT                        = 1 << 8,//硬件故障
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
/* Private define ------------------------------------------------------------*/
/*使用命令延迟去更改操作，不能保证命令已经更改
更改操作模式，可以使用0X6061查看模式读取保证更改成功
阻塞线程，判断超时。恢复运行
*/
#define SYNC_DELAY            rt_thread_delay(RT_TICK_PER_SECOND/50)//命令延时
#define ENCODER_RES           (pow(2,24))                           //编码器分辨率 16,777,216
#define PULSES_PER_TURN       (100000)                              //脉冲数/转
//电子齿轮比 6093h sub1/6093h sub2
#define ELECTRONIC_GEAR_RATIO (ENCODER_RES / 100000)                //电子齿轮比
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
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  控制电机使能并选择模式
  * @param  mode:工作模式 ，具体查看结构体@MODE_OPERATION
  * @retval None
  * @note   没有节点选择
*/
static void motor_on(MODE_OPERATION mode)
{
//    int nodeId = 1;
//    if(argc > 1) {
//        nodeId = atoi(argv[1]);
//    }
//    (void)nodeId;
	Modes_of_operation = mode;
	Profile_velocity = 0;
	Target_position = 0;

	Controlword = CONTROL_WORD_SHUTDOWN;
	SYNC_DELAY;
	Controlword = CONTROL_WORD_SWITCH_ON;
	SYNC_DELAY;
	Controlword = CONTROL_WORD_ENABLE_OPERATION;
}
/**
  * @brief  控制电机关闭.
  * @param  None.
  * @retval None.
  * @note   可以用来急停
  急停减速时间斜率  6085h 默认值 200ms [3000 rpm减速到 0    rpm所需要的时间]
*/
static void motor_off(void)
{
//    int nodeId = 1;
//    if(argc > 1) {
//        nodeId = atoi(argv[1]);
//    }
//    (void)nodeId;
	Controlword = CONTROL_WORD_SHUTDOWN;
	SYNC_DELAY;
	Controlword = CONTROL_WORD_DISABLE_VOLTAGE;
}
/**
  * @brief  控制电机以位置规划模式运动
  * @param  position:   位置    单位PUU 脉冲个数
  * @param  speed:      速度    单位PUU/sec
  * @param  immediately:命令立即生效指令。 设为 0，关闭命令立即生效指令 1，立刻生效，即未到达目标位置也可执行下次运动
  * @param  abs_rel:    运动模式。 设为0，绝对运动模式；设为1，相对运动模式
  * @retval None.
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
static void motor_profile_position(int32_t position,int32_t speed,bool abs_rel,bool immediately)
{
  Target_position = position;
  Profile_velocity = speed;
  SYNC_DELAY;

  //由于命令触发是正缘触发，因此必须先将 Bit 4切为 off
  Controlword = (CONTROL_WORD_ENABLE_OPERATION);
  SYNC_DELAY;

  if(immediately == false)//设置为命令触发模式，不立刻生效
  {
    
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  }
  else//设置为命令触发模式，立刻生效
  {
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  }

  if(abs_rel == false)//设置为绝对运动
  {
    Controlword &= ~ABS_REL;
  }
  else//设置为相对运动
  {
    Controlword |=  ABS_REL;
  }
}
/**
  * @brief  控制电机以插补位置模式运动
  * @param  position:   位置    单位PUU 脉冲个数
  * @param  speed:      速度    单位PUU/sec
  * @retval None.
  * @note   None
*/
static void motor_interpolation_position (int32_t position,int32_t speed,bool abs_rel,bool immediately)
{
  Target_position = position;
  Profile_velocity = speed;
  SYNC_DELAY;

  //由于命令触发是正缘触发，因此必须先将 Bit 4切为 off
  Controlword = (CONTROL_WORD_ENABLE_OPERATION);
  SYNC_DELAY;

  if(immediately == false)//设置为命令触发模式，不立刻生效
  {
    
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  }
  else//设置为命令触发模式，立刻生效
  {
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//由于命令触发是正缘触发，Bit 4切为再切至 on。 
  }

  if(abs_rel == false)//设置为绝对运动
  {
    Controlword &= ~ABS_REL;
  }
  else//设置为相对运动
  {
    Controlword |=  ABS_REL;
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
*/
static void motor_state(void)
{
  LOG_I("Mode operation:%d",Modes_of_operation);
	LOG_I("ControlWord 0x%0X", Controlword);
  LOG_I("StatusWord 0x%0X", Statusword);
  
  if(CANOPEN_GET_BIT(Statusword , FAULT))
    LOG_E("motor fault!");
  if(!CANOPEN_GET_BIT(Statusword , READ_QUICK_STOP))//为0急停
    LOG_W("motor quick stop!");
  if(CANOPEN_GET_BIT(Statusword , WARNING))
    LOG_W("motor warning!");
  if(CANOPEN_GET_BIT(Statusword , TARGET_REACHED))
    LOG_I("motor target reached!");
  if(CANOPEN_GET_BIT(Statusword , POSITIVE_LIMIT))
    LOG_W("motor touch  positive limit!");
  if(CANOPEN_GET_BIT(Statusword , NEGATIVE_LIMIT))
    LOG_W("motor touch  negative limit!");
  if(CANOPEN_GET_BIT(Statusword , FOLLOWING_ERROR))
  {
    if(Modes_of_operation == PROFILE_POSITION_MODE)
      LOG_E("motor following error!");
  }
  if(Alarm_code)
    LOG_E("motor alarm code is AL.%03X",Alarm_code);
  
	LOG_I("current position %d PUU", Position_actual_value);  //注意为0X6064，0X6063为增量式位置
	LOG_I("current speed %.1f RPM", Velocity_actual_value / 10.0f);//注意单位为0.1rpm
}
#ifdef RT_USING_MSH
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
#define MOTOR_CMD_RELMOVE               3
  size_t i = 0;

  const char* help_info[] =
    {
            [MOTOR_CMD_ON]             = "motor on                                                  - Enable motor control",
            [MOTOR_CMD_OFF]            = "motor off                                                 - Disenable motor control",
            [MOTOR_CMD_STATE]          = "motor state                                               - Display motor status.",
            [MOTOR_CMD_RELMOVE]        = "motor relmove [position] <speed> <abs/rel> <immediately>  - Setting motor motion position,speed,0:abs/1:rel,immediately.",
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
          MODE_OPERATION mode;
          if(argc <= 2) 
          {
              rt_kprintf("Usage: motor [mode] -pp ip hm pv pt\n");
              return;
          }
          if (!strcmp(argv[2], "pp"))
          {
            mode = PROFILE_POSITION_MODE;
          }
          else if (!strcmp(argv[2], "ip"))
          {
            mode = INTERPOLATED_POSITION_MODE;
          }
          else
          {
              rt_kprintf("Usage: motor [mode] -pp ip hm pv pt\n");
              return;
          }
          motor_on(mode);
        }
        else if (!strcmp(operator, "off"))
        {
            motor_off();
        }
        else if (!strcmp(operator, "state"))
        {
            motor_state();
        }
        else if (!strcmp(operator, "relmove"))
        {
            int32_t position;
            //RPM:60
            int32_t speed = PULSES_PER_TURN;//设置默认速度以1转每秒运行,单位：PUU/sec 
            bool immediately = false;
            bool abs_rel     = false;
            if(argc <= 2) 
            {
                rt_kprintf("Usage: motor_relmove [position] <speed> <0:abs/1:rel> <immediately>\n");
                return;
            }

            position = atoi(argv[2]);
            if(argc > 3) 
            {
                speed = atoi(argv[3]);
            }
            if(argc > 4) 
            {
                abs_rel = atoi(argv[4]);
            }
            if(argc > 5) 
            {
                immediately = atoi(argv[5]);
            }
            rt_kprintf("move to position: %d, speed: %d\n", position, speed);
            rt_kprintf("abs_rel: %d, immediately: %d\n", abs_rel, immediately);
            motor_profile_position(position,speed,abs_rel,immediately);
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
  size_t i = 0;

    if (argc < 2)
    {
        rt_kprintf("Usage: canoepn_CW [word] --shutdown/on/en/dis\n");
        rt_kprintf("on                       --SWITCH ON\n");
        rt_kprintf("en                       --ENABLE OPERATION\n");
        rt_kprintf("dis                      --DISABLE VOLTAGE\n");
        return;
    }
    else
    {
        const char *operator = argv[1];
        if (!strcmp(operator, "shutdown"))
        {
            Controlword = CONTROL_WORD_SHUTDOWN;
        }
        else if(!strcmp(operator, "on"))
        {
            Controlword = CONTROL_WORD_SWITCH_ON;
        }
        else if(!strcmp(operator, "en"))
        {
            Controlword = CONTROL_WORD_ENABLE_OPERATION;
        }
        else if(!strcmp(operator, "dis"))
        {
            Controlword = CONTROL_WORD_DISABLE_VOLTAGE;
        }
        else
        {
          rt_kprintf("Usage: canoepn_CW [word] --shutdown/on/en/dis\n");
          rt_kprintf("on                       --SWITCH ON\n");
          rt_kprintf("en                       --ENABLE OPERATION\n");
          rt_kprintf("dis                      --DISABLE VOLTAGE\n");
          return;
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_CONTROL_WORD,canopen_cw,canopen control word.);
#endif