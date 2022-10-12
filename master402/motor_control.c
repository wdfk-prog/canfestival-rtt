#include <stdint.h>
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
1  Profile Position Mode (λ�ù滮ģʽ) 
  1. �趨ģʽ��OD 6060h = 01h��Ϊλ�ÿ���ģʽ�� 
  2. �趨Ŀ��λ�ã�OD 607Ah (��λ��PUU)�� 
  PUU(Pulse of User Unit),�˵�λΪ�������ӳ��ֱ������ŵ���
  3. �趨�ٶ����OD 6081h (��λ��PUU/sec)�� //��Ĭ��ֵ
  4. �趨���ٶ�ʱ��б�ʣ�OD 6083h (��λ��ms)��//��Ĭ��ֵ
  5. �趨���ٶ�ʱ��б�ʣ�OD 6084h (��λ��ms)��//��Ĭ��ֵ
  6. �趨����ָ�OD 6040h�����������²������������ 6.1�� 6.2��Ϊ��ʹ������ 
  ��״̬�� (state machine) ����׼��״̬��״̬��˵��������½� 12.4�� OD 6040h
  ˵���� 
  ����  ˵�� 
  6.1  Shutdown (�ر�) 
  6.2  Switch on (�ŷ� Servo On׼��) 
  6.3  Enable Operation (�ŷ� Servo On) 
  6.4  ����� (��Ե����) 1 1 1 1 1��bit0~bit4��
  bitλ  ����          ֵ   ˵��
  4      ���趨��      0     û�ге�Ŀ��λ��
                       1     ����Ŀ��λ��
  5      ���������趨  0     ���ʵ�ʶ�λ��Ȼ��ʼ��һ����λ
                       1     �ж�ʵ�ʶ�λ����ʼ��һ����λ
  6      abs / rel     0     Ŀ��λ����һ������ֵ
                       1     Ŀ��λ����һ�����ֵ
  7      ֹͣ          0     ִ�ж�λ
                       1     ���������ٵ�ֹͣ��(���û����������֧��)
  7. ����ɵ�һ���˶����������Ҫִ����һ���˶����������趨Ŀ��λ�á��ٶȵ� 
  ������ 
  8. �趨����ָ�OD 6040h���������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  ������ on�� 
  ����  ˵�� 
  8.1   Enable Operation (�ŷ� Servo On) 
  8.2   ����� (��Ե����) 
  ��ȡ��������Ϣ�� 
  1. ��ȡ OD 6064hȡ��Ŀǰ�������λ�á� 
  2. ��ȡ OD 6041hȡ����������״̬������ following error (׷�����)��set-point 
  acknowledge (�յ�����֪ͨ) �� target reached (����Ŀ��֪ͨ)
  * @author
  ̨��A3�������ΪCANOPENģʽ
  1.�ָ���������
  �û��������в������� CANopen��λ���� ASDA-A3�ŷ��������� 
  1. �趨 CANopenģʽ�������� P1.001��Ϊ 0x0C�� 
  2. �趨�ڵ� ID���� P3.000��Χ��Ϊ 01h ~ 7Fh�� 
  3. ������ P3.001��Ϊ 0403h��P3.001.Z�趨���� 1 Mbps (0��125 Kbps�� 
  1��250 Kbps��2��500 Kbps��3��750 Kbps��4��1 Mbps)�� 
  4. ���齫 P3.012.Z�趨Ϊ 1����ʵ�ֽ��±�����ϵ籣�ֵĹ��ܡ� 
  ���������������µ���ǽ���ͨѶ���ú��±�� P������ά�ֱ������趨��
  ������� CANopen / DMCNET / EtherCAT��������ֵ��
  5. ���鿪����̬��բ���ܣ�P1.032 = 0x0000��
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
/*0x6040����ָ�� Controlword ״̬λ����*/
typedef enum
{
  WRITE_SWITCH_ON             = 1 << 0,//���¿���
  EN_VOLTAGE                  = 1 << 1,//ʹ�ܵ�Դ
  QUICK_STOP                  = 1 << 2,//��ͣ
  EN_OPERATION                = 1 << 3,//����ʹ��
  FAULT_RESET                 = 1 << 7,//��λ
  HALT                        = 1 << 8,//Ӳ������
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
/* Private define ------------------------------------------------------------*/
/*ʹ�������ӳ�ȥ���Ĳ��������ܱ�֤�����Ѿ�����
���Ĳ���ģʽ������ʹ��0X6061�鿴ģʽ��ȡ��֤���ĳɹ�
�����̣߳��жϳ�ʱ���ָ�����
*/
#define SYNC_DELAY            rt_thread_delay(RT_TICK_PER_SECOND/50)//������ʱ
#define ENCODER_RES           (pow(2,24))                           //�������ֱ��� 16,777,216
#define PULSES_PER_TURN       (100000)                              //������/ת
//���ӳ��ֱ� 6093h sub1/6093h sub2
#define ELECTRONIC_GEAR_RATIO (ENCODER_RES / 100000)                //���ӳ��ֱ�
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
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��ģʽ
  * @param  mode:����ģʽ ������鿴�ṹ��@MODE_OPERATION
  * @retval None
  * @note   û�нڵ�ѡ��
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
  * @brief  ���Ƶ���ر�.
  * @param  None.
  * @retval None.
  * @note   ����������ͣ
  ��ͣ����ʱ��б��  6085h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
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
  * @brief  ���Ƶ����λ�ù滮ģʽ�˶�
  * @param  position:   λ��    ��λPUU �������
  * @param  speed:      �ٶ�    ��λPUU/sec
  * @param  immediately:����������Чָ� ��Ϊ 0���ر�����������Чָ�� 1��������Ч����δ����Ŀ��λ��Ҳ��ִ���´��˶�
  * @param  abs_rel:    �˶�ģʽ�� ��Ϊ0�������˶�ģʽ����Ϊ1������˶�ģʽ
  * @retval None.
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
static void motor_profile_position(int32_t position,int32_t speed,bool abs_rel,bool immediately)
{
  Target_position = position;
  Profile_velocity = speed;
  SYNC_DELAY;

  //�������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  Controlword = (CONTROL_WORD_ENABLE_OPERATION);
  SYNC_DELAY;

  if(immediately == false)//����Ϊ�����ģʽ����������Ч
  {
    
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }
  else//����Ϊ�����ģʽ��������Ч
  {
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }

  if(abs_rel == false)//����Ϊ�����˶�
  {
    Controlword &= ~ABS_REL;
  }
  else//����Ϊ����˶�
  {
    Controlword |=  ABS_REL;
  }
}
/**
  * @brief  ���Ƶ���Բ岹λ��ģʽ�˶�
  * @param  position:   λ��    ��λPUU �������
  * @param  speed:      �ٶ�    ��λPUU/sec
  * @retval None.
  * @note   None
*/
static void motor_interpolation_position (int32_t position,int32_t speed,bool abs_rel,bool immediately)
{
  Target_position = position;
  Profile_velocity = speed;
  SYNC_DELAY;

  //�������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  Controlword = (CONTROL_WORD_ENABLE_OPERATION);
  SYNC_DELAY;

  if(immediately == false)//����Ϊ�����ģʽ����������Ч
  {
    
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }
  else//����Ϊ�����ģʽ��������Ч
  {
    Controlword = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }

  if(abs_rel == false)//����Ϊ�����˶�
  {
    Controlword &= ~ABS_REL;
  }
  else//����Ϊ����˶�
  {
    Controlword |=  ABS_REL;
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
*/
static void motor_state(void)
{
  LOG_I("Mode operation:%d",Modes_of_operation);
	LOG_I("ControlWord 0x%0X", Controlword);
  LOG_I("StatusWord 0x%0X", Statusword);
  
  if(CANOPEN_GET_BIT(Statusword , FAULT))
    LOG_E("motor fault!");
  if(!CANOPEN_GET_BIT(Statusword , READ_QUICK_STOP))//Ϊ0��ͣ
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
  
	LOG_I("current position %d PUU", Position_actual_value);  //ע��Ϊ0X6064��0X6063Ϊ����ʽλ��
	LOG_I("current speed %.1f RPM", Velocity_actual_value / 10.0f);//ע�ⵥλΪ0.1rpm
}
#ifdef RT_USING_MSH
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
            int32_t speed = PULSES_PER_TURN;//����Ĭ���ٶ���1תÿ������,��λ��PUU/sec 
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