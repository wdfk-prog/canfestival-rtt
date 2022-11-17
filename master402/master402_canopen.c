/**
 * @file master402_canopen.c
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
/* Includes ------------------------------------------------------------------*/
#include "master402_canopen.h"
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include <rtthread.h>
#include <finsh.h>

#include "canfestival.h"
#include "canopen_callback.h"
#include "timers_driver.h"

#include "motor_control.h"
/* Private typedef -----------------------------------------------------------*/
/* 
 * �ڵ�����
*/
typedef struct
{
  uint8_t nodeID;
  char name[RT_NAME_MAX];
  e_nodeState *nmt_state;
}node_list;
/* 
 * �ڵ�����״̬�ṹ��
*/
typedef struct 
{
  node_list *list;
	uint8_t state;
	uint8_t try_cnt;
  uint16_t err_code;
  uint8_t errSpec[5];
  struct rt_semaphore finish_sem;
}node_config_state;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/*DS301����Э��д��ڵ�������ʽ
  NodeID  : �ڵ�
  Interval: �������*/
#define HEARTBEAT_FORMAT(NodeID,Interval) (((NodeID) << 16) | (Interval))
/*DS301Э�� 31λ��1�ر�PDO��RPDO/TPDO������
  NodeID  : �ڵ�
  Default :  Ĭ��ֵ*/
#define PDO_DISANBLE(NodeID,Default) ((1<<31) | (Default) | (NodeID))
/*DS301Э�� 31λ��1ʹ��PDO��RPDO/TPDO������
  NodeID  : �ڵ�
  Default :  Ĭ��ֵ*/
#define PDO_ENANBLE(Default,NodeID) ((Default) | (NodeID))
/*DS301Э�� 31λ��1ʹ��SYNCͬ����Ϣ
  NodeID  : �ڵ�*/
#define SYNC_ENANBLE(NodeID) ((1 << 30) | (NodeID))
/* Private variables ---------------------------------------------------------*/
CO_Data *OD_Data = &master402_Data;
static node_list can_node[MAX_NODE_COUNT - 1] = 
{
  {MASTER_NODEID,   "master",},
  {SERVO_NODEID_1,    "walk",},
  {SERVO_NODEID_2,    "turn",},
};

static s_BOARD agv_board  = {CANFESTIVAL_CAN_DEVICE_NAME,"1M"};//û��,����CANFESTIVAL
static node_config_state slave_conf[MAX_NODE_COUNT - 2];//����״̬
/* Private function prototypes -----------------------------------------------*/
static void config_node_param(uint8_t nodeId, node_config_state *conf);
/***********************��ʼ������״̬����**************************************************/
/**
  * @brief  ��ʼ�ڵ�
  * @param  None
  * @retval None
  * @note   
*/
static void InitNodes(CO_Data* d, UNS32 id)
{
	setNodeId(OD_Data,MASTER_NODEID);
	setState(OD_Data, Initialisation);
}
/**
  * @brief  ��ʼ��canopen
  * @param  None
  * @retval None
  * @note   
*/
static int canopen_init(void)
{
  OD_Data->heartbeatError = master402_heartbeatError;
	OD_Data->initialisation = master402_initialisation;
	OD_Data->preOperational = master402_preOperational;
	OD_Data->operational = master402_operational;
	OD_Data->stopped   = master402_stopped;
	OD_Data->post_sync = master402_post_sync;
	OD_Data->post_TPDO = master402_post_TPDO;
	OD_Data->storeODSubIndex = (storeODSubIndex_t)master402_storeODSubIndex;
	OD_Data->post_emcy = (post_emcy_t)master402_post_emcy;

  PDODisable(OD_Data,1);
  PDODisable(OD_Data,3);

	canOpen(&agv_board, OD_Data);
	initTimer();

	// Start timer thread
	StartTimerLoop(&InitNodes);
  //�ҹ����ָ��
  for(uint8_t i = 0; i < MAX_NODE_COUNT - 2; i++)
  {
     slave_conf[i].list = &can_node[i+1];
     can_node[i+1].nmt_state = &OD_Data->NMTable[i+2];
  }
  can_node[0].nmt_state = &OD_Data->nodeState;

	return RT_EOK;
}
INIT_APP_EXPORT(canopen_init);
/**
  * @brief  None
  * @param  None
  * @retval None
  * @note   
*/
static void Exit(CO_Data* d, UNS32 id)
{

}
/************************�ⲿ���ú���********************************************************/
/**
  * @brief  ��ȡ�ڵ�����
  * @param  des:Ŀ���ַ
  * @param  nodeID:�ڵ�ID
  * @retval des:Ŀ���ַ
  * @note   ������ڵ�ID����ȷ�������ؿ�ָ��
*/
char *nodeID_get_name(char* des,uint8_t nodeID)
{
  char* r = des;

  if(des == NULL)
    return RT_NULL;

  if(nodeID == can_node[nodeID - 1].nodeID)
  {
    rt_memcpy(des,can_node[nodeID - 1].name,RT_NAME_MAX);
    return des;
  }
  else
    return RT_NULL;
}
/**
  * @brief  ��ȡ�ڵ�NMT״̬
  * @param  nodeID:�ڵ�ID
  * @retval ����NMT״̬
  * @note   ������ڵ�ID����ȷ��������0XFF
*/
e_nodeState nodeID_get_nmt(uint8_t nodeID)
{
  if(nodeID >= MAX_NODE_COUNT)
    return 0XFF;
  if(nodeID == can_node[nodeID - 1].nodeID)
    return *can_node[nodeID - 1].nmt_state;
  else
    return 0XFF;
}
/**
  * @brief  ���ýڵ�������.
  * @param  nodeID:�ڵ�ID
  * @param  errcode:�������
  * @retval �ɹ�����RT_EOK��ʧ�ܷ���-RT_ERROR.
  * @note   None.
*/
UNS8 nodeID_set_errcode(uint8_t nodeID,uint16_t errcode)
{
  if(nodeID >= MAX_NODE_COUNT)
  {
    return -RT_ERROR;
  }
  else
  {
    slave_conf[nodeID - 2].err_code = errcode;
    return RT_EOK;
  }
}
/**
  * @brief  �鿴�ڵ�������.
  * @param  nodeID:�ڵ�ID
  * @retval �ɹ����ؽڵ�������.ʧ�ܷ���0
  * @note   None.
*/
uint16_t nodeID_get_errcode(uint8_t nodeID)
{
  if(nodeID >= MAX_NODE_COUNT)
    return 0;
  if(nodeID == MASTER_NODEID)
    return 0;
  else
    return slave_conf[nodeID - 2].err_code;
}
/**
  * @brief  ���ýڵ�������
  * @param  nodeID:�ڵ�ID
  * @param  errSpec:�������
  * @retval �ɹ�����RT_EOK,ʧ�ܷ���-RT_ERROR
  * @note   None.
*/
UNS8 nodeID_set_errSpec(uint8_t nodeID,const uint8_t errSpec[5])
{
  if(nodeID >= MAX_NODE_COUNT)
  {
    return -RT_ERROR;
  }
  else
  {
    rt_memcpy(slave_conf[nodeID - 2].errSpec,errSpec,5);
    return RT_EOK;
  }
}
/**
  * @brief  ��ȡ�ڵ�������
  * @param  des:Ŀ���ַ
  * @param  nodeID:�ڵ�ID
  * @retval des:Ŀ���ַ,ʧ�ܷ���RT_NULL
  * @note   None.
*/
char* nodeID_get_errSpec(char* des,uint8_t nodeID)
{
  char* r = des;

  if(des == NULL)
    return RT_NULL;

  if(nodeID == MASTER_NODEID || nodeID > MAX_NODE_COUNT || nodeID == 0)
  {
    return RT_NULL;
  }
  else
  {
    if(slave_conf[nodeID - 2].errSpec[0] + slave_conf[nodeID - 2].errSpec[1] + \
       slave_conf[nodeID - 2].errSpec[2] + slave_conf[nodeID - 2].errSpec[3] + 
       slave_conf[nodeID - 2].errSpec[4])
    {
      rt_memcpy(des,slave_conf[nodeID - 2].errSpec,5);
      return des;
    }
    else
    {
      return RT_NULL;
    }
  }
}
#ifdef RT_USING_MSH
/**
  * @brief  ��ӡ�ڵ�״̬
  * @param  state���ڵ�״̬
  * @retval None.
  * @note   None.
*/
static void printf_state(e_nodeState state)
{
  switch(state)
  {
    case Initialisation:
      rt_kprintf("Initialisation");
      break;
    case Stopped:
      rt_kprintf("Stopped");
      break;
    case Operational:
      rt_kprintf("Operational");
      break;
    case Pre_operational:
      rt_kprintf("Pre_operational");
      break;
    case Disconnected:
      rt_kprintf("Disconnected");
      break;   
    case Unknown_state:
      rt_kprintf("Unknown_state");
      break;
    default:
      rt_kprintf("%d",state);
      break;
  }
}
/**
  * @brief  ��ѯ�����nmt״̬
  * @param  None
  * @retval None
  * @note   None
*/
static void cmd_canopen_nmt(uint8_t argc, char **argv) 
{
#define NMT_CMD_LIST                    0
#define NMT_CMD_GET                     1
#define NMT_CMD_SLAVE_SET               2
#define NMT_CMD_PRE                     NMT_CMD_SLAVE_SET+2

  const char* help_info[] =
    {
        [NMT_CMD_LIST]           = "canopen_nmt list                        - Print all node NMT.",
        [NMT_CMD_GET]            = "canopen_nmt get [nodeID]                - Get nodeID state.",
        [NMT_CMD_SLAVE_SET]      = "canopen_nmt s   [operational] <nodeID>  - Slvae  NMT set start/stop/pre/rn/rc.",
        [NMT_CMD_SLAVE_SET+1]    = "                                        - rn:Reset_Node rc:Reset_Comunication.",
        [NMT_CMD_PRE]            = "canopen_nmt m   [operational] <nodeID>  - Master NMT set start/stop/pre.",
    };

    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        for (size_t i = 0; i < sizeof(help_info) / sizeof(char*); i++)
        {
            rt_kprintf("%s\n", help_info[i]);
        }
        rt_kprintf("\n");
        return;
    }
    else
    {
        const char *operator = argv[1];
        if (!strcmp(operator, "list"))//��ӡ���нڵ�NMT״̬
        {
          const char *item_title = "NMT";
          int maxlen = RT_NAME_MAX;    

          rt_kprintf("%-*.*s nodeID  status\n",maxlen,maxlen,item_title);
          rt_kprintf("-------- ------  -------\n");
          for(UNS8 i = 0; i < MAX_NODE_COUNT - 1; i++)
          {
              rt_kprintf("%-*.*s 0X%02X    ",maxlen,maxlen,can_node[i].name,can_node[i].nodeID);
              printf_state(*can_node[i].nmt_state);
              rt_kprintf("\n");
          }
        }
        else if (!strcmp(operator, "get"))//��ѯ�ڵ�NMT
        {
          if(argc <= 2) 
          {
              rt_kprintf("Usage:canoepn get [nodeID]  - Get nodeID state\n");
              return;
          }
          uint8_t nodeid = atoi(argv[2]);
          rt_kprintf("%s 0X%02X    ",can_node[nodeid - 1].name,can_node[nodeid - 1].nodeID);
          rt_kprintf("nodeID state is ");
          printf_state(*can_node[nodeid - 1].nmt_state);
          rt_kprintf("\n");       
        }
        else if (!strcmp(operator, "s"))//�ӻ�����NMT
        {
          uint8_t nodeID = 2;
          UNS8    cs = 0X00; 
          if(argc <= 2) 
          {
              rt_kprintf("Usage:canopen_nmt s   [operational] <nodeID>  - Slvae  NMT set start/stop/pre/rn/rc.\n");
              rt_kprintf("                                              - rn:Reset_Node rc:Reset_Comunication.\n");
              return;
          }
          if(argc > 2)
          {
            const char *operator = argv[2];
            if (!strcmp(operator, "start"))
            {
              cs = NMT_Start_Node;
            }
            else if(!strcmp(operator, "stop"))
            {
              cs = NMT_Stop_Node;
            } 
            else if(!strcmp(operator, "pre"))
            {
              cs = NMT_Enter_PreOperational;
            } 
            else if(!strcmp(operator, "rn"))
            {
              cs = NMT_Reset_Node;
            } 
            else if(!strcmp(operator, "rc"))
            {
              cs = NMT_Reset_Comunication;
            }
            else
            {
              rt_kprintf("Usage:canopen_nmt s   [operational] <nodeID>  - Slvae  NMT set start/stop/pre.\n");
              return;
            }
          }
          if(argc > 3)
          {
             nodeID = atoi(argv[4]);
          }
          masterSendNMTstateChange(OD_Data,nodeID,cs);
        }
        else if (!strcmp(operator, "m"))//��������NMT
        {
          const char *operator = argv[2];
          if(argc <= 2) 
          {
              rt_kprintf("Usage:canopen_nmt m   [operational] <nodeID>  - Master NMT set start/stop/pre.\n");
              return;
          }
          if(argc > 2)
          {
            e_nodeState newState;
            if (!strcmp(operator, "start"))
            {
              newState = Operational;
            }
            else if (!strcmp(operator, "stop"))
            {
              newState = Stopped;
            }
            else if (!strcmp(operator, "pre"))
            {
              newState = Pre_operational;
            }
            else
            {
                rt_kprintf("Usage:canopen_nmt m   [operational] <nodeID>  - Master NMT set start/stop/pre/rn/rc.\n");
                rt_kprintf("                                              - rn:Reset_Node rc:Reset_Comunication.\n");
                return;
            }
            setState(OD_Data, newState);
          }
        }
        else
        {
          rt_kprintf("Usage:\n");
          for (size_t i = 0; i < sizeof(help_info) / sizeof(char*); i++)
          {
              rt_kprintf("%s\n", help_info[i]);
          }
          rt_kprintf("\n");
          return;
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_canopen_nmt,canopen_nmt,canoepn nmt cmd.);
#endif
/***********************Ԥ����״̬����**************************************************/
/**
  * @brief  ���ò�����ɻص�
  * @param  None
  * @retval None
  * @note   None
*/
static void config_node_param_cb(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
	node_config_state *conf;

	conf = &slave_conf[nodeId - 2];
	res = getWriteResultNetworkDict(OD_Data, nodeId, &abortCode);
	closeSDOtransfer(OD_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED)
	{
		conf->try_cnt++;
		if(conf->try_cnt < 3)
		{
      LOG_W(" write SDO failed!  nodeId = %d, abortCode = 0x%08X,step = %d", nodeId, abortCode,conf->state + 1);
			config_node_param(nodeId, conf);
		}
		else
		{
			rt_sem_release(&(conf->finish_sem));
			conf->state = 0;
			conf->try_cnt = 0;
      conf->err_code = NODEID_CONFIG_NO_RESPOND;
			LOG_E("SDO config try count > 3, config failed!,error = %d",conf->err_code);
		}
	}
	else
	{
    conf->err_code = NODEID_CONFIG_SUCCESS;
		conf->state++;
		conf->try_cnt = 0;
		config_node_param(nodeId, conf);
	}
}
/**
  * @brief  �����ŷ�����
  * @param  None
  * @retval None
  * @note   �������ȴ��������
*/
void config_node(uint8_t nodeId)
{
  if(Write_SLAVE_control_word(nodeId,0x80) == 0xFF)//��ʼ�����д�������
  {
      LOG_E("nodeId:%d,Failed to clear error.The current node is not in operation",nodeId);
      rt_sem_init(&(slave_conf[nodeId - 2].finish_sem), "servocnf", 0, RT_IPC_FLAG_FIFO);
      if(rt_sem_take(&(slave_conf[nodeId - 2].finish_sem), SDO_REPLY_TIMEOUT) != RT_EOK)
      {
        slave_conf[nodeId - 2].err_code = NODEID_CONFIG_NO_RESPOND;
        //�������ִ�У������ϵ�������账��
        LOG_W("Waiting for the repair to complete, CAN communication is currently unavailable");
        master402_fix_config_err(OD_Data,nodeId);
      }
      rt_sem_detach(&(slave_conf[nodeId - 2].finish_sem));
  }
  else
  {
      slave_conf[nodeId - 2].state = 0;
      slave_conf[nodeId - 2].try_cnt = 0;
      rt_sem_init(&(slave_conf[nodeId - 2].finish_sem), "servocnf", 0, RT_IPC_FLAG_FIFO);

      EnterMutex();
      LOG_I("The configuration starts for node %d",nodeId);
      config_node_param(nodeId, &slave_conf[nodeId - 2]);
      LeaveMutex();
      rt_sem_take(&(slave_conf[nodeId - 2].finish_sem), RT_WAITING_FOREVER);
      rt_sem_detach(&(slave_conf[nodeId - 2].finish_sem));

      if(slave_conf[nodeId - 2].err_code != 0X00)//�����ô����µ��˳�
      {
        LOG_E("Failed to configure the dictionary for node %d",nodeId);
        if(slave_conf[nodeId - 2].err_code == NODEID_CONFIG_NO_SEND)
        {
          LOG_E("The configuration was not sent because the local dictionary failed");
        }
        else if(slave_conf[nodeId - 2].err_code == NODEID_CONFIG_NO_RESPOND)
        {
          LOG_E("The configuration reply did not respond, and the node dictionary failed");
        }
        LOG_W("Waiting for the repair to complete, CAN communication is currently unavailable");
        master402_fix_config_err(OD_Data,nodeId);
        return; //�˳��߳�
      }
      else
      {
        UNS32 errorCode,map_val, size = SDO_MAX_LENGTH_TRANSFER;
        UNS8 data_type;
        errorCode = readLocalDict(OD_Data, 0x1016, nodeId - 1, &map_val, &size, &data_type, 0);
        if(errorCode == OD_SUCCESSFUL)
        {
          /**д������������/���ն��ж�������ʱʱ��  DS301����**/
          /**�и�ʽ���壬�ֵ乤��û��֧�֣���Ҫ�Լ�д��**/
          UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(nodeId,CONSUMER_HEARTBEAT_TIME);//д��ڵ������ʱ��
          errorCode = writeLocalDict(OD_Data, 0x1016, nodeId - 1, &consumer_heartbeat_time, &size, 0);
          if(errorCode != OD_SUCCESSFUL)
            LOG_E("index:0X%04X,subIndex:0X%X,write Local Dict false,abort code is 0X%08X",0x1016,nodeId - 1,errorCode);
        }
        else
        {
          LOG_E("index:0X%04X,subIndex:0X%X,write Local Dict false,abort code is 0X%08X",0x1016,nodeId - 1,errorCode);
          LOG_W("Node %d is not configured with a consumer heartbeat",nodeId);
        }
        //�ڵ�������״̬
        masterSendNMTstateChange(OD_Data, nodeId, NMT_Start_Node);
        LOG_I("Node %d configuration Complete",nodeId);
        rt_thread_mdelay(200);//ȷ��NMT�����·��ɹ�
      }
  }
}
/**
  * @brief  ���õ�ǰ�ڵ�
  * @param  None
  * @retval None
  * @note   None
*/
static void config_single_node(void *parameter)
{
	uint32_t nodeId;
	nodeId = (uint32_t)parameter;
	config_node(nodeId);
}
/**
  * @brief  �ӻ����ߴ�����
  * @param  None
  * @retval None
  * @note   ����config_single_servo�߳�
*/
static void slaveBootupHdl(CO_Data* d, UNS8 nodeId)
{
  master_resume_start(d,nodeId);//�Ƿ���Ҫ�ָ�����ģʽ
	rt_thread_t tid;
  LOG_I("Node %d has gone online",nodeId);
  //�ж��ź����Ƿ��ʼ��
  if(!rt_list_isempty(&slave_conf[nodeId - 2].finish_sem.parent.suspend_thread))
  {
    LOG_I("Node %d is powered on before the MCU",nodeId);
    rt_sem_release(&(slave_conf[nodeId - 2].finish_sem));
  }
  else
  {
    LOG_I("After the MCU is powered on, node %d is powered on",nodeId);
  }
	tid = rt_thread_create("co_cfg", config_single_node, (void *)(int)nodeId, 2048, 12 + nodeId, 2);
	if(tid == RT_NULL)
	{
		LOG_E("canopen config thread start failed!");
	}
	else
	{
		rt_thread_startup(tid);
	}
}
/**
  * @brief  д�뱾���ֵ���������л�nmt����
  * @param  None
  * @retval None
  * @note   nmt״̬�л�Ϊoperation״̬�����̣߳�������ɾ���߳�
*/
void canopen_start_thread_entry(void *parameter)
{
  assert(parameter);
	UNS32 size,errorCode;
	UNS8 data_type;
  CO_Data *d = (CO_Data *)parameter;
  /*д��ڵ��ֵ�*/
//  UNS8 i = 0;//����ȡ��ע�����У�ע�����С��������ڵ��ʼ��
  for (UNS8 i = 0; i < MAX_NODE_COUNT - 2; i++)
  {
    config_node(slave_conf[i].list->nodeID);//��ʼ��ʱʹ�ô����ã�����sem��ʼ����ɾ������
  }
  /*д�뱾���ֵ�*/
	d->post_SlaveBootup = slaveBootupHdl;
  /**д������������/���Ͷ���������ʱ��  DS301����**/
  UNS16 producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
	size = 2;
  errorCode = writeLocalDict(d, 0x1017, 0, &producer_heartbeat_time, &size, 0);
	if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,write Local Dict false,abort code is 0X%08X",0x1017,0,errorCode);
    setState(d, Stopped);//��վ�������������������ͽ��в��������ܷ�����վ���ߣ��ڵ㲻֪����������
    return ;
  }
  //��վ�������ģʽ
  setState(d, Operational);
  /**�и�ʽ���壬�ֵ乤��û��֧�֣���Ҫ�Լ�д��**/
  UNS32 sync_id;
	data_type = uint32;
	size = 4;
  errorCode = readLocalDict(d, 0x1005, 0, &sync_id, &size, &data_type, 0);
	if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",0x1005,0,errorCode);
    setState(d, Stopped);//��վ������ͬ����Ϣ�����ܲ����ڵ�
    return ;
  }
  else
  {
    sync_id = SYNC_ENANBLE(sync_id);//DS301 30λ��1 ������CANopen�豸����ͬ����Ϣ
    errorCode = writeLocalDict(d, 0x1005, 0, &sync_id, &size, 0);
    if(errorCode != OD_SUCCESSFUL)
    {
      LOG_E("index:0X%04X,subIndex:0X%X,write Local Dict false,abort code is 0X%08X",0x1005,0,errorCode);
      setState(d, Stopped);//��վ������ͬ����Ϣ�����ܲ����ڵ�
      return ;
    }
  }
}
/******************************д��ڵ��ֵ��������**********************************/
/**
  * @brief  ������ɻص�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId���ڵ�ID
  * @retval FLASE:����,TRUE:�ɹ�.
  * @note   ���޳�ʼ��������ʹ��
*/
static void writeNetworkDictSyncCb(CO_Data* d, UNS8 nodeId) 
{
    node_config_state *conf = &slave_conf[nodeId - 2];
    rt_sem_release(&conf->finish_sem);
}
/**
  * @brief  �첽д��ڵ��ֵ�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId���ڵ�ID
  * @retval FLASE:����,TRUE:�ɹ�.
  * @note   ���޳�ʼ��������ʹ��
*/
static bool writeNetworkDictSync (CO_Data* d, UNS8 nodeId, UNS16 index,
        UNS8 subIndex, UNS32 count, UNS8 dataType, void *data, UNS8 useBlockMode) 
{
    if(nodeId < 1 || nodeId > MAX_NODE_COUNT - 1) 
    {
      LOG_W("invalid nodeId:%d, should between 2 and %d",nodeId,MAX_NODE_COUNT-1);
      return false;
    }

    int try_cnt = 3;
    node_config_state* conf = &slave_conf[nodeId - 2];
    rt_sem_init(&conf->finish_sem, "servocnf", 0, RT_IPC_FLAG_FIFO);
    
    while(try_cnt--) 
    {
        if(writeNetworkDictCallBack(d, nodeId, index, subIndex,
                count, dataType, data, writeNetworkDictSyncCb, useBlockMode) != 0)
        {
            LOG_W("write SDO failed!  nodeId = %d, index: 0X%04X, subIndex: 0X%X", nodeId, index, subIndex);
            closeSDOtransfer(d, nodeId, SDO_CLIENT);
            continue;
        }

        if(rt_sem_take(&conf->finish_sem, SDO_REPLY_TIMEOUT) != RT_EOK) 
        {
            LOG_W("write SDO timeout!  nodeId = %d, index: 0X%04X, subIndex: 0X%X", nodeId, index, subIndex);
            closeSDOtransfer(d, nodeId, SDO_CLIENT);
            continue;
        }

        UNS32 abortCode = 0;
        UNS8 res = getWriteResultNetworkDict(d, nodeId, &abortCode);
        closeSDOtransfer(d, nodeId, SDO_CLIENT);

        if(res != SDO_FINISHED)
        {
            LOG_W("get SDO write result failed!  nodeId = %d, index: 0X%04X, subIndex: 0X%X, abortCode = 0X%08X", nodeId, index, subIndex, abortCode);
            continue;
        }
        rt_sem_detach(&conf->finish_sem);
        return true;
    }
    rt_sem_detach(&conf->finish_sem);
    return false;
}
/**
  * @brief  ��ʼ���������ȡ�����ֵ������д��ڵ��ֵ�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId:�ڵ�ID
  * @param  local_index:��������
  * @param  local_subIndex:����������
  * @param  slave_Index:�ӻ�����
  * @param  slave_subIndex:�ӻ�������
  * @retval 0XFF:����0X00:�ɹ�.
  * @note   ���޳�ʼ��������ʹ��
            ��ʼ������ʹ�ã��ᵼ���ź����ظ���ʼ��������ϵͳ����
            ��ȡ�����ֵ����ֵ�������ڵ�ñ���ֵ��
*/
static UNS8 read_local_send_node_init_end(CO_Data* d,UNS8 nodeId,UNS16 local_index,UNS8 local_subIndex,UNS16 slave_index,UNS8 slave_subIndex)
{
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS32 pdo_map_val = 0,errorCode = 0; 
  UNS8  dataType = 0;

  errorCode = readLocalDict(d,local_index,local_subIndex,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",local_index,local_subIndex,errorCode);
    return 0xFF;
  }
  errorCode = writeNetworkDictSync(d,nodeId,slave_index,slave_subIndex,size,dataType,&pdo_map_val,0);
  if(errorCode != true)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,write slave Dict false,abort code is 0X%08X",slave_index,slave_subIndex,errorCode);
    return 0xFF;
  }
  return 0x00;
}
/**
  * @brief  ��ȡ�����ֵ�ֵ���������ӻ��ڵ� ����������վTPDO��RPDO��RPDO��TPDO
  * @param  index:��������
  * @param  subIndex:����������
  * @param  nodeId:�ڵ�ID
  * @retval 0XFF:����0X00:�ɹ�..
  * @note   ����������վTPDO��RPDO��RPDO��TPDO
            ��ȡ��Ӧ��ͼӳ�����������ز������������ӻ�
*/
static UNS8 read_local_send_node_pdo_T_R(UNS16 index,UNS8 subIndex,uint8_t nodeId)
{
  UNS32 pdo_map_val;
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS8  dataType;
  UNS32 errorCode;
  UNS16  slave_Index;
  
  if(0X1600 <= index && index <= 0X17FF)//�ӻ� Receive PDO Parameters
  {
    //����վTransmit PDO ParametersѰ�Ҷ�Ӧ����
    slave_Index = index + 0X400;
  }
  else if(0X1A00 <= index && index <= 0X1BFF)//Transmit PDO Parameters
  {
    //����վReceive PDO ParametersѰ�Ҷ�Ӧ����
    slave_Index = index - 0X400;
  }
  errorCode = readLocalDict(OD_Data,slave_Index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",index,subIndex,errorCode);
    return 0XFF;
  }
  else
  {
    return writeNetworkDictCallBack(OD_Data,nodeId,index,subIndex,size,dataType,&pdo_map_val,config_node_param_cb,0);
  }
}
/**
  * @brief  ��ȡ�����ֵ����ò��������ӻ��ڵ�
  * @param  local_index:��������
  * @param  local_subIndex:����������
  * @param  slave_Index:�ӻ�����
  * @param  slave_subIndex:�ӻ�������
  * @param  nodeId:�ڵ�ID
  * @retval 0XFF:����0X00:�ɹ�.
  * @note   ����PDO���÷���[������PDO CNT����]��
            ��ȡ�����ֵ��������С���㴫������
*/
static UNS8 read_local_send_node_pdo(UNS16 local_index,UNS8 local_subIndex,UNS16 slave_Index,UNS8 slave_subIndex,uint8_t nodeId)
{
  UNS32 pdo_map_val;
  UNS32 size = SDO_MAX_LENGTH_TRANSFER;
  UNS8  dataType;
  UNS32 errorCode;
  UNS32 SDO_data;

  errorCode = readLocalDict(OD_Data,local_index,local_subIndex,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",local_index,local_subIndex,errorCode);
    return 0XFF;
  }
  else
  {
    /*
          λ             ���� 
    Bit 0 ~ Bit 7     �������ݳ��� 
    Bit 8 ~ Bit 15    ���������� 
    Bit 16 ~ Bit 31   �������� 
    */
    SDO_data = (local_index << 16) + (local_subIndex << 8) + (size * 8);
    pdo_map_val = 0;size = SDO_MAX_LENGTH_TRANSFER;dataType = 0;//�����������
    errorCode = readLocalDict(OD_Data,slave_Index,slave_subIndex,(void *)&pdo_map_val,&size,&dataType,0);
    if(errorCode != OD_SUCCESSFUL)
    {
      LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,abort code is 0X%X",slave_Index,slave_subIndex,errorCode);
      return 0XFF;
    }
    else
    {
      return writeNetworkDictCallBack(OD_Data,nodeId,slave_Index,slave_subIndex,size,dataType,&SDO_data,config_node_param_cb,0);
    }
  }
}

/**
  * @brief  ��ʼ���׶ζ�ȡ�����ֵ������д��ڵ��ֵ�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId:�ڵ�ID
  * @param  index:��������
  * @param  subIndex:����������
  * @retval 0XFF:����0X00:�ɹ�.
  * @note   ����ʼ��ʹ��
            ������ȡд�룬�����κμ���
*/
static UNS8 read_local_send_node_init_start(UNS16 index,UNS8 subIndex,UNS8 nodeId)
{
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS32 pdo_map_val = 0,errorCode = 0; 
  UNS8  dataType = 0;

  errorCode = readLocalDict(OD_Data,index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",index,subIndex,errorCode);
    return 0xFF;
  }
  return writeNetworkDictCallBack(OD_Data,nodeId,index,subIndex,size,dataType,&pdo_map_val,config_node_param_cb,0);
}
/******************************SDO���ò�������**********************************/
/**
  * @brief  д��ӻ�Modes_of_operation����
  * @param  nodeId:�ڵ�ID
  * @param  mode:Modes_of_operation 
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None.
*/
UNS8 Write_SLAVE_Modes_of_operation(UNS8 nodeId,INTEGER8 mode)
{
  UNS8 errcode = 0;
  if(0 <= *Modes_of_operation_Node[nodeId - 2].map_val && *Modes_of_operation_Node[nodeId - 2].map_val <= 10)
  {
    //����д�뱾���ֵ�
    *Modes_of_operation_Node[nodeId - 2].map_val = mode;
  }
  else
  {
    LOG_E("The Modes_of_operation : %d is out of range. Select a value from 0 to 10",*Modes_of_operation_Node[nodeId - 2].map_val);
    return 0XFF;
  }

  //�������ͽڵ��ֵ�
  if(read_local_send_node_init_end(OD_Data,nodeId,
                          Modes_of_operation_Node[nodeId - 2].index,0x00,//��ȡ���ر���
                          0x6060,0x00) == 0XFF)//�������ӻ�����
    return 0XFF;

  return 0X00;
}
/**
  * @brief  д��ӻ�control_word����
  * @param  nodeId:�ڵ�ID
  * @param  value:Controlword 
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None.
*/
UNS8 Write_SLAVE_control_word(UNS8 nodeId,UNS16 value)
{
  UNS8 errcode = 0;
  //����д�뱾���ֵ�
  *Controlword_Node[nodeId - 2].map_val = value;

  //�������ͽڵ��ֵ�
  if(read_local_send_node_init_end(OD_Data,nodeId,
                          Controlword_Node[nodeId - 2].index,0x00,//��ȡ���ر���
                          0x6040,0x00) == 0XFF)//�������ӻ�����
    return 0XFF;

  return 0X00;
}
/**
  * @brief  д��ӻ�λ�ù滮ģʽ���ٶ�����
  * @param  nodeId:�ڵ�ID
  * @param  speed:�˶��ٶ� ��λ:RPM
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   Profile_velocity ��λPUU/s �趨��Χ UNSIGNED32 Ĭ��ֵ 10000
*/
UNS8 Write_SLAVE_profile_position_speed_set(UNS8 nodeId,UNS32 speed)
{
  UNS8 errcode = 0;
  //����д�뱾���ֵ�
  if(0 <= speed && speed <= 3000)
    Profile_velocity = speed * ELECTRONIC_GEAR_RATIO_NUMERATOR / 60;//
  else
  {
    LOG_E("The Profile_velocity is out of range. Select a value from 0 to 3000");
    return 0XFF;
  }

  //�������ͽڵ��ֵ�
  if(read_local_send_node_init_end(OD_Data,nodeId,
                          0x6081,0x00,//��ȡ���ر���
                          0x6081,0x00) == 0XFF)//�������ӻ�����
        return 0XFF;

  return 0X00;
}
/**
  * @brief  д��ӻ�0X60C2�岹���ڲ���
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   �趨����ͨѶ���� OD 1006h��ͬ
  * �岹����ʱ�� = 60C2H sub1 �� 10^(60C2H sub2)
  * ����������Ҫ�趨�岹����ʱ��Ϊ 10 ms���轫 OD 60C2h sub1 = 1��OD 60C2h sub2 = -4
    �岹����ʱ�� = 1 �� 10^-2 = 0.01s = 10ms
*/
UNS8 Write_SLAVE_Interpolation_time_period(UNS8 nodeId)
{
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS32 pdo_map_val,errorCode; 
  UNS8  dataType;
  UNS16 index = 0X1006;//ͨѶ����ֵ
  UNS8 subIndex = 0;//������0

  errorCode = readLocalDict(OD_Data,index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);//��ȡ0X1006ͨѶ����ֵ
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,SDO abort code is 0X%X",index,subIndex,errorCode);
    return 0xFF;
  }
  extern int Bit_Int_2(long n);
  Interpolation_time_period_Interpolation_time_index = (Bit_Int_2(pdo_map_val) - 1) - 6;//ʮ�Ĵη��� = 10^(n-6)
  Interpolation_time_period_Interpolation_time_units = pdo_map_val / pow(10,(Bit_Int_2(pdo_map_val) - 1));//�岹����ʱ�䳣��
  
  if(read_local_send_node_init_end(OD_Data,nodeId,0X60C2,0x01,0X60C2,0x01) == 0XFF)
    return 0XFF;
  if(read_local_send_node_init_end(OD_Data,nodeId,0X60C2,0x02,0X60C2,0x02) == 0XFF)
    return 0XFF;

  return 0x00;
}
/**
  * @brief  д��ӻ�ԭ�㸴��ģʽ���������
  * @param  nodeId:�ڵ�ID
  * @param  offest:ԭ��ƫ��ֵ ��λ:PUU [ע��:ֻ�ǰ�ԭ��ƫ��ֵ�㵱Ϊ0����㣬�������˶���0����㴦]
  * @param  method:��ԭ��ʽ   ��Χ:0 ~ 35
  * @param  switch_speed:Ѱ��ԭ�㿪���ٶ� �趨��Χ 0.1 ~ 200 Ĭ��ֵ 10  ��λrpm ����:С�����һλ
  * @param  zero_speed:Ѱ�� Z�����ٶ�     �趨��Χ 0.1 ~ 50  Ĭ��ֵ 2   ��λrpm ����:С�����һλ
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   Home PositionΪִ��ԭ�㸴��ʱ���ҵ���ԭ��ο��㣬��ԭ�� Sensor��Z����ȡ�
            ��ԭ��ο����ҵ��󣬴Ӹõ���ƫ�Ƶ�λ�ü�Ϊ�û������ԭ�㡣

            �� 35��ԭ�㸴��ģʽ���û��趨���� 1 ~ 16�ֻ�ԭ��ģʽ����Ѱ Z���壻 
            ���� 17 ~ 34�����ǲ���Ѱ Z���壻�� 35�����Ƕ��嵱ǰλ��Ϊԭ�㡣

            switch_speed��zero_speedд�뵥λΪ0.1rpm

            �趨ԭ�㸴���/����ʱ�� OD 609Ah Ĭ��ֵ 100 ��λ ms 
            �˶������趨��ʱ��б��Ϊ���� (0 rpm���ٵ� 3000 rpm) ������ (3000 rpm���ٵ�0 rpm) ����Ҫ��ʱ�䡣�˶���������� Homing Mode (ԭ�㸴��ģʽ)�� 
*/
UNS8 Write_SLAVE_Homing_set(UNS8 nodeId,UNS32 offset,UNS8 method,float switch_speed,float zero_speed)
{
  UNS8 errcode = 0;
  //����д�뱾���ֵ�
  Home_offset = offset;
  if(0 <= method && method <= 35)
    Homing_method = method;
  else
  {
    LOG_E("The method is out of range. Select a value from 0 to 35");
    return 0XFF;
  }
  if(0.1 <= switch_speed && switch_speed <= 200)
    Homing_speeds_Speed_for_switch_search = (uint32_t)(switch_speed * 10);
  else
  {
    LOG_E("The switch_speed is out of range. Select a value from 0.1 to 200");
    return 0XFF;
  }
  if(0.1 <= zero_speed && zero_speed <= 50)
    Homing_speeds_Speed_for_zero_search = (uint32_t)(zero_speed * 10);
  else
  {
    LOG_E("The zero_speed is out of range. Select a value from 0.1 to 50");
    return 0XFF;
  }

  //�������ͽڵ��ֵ�
  if(read_local_send_node_init_end(OD_Data,nodeId,0X607C,0x00,0X607C,0x00) == 0XFF)
    return 0XFF;
  if(read_local_send_node_init_end(OD_Data,nodeId,0X6098,0x00,0X6098,0x00) == 0XFF)
    return 0XFF;
  if(read_local_send_node_init_end(OD_Data,nodeId,0X6099,0x01,0X6099,0x01) == 0XFF)
    return 0XFF;
  if(read_local_send_node_init_end(OD_Data,nodeId,0X6099,0x02,0X6099,0x02) == 0XFF)
    return 0XFF;

  return 0X00;
}
/******************************NODE2 ���ò�������****************************************/
/******************************TPDO1 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO1���� , ֡ID0X183 DLC:8
  * �ֵ乤�ߣ�0x1400 Receive PDO 1 Parameter:COB ID used by PDO:0x00000183
  * �ֵ乤�ߣ�0x1600 Receive PDO 1 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��TPDO1,����������ͨ��RPDO1
  * PDO DATA:0x50 06 4E C3 00 00 00 00  С��ģʽ.
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 00 C3 4E = 49998 Position actual value (Ox6064)
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 00 C3 4E = 49998 Velocity_actual_value (Ox606C)
*/
static UNS8 NODE2_DIS_SLAVE_TPDO1(uint8_t nodeId)
{
// disable Slave's TPDO���رմӻ�TPDO1��������д�����
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000180,nodeId);//0x80000180 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_TPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE2_Clear_SLAVE_TPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//����ӻ�����������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A00, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_TPDO1_Sub1(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A00,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE2_Write_SLAVE_TPDO1_Sub2(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A00,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE2_Write_SLAVE_TPDO1_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A00,0,nodeId);//����д����ȷ������
}
static UNS8 NODE2_EN_SLAVE_TPDO1(uint8_t nodeId)
{
//enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000180,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
/******************************TPDO2 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO2���� , ֡ID0X283 DLC:4
  * �ֵ乤�ߣ�0x1401 Receive PDO 2 Parameter:COB ID used by PDO:0x00000283
  * �ֵ乤�ߣ�0x1601 Receive PDO 2 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��TPDO2,����������ͨ��RPDO2
  * PDO DATA:0x37 16   С��ģʽ.
  * SUBINDEX1:16λ,ռ��2���ֽ�.��0X1637 Statusword (Ox6041) 
*/
static UNS8 NODE2_DIS_SLAVE_TPDO2(uint8_t nodeId)
{
 //disable Slave's TPDO �رմӻ�TPDO2��������д�����
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000280,nodeId);//0x80000280 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_TPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE2_Clear_SLAVE_TPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//����ӻ�����������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A01, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_TPDO2_Sub1(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A01,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE2_Write_SLAVE_TPDO2_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A01,0,nodeId);//����д����ȷ������
}
static UNS8 NODE2_EN_SLAVE_TPDO2(uint8_t nodeId)
{
//enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000280,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
/******************************RPDO1 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO1���� , ֡ID0X203 DLC:8
  * �ֵ乤�ߣ�0x1800 Transmit PDO 1 Parameter:COB ID used by PDO:0x00000203
  * �ֵ乤�ߣ�0x1A00 Transmit PDO 1 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��RPDO1,����������ͨ��TPDO1
  * PDO DATA:0x00 00 01 00 00 00 00   С��ģʽ.
  * SUBINDEX1:32λ,ռ��4���ֽ�.��00 00 C3 50 = 50000  Target position (0x607A)
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 01 86 A0 = 100000 Target_velocity (0x60FF)
*/
static UNS8 NODE2_DIS_SLAVE_RPDO1(uint8_t nodeId)
{
// disable Slave's RPDO
  UNS32 RPDO_COBId = PDO_DISANBLE(0x80000200,nodeId);;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_RPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE2_Clear_SLAVE_RPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//�������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1600, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_RPDO1_Sub1(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1600,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE2_Write_SLAVE_RPDO1_Sub2(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1600,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE2_Write_SLAVE_RPDO1_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1600,0,nodeId);//����д����ȷ������
}
static UNS8 NODE2_EN_SLAVE_RPDO1(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 RPDO_COBId = PDO_ENANBLE(0x00000200,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_node_param_cb, 0);
}
/******************************RPDO2 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO2���� , ֡ID0X303 DLC:4
  * �ֵ乤�ߣ�0x1801 Transmit PDO 2 Parameter:COB ID used by PDO:0x00000303
  * �ֵ乤�ߣ�0x1A01 Transmit PDO 2 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��RPDO2,����������ͨ��TPDO2
  * PDO DATA:0x50 C3 00 00  С��ģʽ.
  * SUBINDEX1:32λ,ռ��4���ֽ�.��00 00 C3 50 = 50000  POS_CMD (0x60C1)
*/
static UNS8 NODE2_DIS_SLAVE_RPDO2(uint8_t nodeId)
{
// disable Slave's RPDO
  UNS32 RPDO_COBId = PDO_DISANBLE(0x80000300,nodeId);;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &RPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_RPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE2_Clear_SLAVE_RPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//�������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1601, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_RPDO2_Sub1(uint8_t nodeId)
{
  UNS32 pdo_map_val;
  UNS32 size = SDO_MAX_LENGTH_TRANSFER;
  UNS8  dataType;
  UNS32 errorCode;
  UNS32 SDO_data;

  errorCode = readLocalDict(OD_Data,0X2F00,0,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",0X2F00,0,errorCode);
    return 0XFF;
  }
  else
  {
    /*
          λ             ���� 
    Bit 0 ~ Bit 7     �������ݳ��� 
    Bit 8 ~ Bit 15    ���������� 
    Bit 16 ~ Bit 31   �������� 
    */
    SDO_data = (0X60C1 << 16) + (1 << 8) + (size * 8);
    pdo_map_val = 0;size = SDO_MAX_LENGTH_TRANSFER;dataType = 0;//�����������
    errorCode = readLocalDict(OD_Data,0X1A01,1,(void *)&pdo_map_val,&size,&dataType,0);
    if(errorCode != OD_SUCCESSFUL)
    {
      LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,abort code is 0X%X",0X1A01,1,errorCode);
      return 0XFF;
    }
    else
    {
      return writeNetworkDictCallBack(OD_Data,nodeId,0X1601,1,size,dataType,&SDO_data,config_node_param_cb,0);
    }
  }
}
static UNS8 NODE2_Write_SLAVE_RPDO2_Sub2(uint8_t nodeId)
{
  UNS32 pdo_map_val;
  UNS32 size = SDO_MAX_LENGTH_TRANSFER;
  UNS8  dataType;
  UNS32 errorCode;
  UNS32 SDO_data;

  errorCode = readLocalDict(OD_Data,0X2F01,0,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%04X,subIndex:0X%X,read Local Dict false,abort code is 0X%08X",0X2F01,0,errorCode);
    return 0XFF;
  }
  else
  {
    /*
          λ             ���� 
    Bit 0 ~ Bit 7     �������ݳ��� 
    Bit 8 ~ Bit 15    ���������� 
    Bit 16 ~ Bit 31   �������� 
    */
    SDO_data = (0X60C1 << 16) + (2 << 8) + (size * 8);
    pdo_map_val = 0;size = SDO_MAX_LENGTH_TRANSFER;dataType = 0;//�����������
    errorCode = readLocalDict(OD_Data,0X1A01,2,(void *)&pdo_map_val,&size,&dataType,0);
    if(errorCode != OD_SUCCESSFUL)
    {
      LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,abort code is 0X%X",0X1A01,2,errorCode);
      return 0XFF;
    }
    else
    {
      return writeNetworkDictCallBack(OD_Data,nodeId,0X1601,2,size,dataType,&SDO_data,config_node_param_cb,0);
    }
  }
}
static UNS8 NODE2_Write_SLAVE_RPDO2_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1601,0,nodeId);//����д����ȷ������
}
static UNS8 NODE2_EN_SLAVE_RPDO2(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000300,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
/******************************����ʱ�� ���ò�������*********************************/
static UNS8 NODE2_Write_SLAVE_P_heartbeat(uint8_t nodeId)
{
  UNS16 producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1017, 0, 
    2, uint16, &producer_heartbeat_time, config_node_param_cb, 0);
}
static UNS8 NODE2_Write_SLAVE_C_heartbeat(uint8_t nodeId)
{
  //��Ҫ�����ֵ�д������������ʱ�䣬��Ȼ����������
  UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(MASTER_NODEID,CONSUMER_HEARTBEAT_TIME);//д��ڵ�1������ʱ��
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1016, 1, 
    4, uint16, &consumer_heartbeat_time, config_node_param_cb, 0);
}
/*****************************������������*********************************/
/**
  * @brief  �����ӻ�S�ͼӼ��ٹ���
  * @param  None
  * @retval None
  * @note   None
*/
static UNS8 NODE2_Write_SLAVE_S_Move(uint8_t nodeId)
{
  extern subindex master402_Index2124;
  S_move = TRUE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x2124, 0, 
    master402_Index2124.size, master402_Index2124.bDataType, &S_move, config_node_param_cb, 0);
}
/*****************************�������ò�������*********************************/
static UNS8 NODE2_MotorCFG_Done(uint8_t nodeId)
{
	node_config_state *conf;
	conf = &slave_conf[nodeId - 2];
  rt_sem_release(&(conf->finish_sem));
  return 0;
}
/**
  * @brief  ����ָ������.
  * @param  None.
  * @retval None.
  * @note   
ע�⺯����˳�������Խ������
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static UNS8 (*NODECFG_Operation_2[])(uint8_t nodeId) = 
{ 
  //TPDO1ͨ������
  NODE2_DIS_SLAVE_TPDO1,
  NODE2_Write_SLAVE_TPDO1_Type,
  NODE2_Clear_SLAVE_TPDO1_Cnt,
  NODE2_Write_SLAVE_TPDO1_Sub1,
  NODE2_Write_SLAVE_TPDO1_Sub2,
  NODE2_Write_SLAVE_TPDO1_Sub0,
  NODE2_EN_SLAVE_TPDO1,
  //TPDO2ͨ������
  NODE2_DIS_SLAVE_TPDO2,
  NODE2_Write_SLAVE_TPDO2_Type,
  NODE2_Clear_SLAVE_TPDO2_Cnt,
  NODE2_Write_SLAVE_TPDO2_Sub1,
  NODE2_Write_SLAVE_TPDO2_Sub0,
  NODE2_EN_SLAVE_TPDO2,
  //RPDO1ͨ������
  NODE2_DIS_SLAVE_RPDO1,
  NODE2_Write_SLAVE_RPDO1_Type,
  NODE2_Clear_SLAVE_RPDO1_Cnt,
  NODE2_Write_SLAVE_RPDO1_Sub1,
  NODE2_Write_SLAVE_RPDO1_Sub2,
  NODE2_Write_SLAVE_RPDO1_Sub0,
  NODE2_EN_SLAVE_RPDO1,
  //RPDO2ͨ������
  NODE2_DIS_SLAVE_RPDO2,
  NODE2_Write_SLAVE_RPDO2_Type,
  NODE2_Clear_SLAVE_RPDO2_Cnt,
  NODE2_Write_SLAVE_RPDO2_Sub1,
  NODE2_Write_SLAVE_RPDO2_Sub2,
  NODE2_Write_SLAVE_RPDO2_Sub0,
  NODE2_EN_SLAVE_RPDO2,
  //д������
  NODE2_Write_SLAVE_P_heartbeat,
  NODE2_Write_SLAVE_C_heartbeat,
  //��������
  NODE2_Write_SLAVE_S_Move,
  //��������
  NODE2_MotorCFG_Done,
};
/******************************NODE3 ���ò�������****************************************/
/******************************TPDO1 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO1���� , ֡ID0X183 DLC:8
  * �ֵ乤�ߣ�0x1400 Receive PDO 1 Parameter:COB ID used by PDO:0x00000183
  * �ֵ乤�ߣ�0x1600 Receive PDO 1 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��TPDO1,����������ͨ��RPDO1
  * PDO DATA:0x50 06 4E C3 00 00 00 00  С��ģʽ.
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 00 C3 4E = 49998 Position actual value (Ox6064)
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 00 C3 4E = 49998 Velocity_actual_value (Ox606C)
*/
static UNS8 NODE3_DIS_SLAVE_TPDO1(uint8_t nodeId)
{
// disable Slave's TPDO���رմӻ�TPDO1��������д�����
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000180,nodeId);//0x80000180 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_TPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE3_Clear_SLAVE_TPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//����ӻ�����������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A00, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_TPDO1_Sub1(uint8_t nodeId)
{
  //����0X6040����д��ӻ�TPDOͨ����
  return read_local_send_node_pdo(0X6064,0,0x1A00,1,nodeId);
}
static UNS8 NODE3_Write_SLAVE_TPDO1_Sub2(uint8_t nodeId)
{
  return read_local_send_node_pdo(0X606C,0,0x1A00,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE3_Write_SLAVE_TPDO1_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A00,0,nodeId);//����д����ȷ������
}
static UNS8 NODE3_EN_SLAVE_TPDO1(uint8_t nodeId)
{
//enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000180,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
/******************************TPDO2 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO2���� , ֡ID0X283 DLC:4
  * �ֵ乤�ߣ�0x1401 Receive PDO 2 Parameter:COB ID used by PDO:0x00000283
  * �ֵ乤�ߣ�0x1601 Receive PDO 2 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��TPDO2,����������ͨ��RPDO2
  * PDO DATA:0x37 16   С��ģʽ.
  * SUBINDEX1:16λ,ռ��2���ֽ�.��0X1637 Statusword (Ox6041) 
*/
static UNS8 NODE3_DIS_SLAVE_TPDO2(uint8_t nodeId)
{
 //disable Slave's TPDO �رմӻ�TPDO2��������д�����
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000280,nodeId);//0x80000280 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_TPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE3_Clear_SLAVE_TPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//����ӻ�����������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A01, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_TPDO2_Sub1(uint8_t nodeId)
{
  return read_local_send_node_pdo(0x6041,0,0X1A01,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE3_Write_SLAVE_TPDO2_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1A01,0,nodeId);//����д����ȷ������
}
static UNS8 NODE3_EN_SLAVE_TPDO2(uint8_t nodeId)
{
//enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000280,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
/******************************RPDO1 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO1���� , ֡ID0X202 DLC:8
  * �ֵ乤�ߣ�0x1800 Transmit PDO 1 Parameter:COB ID used by PDO:0x00000202
  * �ֵ乤�ߣ�0x1A00 Transmit PDO 1 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��RPDO1,����������ͨ��TPDO1
  * PDO DATA:0x00 00 01 00 00 00 00   С��ģʽ.
  * SUBINDEX1:32λ,ռ��4���ֽ�.��00 00 C3 50 = 50000  Target position (0x607A)
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 01 86 A0 = 100000 Target_velocity (0x60FF)
*/
static UNS8 NODE3_DIS_SLAVE_RPDO1(uint8_t nodeId)
{
// disable Slave's RPDO
  UNS32 RPDO_COBId = PDO_DISANBLE(0x80000200,nodeId);;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_RPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE3_Clear_SLAVE_RPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//�������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1600, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_RPDO1_Sub1(uint8_t nodeId)
{
  return read_local_send_node_pdo(0x607A,0,0X1600,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE3_Write_SLAVE_RPDO1_Sub2(uint8_t nodeId)
{
  return read_local_send_node_pdo(0x60FF,0,0X1600,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE3_Write_SLAVE_RPDO1_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1600,0,nodeId);//����д����ȷ������
}
static UNS8 NODE3_EN_SLAVE_RPDO1(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 RPDO_COBId = PDO_ENANBLE(0x00000200,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_node_param_cb, 0);
}
/******************************RPDO2 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO2���� , ֡ID0X302 DLC:4
  * �ֵ乤�ߣ�0x1801 Transmit PDO 2 Parameter:COB ID used by PDO:0x00000302
  * �ֵ乤�ߣ�0x1A01 Transmit PDO 2 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��RPDO2,����������ͨ��TPDO2
  * PDO DATA:0x50 C3 00 00  С��ģʽ.
  * SUBINDEX1:32λ,ռ��4���ֽ�.��00 00 C3 50 = 50000  POS_CMD (0x60C1)
*/
static UNS8 NODE3_DIS_SLAVE_RPDO2(uint8_t nodeId)
{
// disable Slave's RPDO
  UNS32 RPDO_COBId = PDO_DISANBLE(0x80000300,nodeId);;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &RPDO_COBId, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_RPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 2, 
    1, uint8, &trans_type, config_node_param_cb, 0);
}
static UNS8 NODE3_Clear_SLAVE_RPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//�������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1601, 0, 
    1, uint8, &pdo_map_cnt, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_RPDO2_Sub1(uint8_t nodeId)
{
  return read_local_send_node_pdo(0x60C1,1,0X1601,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 NODE3_Write_SLAVE_RPDO2_Sub0(uint8_t nodeId)
{
  return read_local_send_node_pdo_T_R(0X1601,0,nodeId);//����д����ȷ������
}
static UNS8 NODE3_EN_SLAVE_RPDO2(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000300,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &TPDO_COBId, config_node_param_cb, 0);
}
/******************************����ʱ�� ���ò�������*********************************/
static UNS8 NODE3_Write_SLAVE_P_heartbeat(uint8_t nodeId)
{
  UNS16 producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1017, 0, 
    2, uint16, &producer_heartbeat_time, config_node_param_cb, 0);
}
static UNS8 NODE3_Write_SLAVE_C_heartbeat(uint8_t nodeId)
{
  //��Ҫ�����ֵ�д������������ʱ�䣬��Ȼ����������
  UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(MASTER_NODEID,CONSUMER_HEARTBEAT_TIME);//д��ڵ�1������ʱ��
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1016, 1, 
    4, uint16, &consumer_heartbeat_time, config_node_param_cb, 0);
}
/*****************************������������*********************************/
/**
  * @brief  �����ӻ�S�ͼӼ��ٹ���
  * @param  None
  * @retval None
  * @note   None
*/
static UNS8 NODE3_Write_SLAVE_S_Move(uint8_t nodeId)
{
  extern subindex master402_Index2124;
  S_move = TRUE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x2124, 0, 
    master402_Index2124.size, master402_Index2124.bDataType, &S_move, config_node_param_cb, 0);
}
/*****************************�������ò�������*********************************/
static UNS8 NODE3_MotorCFG_Done(uint8_t nodeId)
{
	node_config_state *conf;
	conf = &slave_conf[nodeId - 2];
  rt_sem_release(&(conf->finish_sem));
  return 0;
}
/**
  * @brief  ����ָ������.
  * @param  None.
  * @retval None.
  * @note   
ע�⺯����˳�������Խ������
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static UNS8 (*NODECFG_Operation_3[])(uint8_t nodeId) = 
{ 
  //TPDO1ͨ������
  NODE3_DIS_SLAVE_TPDO1,
  NODE3_Write_SLAVE_TPDO1_Type,
  NODE3_Clear_SLAVE_TPDO1_Cnt,
  NODE3_Write_SLAVE_TPDO1_Sub1,
  NODE3_Write_SLAVE_TPDO1_Sub2,
  NODE3_Write_SLAVE_TPDO1_Sub0,
  NODE3_EN_SLAVE_TPDO1,
  //TPDO2ͨ������
  NODE3_DIS_SLAVE_TPDO2,
  NODE3_Write_SLAVE_TPDO2_Type,
  NODE3_Clear_SLAVE_TPDO2_Cnt,
  NODE3_Write_SLAVE_TPDO2_Sub1,
  NODE3_Write_SLAVE_TPDO2_Sub0,
  NODE3_EN_SLAVE_TPDO2,
  //RPDO1ͨ������
  NODE3_DIS_SLAVE_RPDO1,
  NODE3_Write_SLAVE_RPDO1_Type,
  NODE3_Clear_SLAVE_RPDO1_Cnt,
  NODE3_Write_SLAVE_RPDO1_Sub1,
  NODE3_Write_SLAVE_RPDO1_Sub2,
  NODE3_Write_SLAVE_RPDO1_Sub0,
  NODE3_EN_SLAVE_RPDO1,
  //RPDO2ͨ������
  NODE3_DIS_SLAVE_RPDO2,
  NODE3_Write_SLAVE_RPDO2_Type,
  NODE3_Clear_SLAVE_RPDO2_Cnt,
  NODE3_Write_SLAVE_RPDO2_Sub1,
  NODE3_Write_SLAVE_RPDO2_Sub0,
  NODE3_EN_SLAVE_RPDO2,
  //д������
  NODE3_Write_SLAVE_P_heartbeat,
  NODE3_Write_SLAVE_C_heartbeat,
  //��������
  NODE3_Write_SLAVE_S_Move,
  //��������
  NODE3_MotorCFG_Done,
};
/**
  * @brief  ���ò���
  * @param  None
  * @retval None
  * @note   None
*/
static void config_node_param(uint8_t nodeId, node_config_state *conf)
{
  if(nodeId == SERVO_NODEID_1)
  {
    //�ж�����Խ�� ָ���޷��ж�ָ�����ݴ�С https://bbs.csdn.net/topics/80323809
    if(conf->state < sizeof(NODECFG_Operation_2) / sizeof(NODECFG_Operation_2[0]))
    {
      conf->err_code = NODECFG_Operation_2[conf->state](nodeId);
    }
  }
  else if(nodeId == SERVO_NODEID_2)
  {
    //�ж�����Խ�� ָ���޷��ж�ָ�����ݴ�С https://bbs.csdn.net/topics/80323809
    if(conf->state < sizeof(NODECFG_Operation_3) / sizeof(NODECFG_Operation_3[0]))
    {
      conf->err_code = NODECFG_Operation_3[conf->state](nodeId);
    }
  }
  else
  {
    LOG_E("Configuration node %d does not exist",nodeId);
  }
  if(conf->err_code != 0)//���ò�������
  {
    rt_sem_release(&(conf->finish_sem));
    LOG_W("Step %d configuration parameters are incorrect",conf->state + 1);
    conf->state = 0;
  }
  else
  {
    conf->err_code = NODEID_CONFIG_SUCCESS;
  }
}