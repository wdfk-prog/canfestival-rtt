/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  * �쳣����
  * 
  * CAN�߶�·��������ǰ�̣߳��ȴ���·�ָ�
  *
  * MCU ��ʼ��ʱ:CAN���߶Ͽ�[�Ѵ���] ���ڵ����[�Ѵ���] ��ڵ����[�Ѵ���] ���ڵ����[��ʱ�޷�����] ��ڵ����[�Ѵ���]
  * MCU ����̬ʱ:CAN���߶Ͽ�[�Ѵ���] ���ڵ����[�Ѵ���] ��ڵ����[�Ѵ���] ���ڵ����[��ʱ�޷�����] ��ڵ����[�Ѵ���]
  * @author
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "canopen_callback.h"
/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>
#include <stdlib.h>
#include "canfestival.h"

#include "master402_od.h"
#include "master402_canopen.h"
/* Private typedef -----------------------------------------------------------*/
/*�޸�����ṹ��
*/
typedef struct
{
  UNS8 lock;    //�����̣߳�ȷ��ֻ��һ���̴߳���
  UNS8 try_cnt; //�޸����Դ���
}Fix_Typedef;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static Fix_Typedef node[MAX_NODE_COUNT - 2];
static Fix_Typedef cfg[MAX_NODE_COUNT - 2];
/* Private function prototypes -----------------------------------------------*/
static void master402_fix_node_Disconnected(void* parameter);
/**
  * @brief  ��վ�ڵ������쳣�ص�
  * @param  None.
  * @retval None.
  * @note   ����ԭ��1.�ӻ����ߡ��ӻ����ߺ��ͨ��emcy�ص����
  * 				 2.�����ӻ���can���쳣����ѯ�ӻ��ڵ�״̬��Disconnected�л���֤���ӻ��������ߡ�
*/
void master402_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
  if(++node[heartbeatID - 2].try_cnt <= 5)
  {
    LOG_E("heartbeatError!heartbeatID:0x%x,try cnt = %d", heartbeatID,node[heartbeatID - 2].try_cnt);
    if(node[heartbeatID - 2].lock == 0)
    {
      rt_thread_t tid;
      char name[RT_NAME_MAX + 1];
      rt_sprintf(name,"NODEerr%d",heartbeatID);
      tid = rt_thread_create(name, master402_fix_node_Disconnected,
                              (void *)(int)heartbeatID,//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
                              1024, 12, 2);

      if(tid == RT_NULL)
      {
        LOG_E("canfestival get nodeid state thread start failed!");
      }
      else
      {
        rt_thread_startup(tid);
      }
    }
    else
    {
      LOG_W("nodeID :%d,Unable to create a new thread because an existing thread is running",heartbeatID);
    }
  }
  else if(node[heartbeatID - 2].try_cnt > 5)
  {
     LOG_E("NodeID:%d,The number %d of repairs is too many. It is confirmed that it is not an occasional anomaly. It will not be repaired any more.",heartbeatID,node[heartbeatID - 2].try_cnt);
  }
}
/**
  * @brief  ��վ�ڵ��ʼ���ص�.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_initialisation(CO_Data* d)
{
	LOG_I("canfestival enter initialisation state");
}
/**
  * @brief  ��վ�ڵ�Ԥ�����ص�.
  * @param  None.
  * @retval None.
  * @note   ������վ�������ò���
*/
void master402_preOperational(CO_Data* d)
{
	rt_thread_t tid;
	LOG_I("canfestival enter preOperational state");
	tid = rt_thread_create("co_cfg", canopen_start_thread_entry, (void *)(int)d,//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
                        2048, 12, 2);
	if(tid == RT_NULL)
	{
		LOG_E("canfestival config thread start failed!");
	}
	else
	{
		rt_thread_startup(tid);
	}
}
/**
  * @brief  ��վ�ڵ�������ģʽ�ص�.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_operational(CO_Data* d)
{
  extern void canSend_Clear_errcnt(void);
  canSend_Clear_errcnt();
	LOG_I("canfestival enter operational state");
}
/**
  * @brief  ��վ�ڵ����ֹͣ�ص�.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_stopped(CO_Data* d)
{
	LOG_E("canfestival enter stop state");
}
/**
  * @brief  ��վ�ڵ㷢��sync�ص�.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_post_sync(CO_Data* d)
{
//	rt_kprintf(" SYNC received. Proceed.\n");
}
/**
  * @brief  ��վ�ڵ������TPDO�ص�.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_post_TPDO(CO_Data* d)
{
//    rt_kprintf("Send TPDO \n");
}
/**
  * @brief  ��վ�ڵ㴢��ص�.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex)
{
	/*TODO : 
	 * - call getODEntry for index and subindex, 
	 * - save content to file, database, flash, nvram, ...
	 * 
	 * To ease flash organisation, index of variable to store
	 * can be established by scanning d->objdict[d->ObjdictSize]
	 * for variables to store.
	 * 
	 * */
	LOG_I("storeODSubIndex : %4.4x %2.2x", wIndex,  bSubindex);
}
/**
  * @brief  ��վ�ڵ�emcy�����¼��ص�.
  * @param  None.
  * @retval None.
  * @note   None.
errReg:���ĳһ�ض��Ĵ�������Ӧλ�� 1b��ͨ�ô���λ��ǿ���豸����֧�ֵģ�������ѡ���κδ������������λͨ�ô���
λ M/O     ����
0   M     ͨ�ô���
1   O     ����
2   O     ��ѹ
3   O     �¶�
4   O     ͨ�Ŵ���(���������״̬)
5   O     �豸Э��ָ��
6   O     ����(ʼ��Ϊ 0b)
7   O     ������ָ��

ErrorCode: 6200  �û���� -�������[]
ErrorCode: 3130  ����·��Դ�쳣[����������]
ErrorCode: 8200  Э�����C���� AL121 ����λ������ PDO ��������ʱ��PDO ��ָ���Ķ����ֵ� Index ���벻��ȷ�������������޷���ʶ��
                              AL124 PDO������ȡ�Ķ����ֵ䷶Χ���� ѶϢ�е����ݳ���ָ�������ֵ�ķ�Χ[����д���ٶ�4000����0~3000��Χ]
ErrorCode: 8110  CAN overflow (object loss) AL113 TxPDO ����ʧ��
*/
void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
  if(errCode != 0)//Ӧ���������
  {
      if(d->nodeState == Pre_operational && d->NMTable[nodeID] == Unknown_state)
      {
        LOG_D("nodeID:%d,The last node error is not cleared. Do not worry",nodeID);
      }
      else
      {
        LOG_E("received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x", nodeID, errCode, errReg);
        if(errSpec[0]+errSpec[1]+errSpec[2]+errSpec[3]+errSpec[4])
        {
          LOG_E("Manufacturer Specific: 0X%X %2.2X %2.2X %2.2X %2.2X",errSpec[0],errSpec[1],errSpec[2],errSpec[3],errSpec[4]);
        }
      }
  }
}
/*******************************ERROR FIX CODE*****************************************************************/
/**
  * @brief  ��վ�ָ�����״̬.
  * @param  None.
  * @retval None.
  * @note   ���¿�ʼ����������
*/
void master_resume_start(CO_Data *d,UNS8 nodeId)
{
  if(getState(d) == Stopped)//canͨ���쳣������ʧ�ܶ�Σ�����ֹͣ״̬
  {
    //Stop״̬ʱ��ɾ��������������ʱ��
    if (*d->ProducerHeartBeatTime)//�ָ���OP״̬ʱ����Ƿ�������������ʱ��
    {
      TIMEVAL time = *d->ProducerHeartBeatTime;
      extern void ProducerHeartbeatAlarm(CO_Data* d, UNS32 id);
      //����������ʱ�䶨ʱ���������ö�ʱ�ص�
      LOG_W("Restart the producer heartbeat");
      d->ProducerHeartBeatTimer = SetAlarm(d, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
    }
    LOG_W("The master station enters the operation state from the stop state");
    setState(d, Operational);//ת��Operational״̬
  }
  masterSendNMTstateChange(d,nodeId,NMT_Start_Node);
}

/**
  * @brief  �޸��ڵ�NMT�쳣
  * @param  None.
  * @retval None.
  * @note   ��ѯ�ӻ��ڵ�״̬��Ϊpre��֤���ӻ��������ߡ�
            ���Խ���NMT�������»ָ�ͨ��
*/
static void master402_fix_node_Disconnected(void* parameter)
{
	static e_nodeState last;
	e_nodeState now;

	int heartbeatID = (int)parameter;//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
	LOG_E("heartbeatID abnormal:0x%x  ", heartbeatID);
  node[heartbeatID - 2].lock = 0xff;
	while (1)
	{
		last = now;
		now = getNodeState(OD_Data,heartbeatID);
		if(now != last)//�ڵ�״̬�����仯��֤���ӻ�����
		{
			if(now == Operational)//��0x8130�����봦�������
			{
        LOG_I("nodeID:%d,Handled by the 0x8130 error code handler,def ThreadFinished",heartbeatID);
        node[heartbeatID - 2].lock = 0;
				return;//ɾ���߳�
			}
			else if(now == Pre_operational)
			{
        master_resume_start(OD_Data,heartbeatID);
				LOG_I("nodeID:%d,Determines that the line is restored and switches the slave machine to operation mode, deleting the current thread",heartbeatID);
        node[heartbeatID - 2].lock = 0;
				return;//�˳��߳�
			}
      else if(now == Initialisation)
      {
//        setState(OD_Data, Initialisation);//Initialisation
        LOG_I("nodeID:%d,After the heartbeat of the node is abnormal, the node is shut down and powered on",heartbeatID);
        node[heartbeatID - 2].lock = 0;
				return;//�˳��߳�
      }
		}

		rt_thread_mdelay(CONSUMER_HEARTBEAT_TIME);//���ջ�ȡ�������ʱ������ж�
	}
}
/**
  * @brief  �޸����ô����߳�
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void master402_fix_config_err_thread_entry(void* parameter)
{
	static e_nodeState last;
	e_nodeState now;
  int nodeId = (int)parameter;//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
	LOG_E("nodeId abnormal:0x%x  ", nodeId);
  while (1)
  {
    now = getNodeState(OD_Data,nodeId);
    if(now == Unknown_state)//û�н���ͨ��
    {
      masterRequestNodeState(OD_Data,nodeId);//����Node Guarding Request�����ѯ�ڵ�״̬
    }
    else if(now == Disconnected)//�Ͽ�����
    {
      masterRequestNodeState(OD_Data,nodeId);//����Node Guarding Request�����ѯ�ڵ�״̬
    }
    else if(now == Pre_operational)//ͨ�Żָ�
    {
      if(getState(OD_Data) != Operational || getState(OD_Data) != Pre_operational)
      {
        //Stop״̬ʱ��ɾ��������������ʱ��
        if (*OD_Data->ProducerHeartBeatTime)//�ָ���OP״̬ʱ����Ƿ�������������ʱ��
        {
          TIMEVAL time = *OD_Data->ProducerHeartBeatTime;
          extern void ProducerHeartbeatAlarm(CO_Data* d, UNS32 id);
          //����������ʱ�䶨ʱ���������ö�ʱ�ص�
          LOG_W("Restart the producer heartbeat");
          OD_Data->ProducerHeartBeatTimer = SetAlarm(OD_Data, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
        }
        LOG_W("The master station enters the operation state from the stop state");
        setState(OD_Data, Operational);//ת��Operational״̬
      }
      config_node(nodeId);
      LOG_I("nodeID:%d,The line comm.unication of the node is restored",nodeId);
      return; //ɾ���߳�
    }
    else if(now == Initialisation)//�ڵ�ϵ���ϵ�
    {
      LOG_I("nodeID:%d,Restart the node after the node is shut down",nodeId);
      return; //ɾ���߳�
    }
    else
    {
      masterRequestNodeState(OD_Data,nodeId);//����Node Guarding Request�����ѯ�ڵ�״̬
    }
      
    rt_thread_mdelay(CONSUMER_HEARTBEAT_TIME);//���ջ�ȡ�������ʱ������ж�
  }  
}
/**
  * @brief  �޸����ô���
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_fix_config_err(CO_Data *d,UNS8 nodeId)
{
  char name[RT_NAME_MAX];
  rt_sprintf(name,"%s%c","cf_err",'0'+nodeId);
  if(++cfg[nodeId - 2].try_cnt <= 3)
  {
    LOG_I("nodeID:%d,Enabling the repair thread,Repair times = %d",nodeId,cfg[nodeId - 2].try_cnt);
    rt_thread_t tid = rt_thread_create(name, master402_fix_config_err_thread_entry,
                      (void *)(int)nodeId,//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
                      1024, 12, 2);

    if(tid == RT_NULL)
    {
      LOG_E("uanodeID:%d,canfestival fix_config_err thread start failed!",nodeId);
    }
    else
    {
      rt_thread_startup(tid);
    }
  }
  else if(cfg[nodeId - 2].try_cnt == 4)
  {
     LOG_E("nodeID:%d,The number of repairs is too many. It is confirmed that it is not an occasional anomaly. It will not be repaired any more.",nodeId);
  }
}