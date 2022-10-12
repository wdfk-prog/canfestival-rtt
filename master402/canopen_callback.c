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
  * MCU ��ʼ��ʱ:CAN���߶Ͽ�[�Ѵ���] �ڵ����[�Ѵ���]
  * MCU Ԥ����ʱ:CAN���߶Ͽ�[�Ѵ���] �ڵ����[�Ѵ���]
  * MCU ����̬ʱ:CAN���߶Ͽ�[�Ѵ���] �ڵ����[�Ѵ���]
  * @author
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "canopen_callback.h"
/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>

#include "canfestival.h"

#include "master402_od.h"
#include "master402_canopen.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static rt_uint8_t fix_node_lock= 0;//�����̣߳�ȷ��ֻ��һ���̴߳���
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
  LOG_E("heartbeatError!heartbeatID:0x%x", heartbeatID);
  if(fix_node_lock == 0)
  {
    rt_thread_t tid;
    tid = rt_thread_create("fix_nodeID", master402_fix_node_Disconnected,
                            (void *)(int)heartbeatID,//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
                            2048, 12, 2);

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
    LOG_W("Unable to create a new thread because an existing thread is running");
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
	tid = rt_thread_create("co_cfg", canopen_start_thread_entry, RT_NULL, 2048, 12, 2);
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
*/
void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
  if(errCode != 0)//Ӧ���������
  {
    LOG_E("received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x", nodeID, errCode, errReg);
    if(errCode == 0x8130)//�ڵ㱣�����������������
    {
      /*�����˴������ԭ�򣺴ӻ����߻���can�ź����쳣������
        �ܹ����յ��˴�����룬֤��ͨ�Żָ��������˴������ͨ������NMT�����
        �ӻ�����pre״̬�����ɽ��в���
        �л��ӻ�����start״̬*/
      LOG_W("ErrorCode: %4.4x :Node protection error or heartbeat error",errCode);
      masterSendNMTstateChange(d,nodeID,NMT_Start_Node);
      LOG_W("Determine the line recovery and switch the slave machine into operation mode");
    }
  }

}
/*******************************ERROR FIX CODE*****************************************************************/
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
	e_nodeState now,master;

	int heartbeatID = (int)parameter;//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
	LOG_E("heartbeatID abnormal:0x%x  ", heartbeatID);
  fix_node_lock = 0xff;
	while (1)
	{
		last = now;
		now = getNodeState(OD_Data,heartbeatID);
		if(now != last)//�ڵ�״̬�����仯��֤���ӻ�����
		{
			if(now == Operational)//��0x8130�����봦�������
			{
        LOG_I("Handled by the 0x8130 error code handler,def ThreadFinished");
        fix_node_lock = 0;
				return;//ɾ���߳�
			}
			else if(now == Pre_operational)
			{
        master = getState(OD_Data);//��ȡ��վ�ڵ�״̬
        if(master == Stopped)//canͨ���쳣������ʧ�ܶ�Σ�����ֹͣ״̬
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
				masterSendNMTstateChange(OD_Data,heartbeatID,NMT_Start_Node);
				LOG_W("Determine the line recovery and switch the slave machine into operation mode,def ThreadFinished");
        fix_node_lock = 0;
				return;//�˳��߳�
			}
      else if(now == Initialisation)
      {
        setState(OD_Data, Initialisation);//Initialisation
        LOG_I("After the heartbeat of the node is abnormal, the node is shut down and powered on");
        fix_node_lock = 0;
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
      setState(OD_Data, Initialisation);//Initialisation ->�Զ�ת��pre״̬ ->ת��op
      LOG_I("The line communication of the node is restored");
      return; //ɾ���߳�
    }
    else if(now == Initialisation)//�ڵ�ϵ���ϵ�
    {
      setState(OD_Data, Initialisation);//Initialisation
      LOG_I("Restart the node after the node is shut down");
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
void master402_fix_config_err(UNS8 nodeId)
{
  setState(OD_Data, Stopped);
  LOG_I("Enabling the repair thread");
  rt_thread_t tid = rt_thread_create("fix_config_err", master402_fix_config_err_thread_entry,
                    (void *)(int)nodeId,//ǿ��ת��Ϊ16λ������void*ָ���ֽ�һ�£�������ǿ��ת����С��ƥ�侯��
                    2048, 12, 2);

  if(tid == RT_NULL)
  {
    LOG_E("canfestival fix_config_err thread start failed!");
  }
  else
  {
    rt_thread_startup(tid);
  }
}