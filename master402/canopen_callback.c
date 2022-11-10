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
  * 异常处理
  * 
  * CAN线短路，阻塞当前线程，等待短路恢复
  *
  * MCU 初始化时:CAN总线断开[已处理] 单节点掉线[已处理] 多节点掉线[已处理] 单节点掉电[暂时无法测试] 多节点掉电[已处理]
  * MCU 操作态时:CAN总线断开[已处理] 单节点掉线[已处理] 多节点掉线[已处理] 单节点掉电[暂时无法测试] 多节点掉电[已处理]
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
/*修复程序结构体
*/
typedef struct
{
  UNS8 lock;    //锁定线程，确保只有一个线程创建
  UNS8 try_cnt; //修复尝试次数
}Fix_Typedef;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static Fix_Typedef node[MAX_NODE_COUNT - 2];
static Fix_Typedef cfg[MAX_NODE_COUNT - 2];
/* Private function prototypes -----------------------------------------------*/
static void master402_fix_node_Disconnected(void* parameter);
/**
  * @brief  主站节点心跳异常回调
  * @param  None.
  * @retval None.
  * @note   产生原因：1.从机掉线。从机上线后可通过emcy回调解决
  * 				 2.主机从机间can线异常。查询从机节点状态从Disconnected切换后，证明从机重新上线。
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
                              (void *)(int)heartbeatID,//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
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
  * @brief  主站节点初始化回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_initialisation(CO_Data* d)
{
	LOG_I("canfestival enter initialisation state");
}
/**
  * @brief  主站节点预操作回调.
  * @param  None.
  * @retval None.
  * @note   开启从站参数配置操作
*/
void master402_preOperational(CO_Data* d)
{
	rt_thread_t tid;
	LOG_I("canfestival enter preOperational state");
	tid = rt_thread_create("co_cfg", canopen_start_thread_entry, (void *)(int)d,//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
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
  * @brief  主站节点进入操作模式回调.
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
  * @brief  主站节点进入停止回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_stopped(CO_Data* d)
{
	LOG_E("canfestival enter stop state");
}
/**
  * @brief  主站节点发送sync回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_post_sync(CO_Data* d)
{
//	rt_kprintf(" SYNC received. Proceed.\n");
}
/**
  * @brief  主站节点初发送TPDO回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_post_TPDO(CO_Data* d)
{
//    rt_kprintf("Send TPDO \n");
}
/**
  * @brief  主站节点储存回调.
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
  * @brief  主站节点emcy紧急事件回调.
  * @param  None.
  * @retval None.
  * @note   None.
errReg:如果某一特定的错误发生相应位置 1b。通用错误位是强制设备必须支持的，其它可选。任何错误产生都将置位通用错误。
位 M/O     意义
0   M     通用错误
1   O     电流
2   O     电压
3   O     温度
4   O     通信错误(溢出、错误状态)
5   O     设备协议指定
6   O     保留(始终为 0b)
7   O     制造商指定

ErrorCode: 6200  用户软件 -常规错误[]
ErrorCode: 3130  主回路电源异常[驱动器掉电]
ErrorCode: 8200  协议错误–常规 AL121 当上位机发送 PDO 给驱动器时，PDO 所指定的对象字典 Index 号码不正确，导致驱动器无法辨识。
                              AL124 PDO所欲存取的对象字典范围错误 讯息中的数据超出指定对象字典的范围[例如写入速度4000超出0~3000范围]
ErrorCode: 8110  CAN overflow (object loss) AL113 TxPDO 传送失败
*/
void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
  if(errCode != 0)//应急错误代码
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
  * @brief  主站恢复操作状态.
  * @param  None.
  * @retval None.
  * @note   重新开始生产者心跳
*/
void master_resume_start(CO_Data *d,UNS8 nodeId)
{
  if(getState(d) == Stopped)//can通信异常，发送失败多次，进入停止状态
  {
    //Stop状态时会删除生产者心跳定时器
    if (*d->ProducerHeartBeatTime)//恢复到OP状态时检查是否有生产者心跳时间
    {
      TIMEVAL time = *d->ProducerHeartBeatTime;
      extern void ProducerHeartbeatAlarm(CO_Data* d, UNS32 id);
      //设置生产者时间定时器，并设置定时回调
      LOG_W("Restart the producer heartbeat");
      d->ProducerHeartBeatTimer = SetAlarm(d, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
    }
    LOG_W("The master station enters the operation state from the stop state");
    setState(d, Operational);//转入Operational状态
  }
  masterSendNMTstateChange(d,nodeId,NMT_Start_Node);
}

/**
  * @brief  修复节点NMT异常
  * @param  None.
  * @retval None.
  * @note   查询从机节点状态变为pre后，证明从机重新上线。
            可以进行NMT命令重新恢复通信
*/
static void master402_fix_node_Disconnected(void* parameter)
{
	static e_nodeState last;
	e_nodeState now;

	int heartbeatID = (int)parameter;//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
	LOG_E("heartbeatID abnormal:0x%x  ", heartbeatID);
  node[heartbeatID - 2].lock = 0xff;
	while (1)
	{
		last = now;
		now = getNodeState(OD_Data,heartbeatID);
		if(now != last)//节点状态发生变化，证明从机上线
		{
			if(now == Operational)//由0x8130错误码处理程序处理
			{
        LOG_I("nodeID:%d,Handled by the 0x8130 error code handler,def ThreadFinished",heartbeatID);
        node[heartbeatID - 2].lock = 0;
				return;//删除线程
			}
			else if(now == Pre_operational)
			{
        master_resume_start(OD_Data,heartbeatID);
				LOG_I("nodeID:%d,Determines that the line is restored and switches the slave machine to operation mode, deleting the current thread",heartbeatID);
        node[heartbeatID - 2].lock = 0;
				return;//退出线程
			}
      else if(now == Initialisation)
      {
//        setState(OD_Data, Initialisation);//Initialisation
        LOG_I("nodeID:%d,After the heartbeat of the node is abnormal, the node is shut down and powered on",heartbeatID);
        node[heartbeatID - 2].lock = 0;
				return;//退出线程
      }
		}

		rt_thread_mdelay(CONSUMER_HEARTBEAT_TIME);//按照获取心跳间隔时间进行判断
	}
}
/**
  * @brief  修复配置错误线程
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void master402_fix_config_err_thread_entry(void* parameter)
{
	static e_nodeState last;
	e_nodeState now;
  int nodeId = (int)parameter;//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
	LOG_E("nodeId abnormal:0x%x  ", nodeId);
  while (1)
  {
    now = getNodeState(OD_Data,nodeId);
    if(now == Unknown_state)//没有进行通信
    {
      masterRequestNodeState(OD_Data,nodeId);//发送Node Guarding Request命令查询节点状态
    }
    else if(now == Disconnected)//断开连接
    {
      masterRequestNodeState(OD_Data,nodeId);//发送Node Guarding Request命令查询节点状态
    }
    else if(now == Pre_operational)//通信恢复
    {
      if(getState(OD_Data) != Operational || getState(OD_Data) != Pre_operational)
      {
        //Stop状态时会删除生产者心跳定时器
        if (*OD_Data->ProducerHeartBeatTime)//恢复到OP状态时检查是否有生产者心跳时间
        {
          TIMEVAL time = *OD_Data->ProducerHeartBeatTime;
          extern void ProducerHeartbeatAlarm(CO_Data* d, UNS32 id);
          //设置生产者时间定时器，并设置定时回调
          LOG_W("Restart the producer heartbeat");
          OD_Data->ProducerHeartBeatTimer = SetAlarm(OD_Data, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
        }
        LOG_W("The master station enters the operation state from the stop state");
        setState(OD_Data, Operational);//转入Operational状态
      }
      config_node(nodeId);
      LOG_I("nodeID:%d,The line comm.unication of the node is restored",nodeId);
      return; //删除线程
    }
    else if(now == Initialisation)//节点断电后上电
    {
      LOG_I("nodeID:%d,Restart the node after the node is shut down",nodeId);
      return; //删除线程
    }
    else
    {
      masterRequestNodeState(OD_Data,nodeId);//发送Node Guarding Request命令查询节点状态
    }
      
    rt_thread_mdelay(CONSUMER_HEARTBEAT_TIME);//按照获取心跳间隔时间进行判断
  }  
}
/**
  * @brief  修复配置错误
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
                      (void *)(int)nodeId,//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
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