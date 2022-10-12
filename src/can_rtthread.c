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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "canfestival.h"
#include "timers_driver.h"
#include "master402_canopen.h"
//#define DBG_TAG "app.CANopen"
//#define DBG_LVL DBG_LOG
//#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
struct can_app_struct
{
    const char *name;
    struct rt_semaphore sem;
};
static struct can_app_struct can_data =
{
    CANFESTIVAL_CAN_DEVICE_NAME
};
/* Private define ------------------------------------------------------------*/
#define MAX_MUTEX_WAIT_TIME 5000  //互斥量等待时间
#define MAX_SEM_WAIT_TIME   5000  //信号量等待时间，can接收超时时间
/* 线程配置 */
#define THREAD_PRIORITY      CANFESTIVAL_RECV_THREAD_PRIO//线程优先级
#define THREAD_TIMESLICE     20//线程时间片
#define THREAD_STACK_SIZE    2048//栈大小
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static rt_device_t candev = RT_NULL;
static CO_Data * od_data = RT_NULL;
static rt_mutex_t canfstvl_mutex = RT_NULL;
static int canSend_err_cnt = 0;//can发送错误计数
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  锁定
  * @param  None
  * @retval None
  * @note   None
*/
void EnterMutex(void)
{
	if(rt_mutex_take(canfstvl_mutex, MAX_MUTEX_WAIT_TIME) != RT_EOK) {
		LOG_E("canfestival take mutex failed!");
	}
}
/**
  * @brief  解锁
  * @param  None
  * @retval None
  * @note   None
*/
void LeaveMutex(void)
{
	if(rt_mutex_release(canfstvl_mutex) != RT_EOK) {
		LOG_E("canfestival release mutex failed!");
	}
}
/**
  * @brief  接收回调
  * @param  None
  * @retval None
  * @note   释放信号量，运行接收线程
*/
static rt_err_t  can1ind(rt_device_t dev,  rt_size_t size)
{
    rt_err_t err = rt_sem_release(&can_data.sem);
    if(err != RT_EOK) {
		LOG_E("canfestival release receive semaphore failed!");
    }
    return err;
}
/**
  * @brief  清除can发送错误计数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void canSend_Clear_errcnt(void)
{
  canSend_err_cnt = 0;
}
/**
  * @brief  can发送函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
unsigned char canSend(CAN_PORT notused, Message *m)
{
	struct rt_can_msg msg;

	msg.id = m->cob_id;
	msg.ide = 0;
	msg.rtr = m->rtr;
	msg.len = m->len;
	memcpy(msg.data, m->data, m->len);
  RT_ASSERT(candev != RT_NULL);

	rt_size_t write_size = rt_device_write(candev, 0, &msg, sizeof(msg));
	if(write_size != sizeof(msg))
  {
    if((!(canSend_err_cnt % (5000 / PRODUCER_HEARTBEAT_TIME))) && canSend_err_cnt != 0)
    {
      LOG_W("canfestival send failed, err = %d,cnt = %d", rt_get_errno(),canSend_err_cnt);
    }
		if(++canSend_err_cnt >= (5*5000 / PRODUCER_HEARTBEAT_TIME)) 
    {
		    setState(od_data, Stopped);
		}
	  return 0xFF;
	}
	canSend_err_cnt = 0;
  return 0;
}
/**
  * @brief  can接收处理线程.
  * @param  None.
  * @retval None.
  * @note   5s超时未接收信号报警.
*/
void canopen_recv_thread_entry(void* parameter)
{
  struct can_app_struct *canpara = (struct can_app_struct *) parameter;
	struct rt_can_msg msg;
	Message co_msg;

  candev = rt_device_find(canpara->name);
  RT_ASSERT(candev);
  rt_sem_init(&can_data.sem, "co-rx", 0, RT_IPC_FLAG_PRIO);
  rt_err_t err = rt_device_open(candev, (RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX));
  if( err != RT_EOK) 
  {
      LOG_E("canfestival open device %s failed, err = %d", canpara->name, err);
      return;
  }
  err = rt_device_set_rx_indicate(candev, can1ind);
  if( err != RT_EOK) 
  {
      LOG_E("canfestival set rx indicate failed, err = %d", err);
      return;
  }

  rt_size_t read_size = 0;
  while (1)
  {
      err = rt_sem_take(&can_data.sem, MAX_SEM_WAIT_TIME);
      if ( err != RT_EOK)
      {
          if(getState(od_data) == Operational) 
          {               
            LOG_W("canfestival wait receive timeout, err = %d", err);
          }
      } 
      else 
      {
        read_size = rt_device_read(candev, 0, &msg, sizeof(msg));
        if( read_size == sizeof(msg))
        {
          co_msg.cob_id = msg.id;
          co_msg.len = msg.len;
          co_msg.rtr = msg.rtr;
          memcpy(co_msg.data, msg.data, msg.len);
          EnterMutex();
          canDispatch(od_data, &co_msg);
          LeaveMutex();
        } 
        else if (read_size == 0)
        {
           LOG_W("canfestival receive faild, err = %d", rt_get_errno());
        } 
        else 
        {
           LOG_W("canfestival receive size wrong, size = %u", read_size);
        }
      }
  }
}
/**
  * @brief  初始化can硬件.
  * @param  None.
  * @retval None.
  * @note   None.
*/
CAN_PORT canOpen(s_BOARD *board, CO_Data * d)
{
	rt_thread_t tid;
	canfstvl_mutex = rt_mutex_create("canfstvl",RT_IPC_FLAG_PRIO);
    
	od_data = d;
  tid = rt_thread_create("cf_recv",
                         canopen_recv_thread_entry, &can_data,
                         THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (tid != RT_NULL) rt_thread_startup(tid);

  return 0;
}