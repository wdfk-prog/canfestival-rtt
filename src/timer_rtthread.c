/**
 * @file timer_rtthread.c
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
/* Includes ------------------------------------------------------------------*/
#include "canfestival.h"
#include "canopen_timer.h"
#include "timers_driver.h"
/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <main.h>
//#define DBG_TAG "app.CANopen"
//#define DBG_LVL DBG_LOG
//#include <rtdbg.h>
/* Private typedef -----------------------------------------------------------*/
struct stm32_hwtimer
{
    rt_hwtimer_t time_device;
    TIM_HandleTypeDef    tim_handle;
    IRQn_Type tim_irqn;
    char *name;
};
/* Private define ------------------------------------------------------------*/
/* 线程配置 */
#define THREAD_PRIORITY      CANFESTIVAL_TIMER_THREAD_PRIO//线程优先级
#define THREAD_TIMESLICE     20//线程时间片
#define THREAD_STACK_SIZE    2048//栈大小

//#define IRQ_PRIORITY  0
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static rt_sem_t canfstvl_timer_sem = RT_NULL;
static rt_device_t canfstvl_timer_dev=RT_NULL;
static rt_hwtimerval_t last_timer_val;
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  设置定时器
  * @param  None
  * @retval None
  * @note   None
*/
void setTimer(TIMEVAL value)
{
	rt_hwtimerval_t val;
	
	val.sec = value / 1000000;
	val.usec = value % 1000000;
	/// Avoid invalid 0 timeout.
	/// See here for detailed reference: https://club.rt-thread.org/ask/question/11194.html .
	if(val.usec < MIN_TIMER_TIMEOUT_US){
		val.usec = MIN_TIMER_TIMEOUT_US;
	}

	last_timer_val.sec = 0;
	last_timer_val.usec = 0;
	if(rt_device_write(canfstvl_timer_dev, 0, &val, sizeof(val)) == 0) {
        LOG_E("CANopen set timer failed, err = %d", rt_get_errno());
	}
}
/**
  * @brief  获取当前运行时间.
  * @param  None.
  * @retval None.
  * @note   None.
*/
TIMEVAL getElapsedTime(void)
{
	rt_hwtimerval_t val;
    
	rt_device_read(canfstvl_timer_dev, 0, &val, sizeof(val));

	return (val.sec - last_timer_val.sec) * 1000000 + (val.usec - last_timer_val.usec);
}
/**
  * @brief  定时器线程.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void canopen_timer_thread_entry(void* parameter)
{	
	while(1)
	{
		if(rt_sem_take(canfstvl_timer_sem, RT_WAITING_FOREVER) != RT_EOK) 
    {
            LOG_E("canfestival take timer sem failed");
            return;
		}

		EnterMutex();
		rt_size_t read_size = rt_device_read(canfstvl_timer_dev, 0, &last_timer_val, sizeof(last_timer_val));
		if( read_size == 0) 
    {
            LOG_E("canfestival read timer failed, err = %d", rt_get_errno());
		}
    else 
    {
            TimeDispatch();
		}
		LeaveMutex();
	}
}
/**
  * @brief  定时器回调函数.
  * @param  None.
  * @retval None.
  * @note   到点释放信号量.
*/
static rt_err_t timer_timeout_cb(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(canfstvl_timer_sem);
    
    return RT_EOK;
}
/**
  * @brief  初始化定时器.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void initTimer(void)
{
	rt_thread_t tid;
	rt_err_t err;
	rt_hwtimer_mode_t mode;
	int freq = 1000000;

	canfstvl_timer_sem = rt_sem_create("canfstvl", 0, RT_IPC_FLAG_PRIO);

	canfstvl_timer_dev = rt_device_find(CANFESTIVAL_TIMER_DEVICE_NAME);
	RT_ASSERT(canfstvl_timer_dev != RT_NULL);
	err = rt_device_open(canfstvl_timer_dev, RT_DEVICE_OFLAG_RDWR);
	if (err != RT_EOK)
  {
      rt_kprintf("CanFestival open timer Failed! err=%d\n", err);
      return;
  }
	rt_device_set_rx_indicate(canfstvl_timer_dev, timer_timeout_cb);
	err = rt_device_control(canfstvl_timer_dev, HWTIMER_CTRL_FREQ_SET, &freq);
  if (err != RT_EOK)
  {
      rt_kprintf("Set Freq=%dhz Failed\n", freq);
  }

  mode = HWTIMER_MODE_ONESHOT;
  err = rt_device_control(canfstvl_timer_dev, HWTIMER_CTRL_MODE_SET, &mode);
	rt_device_read(canfstvl_timer_dev, 0, &last_timer_val, sizeof(last_timer_val));
#ifdef IRQ_PRIORITY
    struct stm32_hwtimer *tim_device = RT_NULL;
    tim_device = rt_container_of(canfstvl_timer_dev, struct stm32_hwtimer, time_device);
    
    HAL_NVIC_SetPriority(tim_device->tim_irqn,IRQ_PRIORITY, 0);
#endif
	tid = rt_thread_create("cf_timer",
                           canopen_timer_thread_entry, RT_NULL,
                           THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
  if (tid != RT_NULL) rt_thread_startup(tid);

}
/**
  * @brief  开启定时器循环.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static TimerCallback_t init_callback;
void StartTimerLoop(TimerCallback_t _init_callback) 
{
	init_callback = _init_callback;
	EnterMutex();
  //设置5us一次性定时器
	SetAlarm(NULL, 0, init_callback, 5, 0);
	LeaveMutex();
}