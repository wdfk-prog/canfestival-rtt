This is a package for RT-Thread operating system.

Forked from the CanFestival-3 project https://bitbucket.org/Mongo/canfestival-3-asc

# CanFestival -RTT

## 1、介绍

- 更改介绍

  对CanFestival -RTT做出修改

  1. canfestival源码暂未出现bug，没有进行修改

  2. canfestival的字典生成工具中对于DS402配置文件不全面，已做更新.

  3. Master402中进行修改大量修改

     - 单个支持SDO命令，源代码仅支持switch循环执行

     - 对不需要频繁读写变量改用pdo传输

     - 添加DS402支持的运动模式，PP,PV,HM
     - 增加CMD命令支持节点NMT状态查看，电机运动控制及反馈
     - 对节点掉线、掉电、总线断线、短路等通信异常情况进行处理。保证通信系统稳定、节点可重新恢复通信
     - 添加doxygen生成API手册

     

- 源代码介绍 https://github.com/gbcwbz/canfestival-rtt

此 package 是 Canfestival (一个开源的 CANopen 协议栈)在 RT-Thread 系统上的移植。使用了
 RT-Thread 的 CAN 驱动和 hwtimer 驱动，从而可以运行于所有提供了这两个驱动的平台。
同时提供了 CANopen 的一些示例，力图做到开箱即用。

### 1.1 目录结构

| 名称 | 说明 |
| ---- | ---- |
| docs  | 文档目录 |
| Master402 | DS402 主站，用于控制伺服电机 |
| inc  | 头文件目录 |
| src  | 源代码目录 |

### 1.2 许可证

Canfestival package 遵循 LGPLv2.1 许可，详见 `LICENSE` 文件。

### 1.3 依赖

- RT-Thread 3.0+
- CAN 驱动
- hwtimer 驱动

## 2、如何打开 CanFestival

使用 CanFestival package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    miscellaneous packages --->
        [*] CanFestival: A free software CANopen framework
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、使用 Canfestival

在 menuconfig 中打开 CAN 驱动和 hwtimer驱动
并且在  CanFestival config 中配置好 CAN 驱动的 device name, 以及 hwtimer 驱动的 device name
```
(can1) CAN device name for CanFestival
(timer1) hwtimer device name for CanFestival
(9) The priority level value of can receive thread
(10) The priority level value of timer thread
[*] Enable Cia402 Master example
```
根据需要配置 can 接收线程，和时钟线程的优先级。
选择需要使用的例子。

在打开 Canfestival package 后，当进行 bsp 编译时，它会被加入到 bsp 工程中进行编译。

## 5、联系方式 & 感谢

* 感谢：gbcwbz 

  https://github.com/gbcwbz/canfestival-rtt

* 维护：wdfk-prog 

  https://github.com/wdfk-prog/canfestival-rtt
