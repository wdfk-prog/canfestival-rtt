This is a package for RT-Thread operating system.

Forked from the CanFestival-3 project https://bitbucket.org/Mongo/canfestival-3-asc

# CanFestival -RTT

## 1������

- ���Ľ���

  ��CanFestival -RTT�����޸�

  1. canfestivalԴ����δ����bug��û�н����޸�

  2. canfestival���ֵ����ɹ����ж���DS402�����ļ���ȫ�棬��������.

  3. Master402�н����޸Ĵ����޸�

     - ����֧��SDO���Դ�����֧��switchѭ��ִ��

     - �Բ���ҪƵ����д��������pdo����

     - ���DS402֧�ֵ��˶�ģʽ��PP,PV,HM
     - ����CMD����֧�ֽڵ�NMT״̬�鿴������˶����Ƽ�����
     - �Խڵ���ߡ����硢���߶��ߡ���·��ͨ���쳣������д�����֤ͨ��ϵͳ�ȶ����ڵ�����»ָ�ͨ��
     - ���doxygen����API�ֲ�

     

- Դ������� https://github.com/gbcwbz/canfestival-rtt

�� package �� Canfestival (һ����Դ�� CANopen Э��ջ)�� RT-Thread ϵͳ�ϵ���ֲ��ʹ����
 RT-Thread �� CAN ������ hwtimer �������Ӷ����������������ṩ��������������ƽ̨��
ͬʱ�ṩ�� CANopen ��һЩʾ������ͼ�������伴�á�

### 1.1 Ŀ¼�ṹ

| ���� | ˵�� |
| ---- | ---- |
| docs  | �ĵ�Ŀ¼ |
| Master402 | DS402 ��վ�����ڿ����ŷ���� |
| inc  | ͷ�ļ�Ŀ¼ |
| src  | Դ����Ŀ¼ |

### 1.2 ���֤

Canfestival package ��ѭ LGPLv2.1 ��ɣ���� `LICENSE` �ļ���

### 1.3 ����

- RT-Thread 3.0+
- CAN ����
- hwtimer ����

## 2����δ� CanFestival

ʹ�� CanFestival package ��Ҫ�� RT-Thread �İ���������ѡ����������·�����£�

```
RT-Thread online packages
    miscellaneous packages --->
        [*] CanFestival: A free software CANopen framework
```

Ȼ���� RT-Thread �İ��������Զ����£�����ʹ�� `pkgs --update` ������°��� BSP �С�

## 3��ʹ�� Canfestival

�� menuconfig �д� CAN ������ hwtimer����
������  CanFestival config �����ú� CAN ������ device name, �Լ� hwtimer ������ device name
```
(can1) CAN device name for CanFestival
(timer1) hwtimer device name for CanFestival
(9) The priority level value of can receive thread
(10) The priority level value of timer thread
[*] Enable Cia402 Master example
```
������Ҫ���� can �����̣߳���ʱ���̵߳����ȼ���
ѡ����Ҫʹ�õ����ӡ�

�ڴ� Canfestival package �󣬵����� bsp ����ʱ�����ᱻ���뵽 bsp �����н��б��롣

## 5����ϵ��ʽ & ��л

* ��л��gbcwbz 

  https://github.com/gbcwbz/canfestival-rtt

* ά����wdfk-prog 

  https://github.com/wdfk-prog/canfestival-rtt
