/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
    HCSR05 超声波模块的相关API接口
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "hi_timer.h"

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_watchdog.h"
#include "iot_gpio.h"
#include "hi_time.h"
#include "hi_io.h"
#include "iot_gpio_ex.h"
#include "pca9555.h"
#include "iot_watchdog.h"
#include "iot_pwm.h"
#include "iot_errno.h"
#include "hi_errno.h"

#define DELAY_US20    20
#define DELAY_MS10    10

#define IOT_PWM_PORT_PWM0   0
#define IOT_PWM_PORT_PWM1   1
#define IOT_PWM_PORT_PWM2   2
#define IOT_PWM_PORT_PWM3   3
#define IOT_FREQ            65535
#define IOT_DUTY            50
#define  COUNT   10
#define  FREQ_TIME    20000

static int IOT_FRQE1=4000;
//蜂鸣器模块
void buzzer(void)
{
    IoTPwmStart(IOT_PWM_PORT_PWM3, IOT_DUTY, IOT_FRQE1);
}
//左轮向前
void LeftWheelForword(void)
{
    IoTPwmStart(IOT_PWM_PORT_PWM2, IOT_DUTY, IOT_FREQ);
}
//左轮向后
void LeftWheelBackword(void)
{
    IoTPwmStart(IOT_PWM_PORT_PWM3, IOT_DUTY, IOT_FREQ);
}
//左轮停下
void LeftWheelStop(void)
{
    IoTPwmStop(IOT_PWM_PORT_PWM2);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
}
//右轮向前
void RightWheelForword(void)
{
    IoTPwmStart(IOT_PWM_PORT_PWM1, IOT_DUTY, IOT_FREQ);
}
//右轮向后
void RightWheelBackword(void)
{
    IoTPwmStart(IOT_PWM_PORT_PWM0, IOT_DUTY, IOT_FREQ);
}
//右轮停止
void RightWheelStop(void)
{
    IoTPwmStop(IOT_PWM_PORT_PWM0);
    IoTPwmStop(IOT_PWM_PORT_PWM1);
}
//电机初始化
void GA12N20Init(void)
{
    // 左电机GPIO5,GPIO6初始化 Initialization of left motor GPIO5 and GPIO6
    IoTGpioInit(IOT_IO_NAME_GPIO_5);
    IoTGpioInit(IOT_IO_NAME_GPIO_6);
    // 右电机GPIO9, GPIO10初始化 Right motor GPIO9, GPIO10 initialization
    IoTGpioInit(IOT_IO_NAME_GPIO_9);
    IoTGpioInit(IOT_IO_NAME_GPIO_10);

    // 设置GPIO5的管脚复用关系为PWM2输出 Set the pin multiplexing relationship of GPIO5 to PWM2 output
    IoSetFunc(IOT_IO_NAME_GPIO_5, IOT_IO_FUNC_GPIO_5_PWM2_OUT);
    // 设置GPIO6的管脚复用关系为PWM3输出 Set the pin multiplexing relationship of GPIO6 to PWM3 output
    IoSetFunc(IOT_IO_NAME_GPIO_6, IOT_IO_FUNC_GPIO_6_PWM3_OUT);
    // 设置GPIO9的管脚复用关系为PWM0输出 Set the pin multiplexing relationship of GPIO9 to PWM0 output
    IoSetFunc(IOT_IO_NAME_GPIO_9, IOT_IO_FUNC_GPIO_9_PWM0_OUT);
    // 设置GPIO10的管脚复用关系为PWM01输出 Set the pin multiplexing relationship of GPIO10 to PWM01 output
    IoSetFunc(IOT_IO_NAME_GPIO_10, IOT_IO_FUNC_GPIO_10_PWM1_OUT);

    // GPIO5方向设置为输出 GPIO5 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_5, IOT_GPIO_DIR_OUT);
    // GPIO6方向设置为输出 GPIO6 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_6, IOT_GPIO_DIR_OUT);
    // GPIO9方向设置为输出 GPIO9 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_9, IOT_GPIO_DIR_OUT);
    // GPIO10方向设置为输出 GPIO10 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_10, IOT_GPIO_DIR_OUT);
    IoTGpioInit(IOT_IO_NAME_GPIO_12);
    // 设置GPIO6的管脚复用关系为PWM3输出 Set the pin multiplexing relationship of GPIO6 to PWM3 output
    IoSetFunc(IOT_IO_NAME_GPIO_12, IOT_IO_FUNC_GPIO_12_PWM3_OUT);

    IoTGpioSetDir(IOT_IO_NAME_GPIO_12, IOT_GPIO_DIR_OUT);
    // 初始化PWM2 Initialize PWM2
    IoTPwmInit(IOT_PWM_PORT_PWM2);
    // 初始化PWM3 Initialize PWM3
    IoTPwmInit(IOT_PWM_PORT_PWM3);
    // 初始化PWM0 Initialize PWM0
    IoTPwmInit(IOT_PWM_PORT_PWM0);
    // 初始化PWM1 Initialize PWM1
    IoTPwmInit(IOT_PWM_PORT_PWM1);
    // 先使两个电机处于停止状态 motors stop

    RightWheelStop();
    LeftWheelStop();
}
//超声波初始化
void Hcsr04Init(void)
{
    /*
     * 设置超声波Echo为输入模式
     * 设置GPIO8功能（设置为GPIO功能）
     * Set ultrasonic echo as input mode
     * Set GPIO8 function (set as GPIO function)
     */
    IoSetFunc(IOT_IO_NAME_GPIO_8, IOT_IO_FUNC_GPIO_8_GPIO);
    /*
     * 设置GPIO8为输入方向
     * Set GPIO8 as the input direction
     */
    IoTGpioSetDir(IOT_IO_NAME_GPIO_8, IOT_GPIO_DIR_IN);
    /*
     * 设置GPIO7功能（设置为GPIO功能）
     * Set GPIO7 function (set as GPIO function)
     */
    IoSetFunc(IOT_IO_NAME_GPIO_7, IOT_IO_FUNC_GPIO_7_GPIO);
    /*
     * 设置GPIO7为输出方向
     * Set GPIO7 as the output direction
     */
    IoTGpioSetDir(IOT_IO_NAME_GPIO_7, IOT_GPIO_DIR_OUT);
}

void SetAngle(unsigned int duty)
{
    unsigned int time1 = FREQ_TIME;

    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_2, IOT_GPIO_VALUE1);
    hi_udelay(duty);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_2, IOT_GPIO_VALUE0);
    hi_udelay(time1 - duty);
}

/* The steering gear is centered
 * 1、依据角度与脉冲的关系，设置高电平时间为1500微秒
 * 2、不断地发送信号，控制舵机居中
*/
void RegressMiddle(void)
{
    unsigned int angle = 1500;
    for (int i = 0; i < COUNT; i++) {
        SetAngle(angle);
    }
}

/* Turn 90 degrees to the right of the steering gear
 * 1、依据角度与脉冲的关系，设置高电平时间为500微秒
 * 2、不断地发送信号，控制舵机向右旋转90度
*/
/*  Steering gear turn right */
void EngineTurnRight(void)
{
    unsigned int angle = 500;
    for (int i = 0; i < COUNT; i++) {
        SetAngle(angle);
    }
}

/* Turn 90 degrees to the left of the steering gear
 * 1、依据角度与脉冲的关系，设置高电平时间为2500微秒
 * 2、不断地发送信号，控制舵机向左旋转90度
*/
/* Steering gear turn left */
void EngineTurnLeft(void)
{
    unsigned int angle = 2450;
    for (int i = 0; i < COUNT; i++) {
        SetAngle(angle);
    }
}

void S92RInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_2);
    IoSetFunc(IOT_IO_NAME_GPIO_2, IOT_IO_FUNC_GPIO_2_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_2, IOT_GPIO_DIR_OUT);
}

static float distance = 0.0;
//距离获取函数
float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    /*
     * 设置GPIO7输出低电平
     * 给trig发送至少10us的高电平脉冲，以触发传感器测距
     * Set GPIO7 to output direction
     * Send a high level pulse of at least 10us to the trig to trigger the range measurement of the sensor
     */
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE1);
    hi_udelay(DELAY_US20);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE0);
    /*
     * 计算与障碍物之间的距离
     * Calculate the distance from the obstacle
     */
    while (1) {
        /*
         * 获取GPIO8的输入电平状态
         * Get the input level status of GPIO8
         */
        IoTGpioGetInputVal(IOT_IO_NAME_GPIO_8, &value);
        /*
         * 判断GPIO8的输入电平是否为高电平并且flag为0
         * Judge whether the input level of GPIO8 is high and the flag is 0
         */
        if (value == IOT_GPIO_VALUE1 && flag == 0) {
            /*
             * 获取系统时间
             * get SysTime
             */
            start_time = hi_get_us();
            flag = 1;
        }
        /*
         * 判断GPIO8的输入电平是否为低电平并且flag为1
         * Judge whether the input level of GPIO8 is low and the flag is 1
         */
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            /*
             * 获取高电平持续时间
             * Get high level duration
             */
            time = hi_get_us() - start_time;
            break;
        }
    }
    /* 计算距离障碍物距离（340米/秒 转换为 0.034厘米/微秒, 2代表去来，两倍距离） */
    /* Calculate the distance from the obstacle */
    /* (340 m/s is converted to 0.034 cm/microsecond 2 represents going and coming, twice the distance) */
    distance = time * 0.034 / 2;
    printf("distance is %0.2f cm\r\n", distance);
    return distance;
}

static volatile int g_buttonState = 0;

void OnFuncKeyPressed(char *arg)
{
    (void) arg;
    g_buttonState = 1;
}

void FuncKeyInit(void)
{
    /*
     * 使能GPIO11的中断功能, OnFuncKeyPressed 为中断的回调函数
     * Enable the interrupt function of GPIO11. OnFuncKeyPressed is the interrupt callback function
     */
    IoTGpioRegisterIsrFunc(IOT_IO_NAME_GPIO_11, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW, OnFuncKeyPressed, NULL);
    /*
     * S3:IO0_2,S4:IO0_3,S5:IO0_4 0001 1100 => 0x1c 将IO0_2,IO0_3,IO0_4方向设置为输入，1为输入，0位输出
     * S3:IO0_ 2,S4:IO0_ 3,S5:IO0_ 4 0001 1100=>0x1c Change IO0_ 2,IO0_ 3,IO0_ 4 direction is set as
     * input, 1 is input, and 0 bit is output
     */
    SetPCA9555GpioValue(PCA9555_PART0_IODIR, 0x1c);
}

void GetFunKeyState(void)
{
    GA12N20Init();
    uint8_t ext_io_state = 0;
    uint8_t ext_io_state_d = 0;
    uint8_t status;
    while(1)
    {
        if (g_buttonState == 1) {
            uint8_t diff;
            status = PCA9555I2CReadByte(&ext_io_state);
            if (status != IOT_SUCCESS) {
                printf("i2c error!\r\n");
                ext_io_state = 0;
                ext_io_state_d = 0;
                g_buttonState = 0;
            }
            diff = ext_io_state ^ ext_io_state_d;
            if (diff == 0) {
                printf("diff = 0! state:%0X, %0X\r\n", ext_io_state, ext_io_state_d);
            }
            if ((diff & 0x10) && ((ext_io_state & 0x10) == 0)) {
                printf("button1 pressed,\r\n");
                TaskMsleep(1000);
                for ( IOT_FRQE1 = 4000; IOT_FRQE1< 4010; IOT_FRQE1++)
                {
                    buzzer();
                    TaskMsleep(10);
                }
                IoTPwmStop(IOT_PWM_PORT_PWM3);
                break;
            }
            status = PCA9555I2CReadByte(&ext_io_state);
            ext_io_state_d = ext_io_state;
            g_buttonState = 0;
        }
    }
    unsigned int time = 200;
    Hcsr04Init();
    S92RInit();
    LeftWheelForword();
    RightWheelForword();
    while(1)
    {
        GetDistance();
        osDelay(DELAY_MS10);
        float a=0.0,c=0.0;
        if (distance<25.0) {
        RightWheelStop();
        LeftWheelStop();

        /* 舵机归中 Steering gear centering */
        RegressMiddle();
        TaskMsleep(time);
        GetDistance();
        /*
         * 舵机左转90度
         * Steering gear turns 90 degrees to the left
         */
        EngineTurnLeft();
        TaskMsleep(time);
        GetDistance();
        a=distance;

        /* 舵机归中 Steering gear centering */
        RegressMiddle();
        TaskMsleep(time);

        /*
         * 舵机右转90度
         * Steering gear turns right 90 degrees
         */
        EngineTurnRight();
        TaskMsleep(time);
        GetDistance();
        TaskMsleep(700);
        c=distance;

        /* 舵机归中 Steering gear centering */
        RegressMiddle();
        TaskMsleep(time);

        osDelay(DELAY_MS10);
        if(c>a)
        {
            LeftWheelForword();
            RightWheelStop();
            TaskMsleep(400);
            LeftWheelStop();
            RightWheelStop();
            IoTPwmStop(IOT_PWM_PORT_PWM3);
            LeftWheelForword();
            RightWheelForword();
        }
        else if(a>=c)
        {
            RightWheelForword();
            LeftWheelStop();
            TaskMsleep(400);
            LeftWheelStop();
            RightWheelStop();
            IoTPwmStop(IOT_PWM_PORT_PWM3);
            LeftWheelForword();
            RightWheelForword();
        }
        c=0;
        a=0;
    }
    else if(distance<10)                                                   
    {
        LeftWheelStop();
        RightWheelStop();
    }
    else if(distance>20.0&&distance<80.0)
    {
        LeftWheelForword();
        RightWheelForword();
    }
    else if(distance>80.0)
    {
        LeftWheelStop();
        RightWheelStop();

        /* 舵机归中 Steering gear centering */
        RegressMiddle();
        TaskMsleep(time);
        GetDistance();
        /*
         * 舵机左转90度
         * Steering gear turns 90 degrees to the left
         */
        EngineTurnLeft();
        TaskMsleep(time);
        GetDistance();
        a=distance;

        /* 舵机归中 Steering gear centering */
        RegressMiddle();
        TaskMsleep(time);

        /*
         * 舵机右转90度
         * Steering gear turns right 90 degrees
         */
        EngineTurnRight();
        TaskMsleep(time);
        GetDistance();
        TaskMsleep(700);
        c=distance;

        /* 舵机归中 Steering gear centering */
        RegressMiddle();
        TaskMsleep(time);
        if(a>40.0&&c>40.0)
        {
            for ( IOT_FRQE1 = 4000; IOT_FRQE1< 4010; IOT_FRQE1++)
            {
                buzzer();
                TaskMsleep(10);
            }
            IoTPwmStop(IOT_PWM_PORT_PWM3);
            TaskMsleep(200);
            for ( IOT_FRQE1 = 4000; IOT_FRQE1< 4010; IOT_FRQE1++)
            {
                buzzer();
                TaskMsleep(10);
            }
            IoTPwmStop(IOT_PWM_PORT_PWM3);
            break;
        }
        else 
        {
            LeftWheelForword();
            RightWheelForword();
            TaskMsleep(800);
        }
    }
    }
}

static void Control(void)
{
    /*
     * IO扩展芯片初始化
     * IO expansion chip initialization
     */
    PCA9555Init();
    /*
     * 配置IO扩展芯片的part1的所有管脚为输出
     * Configure all pins of part1 of IO expansion chip as output
     */
    SetPCA9555GpioValue(PCA9555_PART1_IODIR, PCA9555_OUTPUT);
    /*
     * 按键中断初始化
     * Key interrupt initialization
     */
    FuncKeyInit();
    /*
     * 获取实时的按键状态
     * Get real-time key status
     */
    GetFunKeyState();
}

static void ControlEntry(void)
{
    osThreadAttr_t attr;
    attr.name = "CntrolDemo";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024; // 堆栈大小为1024,Stack size is 1024
    attr.priority =   osPriorityNormal;
    if (osThreadNew((osThreadFunc_t)Control, NULL, &attr) == NULL) {
        printf("[LedExample] Failed to create LedTask!\n");
    }
}

APP_FEATURE_INIT(ControlEntry);


