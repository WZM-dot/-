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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "iot_gpio_ex.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_uart.h"
#include "iot_watchdog.h"
#include "iot_errno.h"
#include "hi_errno.h"
#include "oled_ssd1306.h"
#include "iot_i2c.h"

#define UART_BUFF_SIZE 1000
#define U_SLEEP_TIME   100000
#define DELYA_US20            20

void Uart1GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_0);
    // 设置GPIO0的管脚复用关系为UART1_TX Set the pin reuse relationship of GPIO0 to UART1_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_0, IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_1);
    // 设置GPIO1的管脚复用关系为UART1_RX Set the pin reuse relationship of GPIO1 to UART1_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_1, IOT_IO_FUNC_GPIO_1_UART1_RXD);
}

void Uart1Config(void)
{
    uint32_t ret;
    /* 初始化UART配置，波特率 9600，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART configuration, baud rate is 9600, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = IoTUartInit(HI_UART_IDX_1, &uart_attr);
    if (ret != IOT_SUCCESS) {
        printf("Init Uart1 Falied Error No : %d\n", ret);
        return;
    }
}

int current_humidity_value=0;
int current_temp_value=0;
static void UartTask(void)
{
    unsigned char Adata[200];
    const char *data = "AT+TCMQTTPUB=\"$thing/up/property/K63LXM92Q5/wenshidujiance\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"current_humidity\\\":19,\\\"current_temp\\\":2}}\"\r\n";
    const char *data1= "AT\r\n";
    const char *data2= "ATI\r\n";
    const char *data3= "AT+CPIN?\r\n";
    const char *data4= "AT+CSQ\r\n";
    const char *data5= "AT+CGREG?\r\n";
    const char *data6= "AT+GTSET=\"IPRFMT\",5\r\n";
    const char *data7= "AT+CGMR?\r\n";
    const char *data8= "AT+MIPCALL=1\r\n";
    const char *data9= "AT+TCDEVINFOSET=1,\"K63LXM92Q5\",\"wenshidujiance\",\"y7VscV4veETfURs16RtE+g==\"\r\n";
    const char *data10= "AT+TCMQTTCONN=1,20000,240,1,1\r\n";
    const char *data11= "AT+TCMQTTSUB=\"$thing/down/property/K63LXM92Q5/wenshidujiance\",1 \r\n";
    uint32_t count = 0;
    uint32_t len = 0;
    unsigned char uartReadBuff[UART_BUFF_SIZE] = {0};
    uint32_t len1 = 0;
    unsigned char uartReadBuff1[UART_BUFF_SIZE] = {0};
    uint32_t len2 = 0;
    unsigned char uartReadBuff2[UART_BUFF_SIZE] = {0};
    uint32_t len3 = 0;
    unsigned char uartReadBuff3[UART_BUFF_SIZE] = {0};
    uint32_t len4 = 0;
    unsigned char uartReadBuff4[UART_BUFF_SIZE] = {0};
    uint32_t len5 = 0;
    unsigned char uartReadBuff5[UART_BUFF_SIZE] = {0};
    uint32_t len6 = 0;
    unsigned char uartReadBuff6[UART_BUFF_SIZE] = {0};
    uint32_t len7 = 0;
    unsigned char uartReadBuff7[UART_BUFF_SIZE] = {0};
    uint32_t len8 = 0;
    unsigned char uartReadBuff8[UART_BUFF_SIZE] = {0};
    uint32_t len9 = 0;
    unsigned char uartReadBuff9[UART_BUFF_SIZE] = {0};
    uint32_t len10 = 0;
    unsigned char uartReadBuff10[UART_BUFF_SIZE] = {0};
    uint32_t len11 = 0;
    unsigned char uartReadBuff11[UART_BUFF_SIZE] = {0};
    // 对UART1的一些初始化 Some initialization of UART1
    Uart1GpioInit();
    // 对UART1参数的一些配置 Some configurations of UART1 parameters
    Uart1Config();
    while (1) {
        // 通过UART1 发送数据 Send data through UART1
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data1, strlen(data1));
        len1 = IoTUartRead(HI_UART_IDX_1, uartReadBuff1, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data2, strlen(data2));
        len2 = IoTUartRead(HI_UART_IDX_1, uartReadBuff2, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data3, strlen(data3));
        len3 = IoTUartRead(HI_UART_IDX_1, uartReadBuff3, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data4, strlen(data4));
        len4 = IoTUartRead(HI_UART_IDX_1, uartReadBuff4, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data5, strlen(data5));
        len5 = IoTUartRead(HI_UART_IDX_1, uartReadBuff5, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data6, strlen(data6));
        len6 = IoTUartRead(HI_UART_IDX_1, uartReadBuff6, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data7, strlen(data7));
        len7 = IoTUartRead(HI_UART_IDX_1, uartReadBuff7, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data8, strlen(data8));
        len8 = IoTUartRead(HI_UART_IDX_1, uartReadBuff8, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data9, strlen(data9));
        len9 = IoTUartRead(HI_UART_IDX_1, uartReadBuff9, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data10, strlen(data10));
        len10 = IoTUartRead(HI_UART_IDX_1, uartReadBuff10, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data11, strlen(data11));
        len11 = IoTUartRead(HI_UART_IDX_1, uartReadBuff11, UART_BUFF_SIZE);
        TaskMsleep(1000);
        IoTUartWrite(HI_UART_IDX_1, (unsigned char*)data, strlen(data));
        len = IoTUartRead(HI_UART_IDX_1, uartReadBuff, UART_BUFF_SIZE);
        TaskMsleep(1000);
        while(1) {
            sprintf(Adata, "AT+TCMQTTPUB=\"$thing/up/property/K63LXM92Q5/wenshidujiance\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"current_humidity\\\":%d,\\\"current_temp\\\":%d}}\"\r\n",
            current_humidity_value, current_temp_value);
            IoTUartWrite(HI_UART_IDX_1, Adata, strlen(Adata));
            TaskMsleep(1000);
            printf("Uart Read Data is: %s \r\n", uartReadBuff1);
        }
    }
}

void UartExampleEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();

    attr.name = "UartTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 15* 1024; // 任务栈大小*1024 stack size 5*1024
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)UartTask, NULL, &attr) == NULL) {
        printf("[UartTask] Failed to create UartTask!\n");
    }
}

APP_FEATURE_INIT(UartExampleEntry);
