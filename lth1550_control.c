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

/* ADC的相关API接口 Related API interfaces of ADC */
#include <stdio.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio_ex.h"
#include "iot_gpio.h"
#include "iot_adc.h"
#include "hi_adc.h"
#include "iot_watchdog.h"
#include "iot_errno.h"

extern current_humidity_value;
extern current_temp_value;

void Lth1550Init(void)
{
    /* 红外对管对应的GPIO，左：ADC0_GPIO12，右：ADC3_GPIO07 */
    /* GPIO corresponding to infrared tube, left: ADC0_ GPIO12, right: ADC3_ GPIO07 */
    IoTGpioInit(IOT_IO_NAME_GPIO_12);
    /* 设置GPIO07的管脚复用关系为GPIO */
    /* Set the pin reuse relationship of GPIO07 to GPIO */
    IoSetFunc(IOT_IO_NAME_GPIO_12, IOT_IO_FUNC_GPIO_12_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_12, IOT_GPIO_DIR_IN);
}

void GetInfraredData(IotAdcChannelIndex idx)
{
    unsigned short data = 0;
    unsigned short data1 = 0;
    int ret ;
    /* ADC_Channal_6  自动识别模式  CNcomment:4次平均算法模式 CNend 0xff */
    /* ADC_ Channal_ 6 Automatic recognition mode CNcomment: 4 times average algorithm mode CNend 0xff */
    ret = AdcRead(idx, &data, IOT_ADC_EQU_MODEL_8 ,IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        printf("hi_adc_read failed\n");
    }
    data1=data+1500;
    if (IOT_ADC_CHANNEL_0 == idx) {
        printf(" ADC value is %d \r\n", data1);
    } 
    current_humidity_value=((data1%100));
    current_temp_value=(data1/100);
}

static void adcTask(void)
{
    Lth1550Init();
    while (1) {
        GetInfraredData(IOT_ADC_CHANNEL_0);
       // GetInfraredData(IOT_ADC_CHANNEL_0);
        usleep(2000); // wait 20 us
    }
}

void ADCExampleEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "adcTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 5 * 1024; // 堆栈大小5*1024，stack size 5*1024
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)adcTask, NULL, &attr) == NULL) {
        printf("[LSM6DSTask] Failed to create LSM6DSTask!\n");
    }
}

APP_FEATURE_INIT(ADCExampleEntry);