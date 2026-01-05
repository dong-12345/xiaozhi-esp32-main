/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "touch_element/touch_button.h"
#include "esp_log.h"

#include <cJSON.h>
#include <esp_log.h>

#include <cstdlib> 
#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "otto_movements.h"
#include "sdkconfig.h"
#include "settings.h"
#include <wifi_station.h>
#include "otto_controller.h"


static const char *TAG = "Touch Button";
#define TOUCH_BUTTON_NUM 1
#define TOUCH_BUTTON_SENS 0.018F

/* 触摸按钮句柄 */
static touch_button_handle_t button_handle[TOUCH_BUTTON_NUM];

/* 触摸按钮通道数组 */
static const touch_pad_t channel_array[TOUCH_BUTTON_NUM] = {
    TOUCH_PAD_NUM12,
};

/* 触摸按钮通道灵敏度数组 */
static const float channel_sens_array[TOUCH_BUTTON_NUM] = {
    TOUCH_BUTTON_SENS,
};

/* 按钮事件处理任务 */
static void button_handler_task(void *arg)
{
    (void)arg; // 未使用
    
    touch_elem_message_t element_message;
    while (1)
    {
        // 等待触摸元素消息
        touch_element_message_receive(&element_message, portMAX_DELAY);
        if (element_message.element_type != TOUCH_ELEM_TYPE_BUTTON) {
            continue;
        }
        // 解析消息
        const touch_button_message_t *button_message = touch_button_get_message(&element_message);
        if (button_message->event == TOUCH_BUTTON_EVT_ON_PRESS) {
            ESP_LOGI(TAG, "Button[%d] Press", (int)element_message.arg);
            // 根据不同按钮执行不同动作
            int button_num = (int)element_message.arg;
            if (button_num == TOUCH_PAD_NUM12) 
            {
                // 触摸按钮12 - 让机器人挥手
                QueueOttoAction(ACTION_HAND_WAVE, 1, 0, 1, 0);
                //Shy(int dir = LEFT, float steps = 5);  // 害羞
            } 
        }
        else if (button_message->event == TOUCH_BUTTON_EVT_ON_RELEASE) 
        {
            ESP_LOGI(TAG, "Button[%d] Release", (int)element_message.arg);
        } 
        else if (button_message->event == TOUCH_BUTTON_EVT_ON_LONGPRESS) 
        {
            ESP_LOGI(TAG, "Button[%d] LongPress", (int)element_message.arg);
        }
    }        
    
    uint32_t touch_value;
    uint32_t benchmark;
    
    while (1)
    {
        for (int i = 0; i < TOUCH_BUTTON_NUM; i++)
        {
            touch_pad_read_raw_data(channel_array[i], &touch_value); // read raw data.
            printf("T%dtouch_value: [%4" PRIu32 "] ", channel_array[i], touch_value);
            touch_pad_read_benchmark(channel_array[i], &benchmark);
            printf("T%dbenchamrk: [%4" PRIu32 "] ", channel_array[i], benchmark);
        }
        printf("\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}


void touch_main(void)
{
    /* 初始化触摸元件库 */
    touch_elem_global_config_t global_config = TOUCH_ELEM_GLOBAL_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(touch_element_install(&global_config));
    ESP_LOGI(TAG, "Touch element library installed");

    touch_button_global_config_t button_global_config = TOUCH_BUTTON_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_button_install(&button_global_config));
    ESP_LOGI(TAG, "Touch button installed");
    
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++)
    {
        touch_button_config_t button_config = {
            .channel_num = channel_array[i],
            .channel_sens = channel_sens_array[i]
        };

        /* 创建触摸按钮 */
        ESP_ERROR_CHECK(touch_button_create(&button_config, &button_handle[i]));
        /* 订阅触摸按钮事件（按下、释放、长按） */
        ESP_ERROR_CHECK(touch_button_subscribe_event(button_handle[i],
                                                    TOUCH_ELEM_EVENT_ON_PRESS | TOUCH_ELEM_EVENT_ON_RELEASE | TOUCH_ELEM_EVENT_ON_LONGPRESS,
                                                    (void *)channel_array[i]));
        
        ESP_ERROR_CHECK(touch_button_set_dispatch_method(button_handle[i], TOUCH_ELEM_DISP_EVENT));

        /* 设置长按事件触发阈值时间 */
        ESP_ERROR_CHECK(touch_button_set_longpress(button_handle[i], 1000));
    }
    ESP_LOGI(TAG, "Touch buttons created");

    /* 创建处理任务来处理事件消息 */
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * 1024, NULL, 5, NULL);

    touch_element_start();
    ESP_LOGI(TAG, "Touch element library start");
}