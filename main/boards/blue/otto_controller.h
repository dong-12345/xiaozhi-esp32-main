#ifndef OTTO_CONTROLLER_H
#define OTTO_CONTROLLER_H

#include "freertos/queue.h"

// 动作类型枚举定义
enum OttoActionType {
    ACTION_WALK = 1,
    ACTION_TURN = 2,
    ACTION_JUMP = 3,
    ACTION_SWING = 4,
    ACTION_MOONWALK = 5,
    ACTION_BEND = 6,
    ACTION_SHAKE_LEG = 7,
    ACTION_SIT = 25,  // 坐下
    ACTION_RADIO_CALISTHENICS = 26,  // 广播体操
    ACTION_MAGIC_CIRCLE = 27,  // 爱的魔力转圈圈
    ACTION_UPDOWN = 8,
    ACTION_TIPTOE_SWING = 9,
    ACTION_JITTER = 10,
    ACTION_ASCENDING_TURN = 11,
    ACTION_CRUSAITO = 12,
    ACTION_FLAPPING = 13,
    ACTION_HANDS_UP = 14,
    ACTION_HANDS_DOWN = 15,
    ACTION_HAND_WAVE = 16,
    ACTION_WINDMILL = 20,  // 大风车
    ACTION_TAKEOFF = 21,   // 起飞
    ACTION_FITNESS = 22,   // 健身
    ACTION_GREETING = 23,  // 打招呼
    ACTION_SHY = 24,        // 害羞
    ACTION_SHOWCASE = 28,   // 展示动作
    ACTION_HOME = 17,
    ACTION_SERVO_SEQUENCE = 18,  // 舵机序列（自编程）
    ACTION_WHIRLWIND_LEG = 19    // 旋风腿
};

/*
#ifdef __cplusplus
extern "C" {
#endif
*/

// 初始化Otto控制器
void InitializeOttoController();

// 从外部访问OttoController的QueueAction功能
void QueueOttoAction(int action_type, int steps, int speed, int direction, int amount);

// 获取OttoController内部的动作队列句柄
QueueHandle_t get_otto_controller_queue();

/*
#ifdef __cplusplus
}
#endif
*/

#endif // OTTO_CONTROLLER_H