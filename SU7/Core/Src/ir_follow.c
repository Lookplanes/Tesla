#include "stm32f1xx_hal.h"
#include "main.h"
#include "ir_follow.h"
#include "motor.h"
#include "delay.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

static IRTrackState_t s_state = {0};
static uint32_t s_last_tx_tick = 0; // 上次发送时间戳

// 将占空比读数（0~7199 或类似）映射到速度百分比（-100~100）
static inline float pct_to_speed_cms(int pct)
{
    // 速度开环估算：百分比 * 最大速度
    return (MAX_SPEED_CM_S * ((float)pct) / 100.0f);
}

void IRFollow_Init(void)
{
    s_state.x_cm = 0.0f;
    s_state.y_cm = 0.0f;
    // 启动方向：正前为 +Y；用数学角度：theta=pi/2 指向 +Y
#ifdef M_PI_2
    s_state.theta = (float)M_PI_2;
#else
    s_state.theta = IR_PI * 0.5f;
#endif
    // 根据电机基准占空比推导默认基础速度：略低于 MOTOR_SPEED，便于稳定跟随
    int default_base = IR_BASE_MOTOR_SPEED - 10;
    if (default_base < 10) default_base = 10;
    if (default_base > 100) default_base = 100;
    s_state.baseSpeed = default_base;
    s_state.enabled = 0;
    s_last_tx_tick = HAL_GetTick();
}

void IRFollow_SetBaseSpeed(int speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    s_state.baseSpeed = speed;
}

void IRFollow_Enable(uint8_t enable)
{
    s_state.enabled = enable ? 1 : 0;
    if (!s_state.enabled) {
        Motor_Stop();
    }
}

static inline uint8_t ir_left_detected(void)
{
    return HAL_GPIO_ReadPin(AVOID_LEFT_GPIO_Port, AVOID_LEFT_Pin) == GPIO_PIN_RESET; // 低电平有效
}

static inline uint8_t ir_right_detected(void)
{
    return HAL_GPIO_ReadPin(AVOID_RIGHT_GPIO_Port, AVOID_RIGHT_Pin) == GPIO_PIN_RESET; // 低电平有效
}

// 姿态开环更新：根据左右轮速度百分比估算线速度与角速度
// 左右轮百分比 => 线速度 v = pct_avg * Vmax；角速度 w 由差速近似，比例系数 kW 可由实测调参
// IR_TRACK_KW 默认已在头文件依据电机参数推导；若头文件未定义，这里给出安全兜底
#ifndef IR_TRACK_KW
#define IR_TRACK_KW 0.02f // 角速度比例系数（rad/s per %diff），需根据底盘调参
#endif

static void update_pose_open_loop(int spdL_pct, int spdR_pct, float dt_s)
{
    float v = pct_to_speed_cms((spdL_pct + spdR_pct) / 2);
    float w = IR_TRACK_KW * (float)(spdR_pct - spdL_pct);

    s_state.theta += w * dt_s;
    // 归一化到 [-pi, pi]
    if (s_state.theta > IR_PI) s_state.theta -= 2.0f * IR_PI;
    if (s_state.theta < -IR_PI) s_state.theta += 2.0f * IR_PI;

    s_state.x_cm += v * cosf(s_state.theta) * dt_s;
    s_state.y_cm += v * sinf(s_state.theta) * dt_s;
}

// 发送位姿字符串："$POS,x.x,y.y\n"
static void tx_pose_if_needed(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_tx_tick) >= IR_TRACK_TX_INTERVAL_MS) {
        char buf[64];
        // 使用整数协议，避免依赖 printf 的浮点支持：
        // $POSI,<x_ccm>,<y_ccm>\n 其中 ccm=0.01cm（保留两位小数）
        float x_scaled = s_state.x_cm * 100.0f;
        float y_scaled = s_state.y_cm * 100.0f;
        int32_t x_ccm = (int32_t)((x_scaled >= 0.0f) ? (x_scaled + 0.5f) : (x_scaled - 0.5f));
        int32_t y_ccm = (int32_t)((y_scaled >= 0.0f) ? (y_scaled + 0.5f) : (y_scaled - 0.5f));
        int n = snprintf(buf, sizeof(buf), "$POSI,%ld,%ld\n", (long)x_ccm, (long)y_ccm);
        if (n > 0 && n < (int)sizeof(buf)) {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100);
        }
        s_last_tx_tick = now;
    }
}

void IRTrack_Loop(void)
{
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//	HAL_Delay(200);
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
//	HAL_Delay(200);
//    if (!s_state.enabled) return;

    // 读取红外传感器
    uint8_t left = ir_left_detected();
    uint8_t right = ir_right_detected();

    // 速度决策（降低速度，保持稳定）
    int spdL = 0, spdR = 0;
    if (left && right) {
        // 中间有物体：直行
        spdL = s_state.baseSpeed;
        spdR = s_state.baseSpeed;
    } else if (left) {
        // 左边有物体：左转
        spdL = -(s_state.baseSpeed);
        spdR = +(s_state.baseSpeed);
    } else if (right) {
        // 右边有物体：右转
        spdL = +(s_state.baseSpeed);
        spdR = -(s_state.baseSpeed);
    } else {
        // 未检测到：慢速直行
        spdL = s_state.baseSpeed / 2;
        spdR = s_state.baseSpeed / 2;
    }

    // 设置电机速度
    Motor_SetSpeedLR(spdL, spdR);

    // 姿态更新：dt = IR_TRACK_DT_MS
    update_pose_open_loop(spdL, spdR, (float)IR_TRACK_DT_MS / 1000.0f);

    // 位姿上报
    tx_pose_if_needed();

    // 循环周期控制
    HAL_Delay(IR_TRACK_DT_MS);
}
