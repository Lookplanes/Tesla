#ifndef IR_FOLLOW_H
#define IR_FOLLOW_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>
#include "motor.h"  // 使用电机参数（MOTOR_SPEED/K_FORWARD_TIME/K_TURN_TIME）

//=== 运行周期与上报节流 ===
#ifndef IR_TRACK_DT_MS
#define IR_TRACK_DT_MS 50u
#endif

#ifndef IR_TRACK_TX_INTERVAL_MS
#define IR_TRACK_TX_INTERVAL_MS 100u
#endif

//=== 与 motor 参数联动的物理量推导 ===
// - motor.c 以占空比 MOTOR_SPEED（%）驱动时，直行速度约为：V_base = 100 / K_FORWARD_TIME（cm/s）
// - 近似线性假设下，100% 占空比的最大速度约：V_max = V_base * (100 / MOTOR_SPEED)
// - 原 IR 跟随模块用 MAX_SPEED_CM_S 作为 100% 对应的线速度，用它按百分比线性缩放

// 提取/兜底 motor 相关参数
#ifdef K_FORWARD_TIME
#define IR_FORWARD_TIME_MS_PER_MM (K_FORWARD_TIME)
#else
#define IR_FORWARD_TIME_MS_PER_MM (3.5f)
#endif

#ifdef K_TURN_TIME
#define IR_TURN_TIME_MS_PER_DEG (K_TURN_TIME)
#else
#define IR_TURN_TIME_MS_PER_DEG (7.5f)
#endif

#ifdef MOTOR_SPEED
#define IR_BASE_MOTOR_SPEED (MOTOR_SPEED)
#else
#define IR_BASE_MOTOR_SPEED (60)
#endif

// 100% 占空比对应的最大线速度（cm/s）
// π 常量
#ifndef IR_PI
#define IR_PI 3.14159265358979323846f
#endif

#ifndef MAX_SPEED_CM_S
#define MAX_SPEED_CM_S ((100.0f / (IR_FORWARD_TIME_MS_PER_MM)) * (100.0f / (float)IR_BASE_MOTOR_SPEED))
#endif

// 角速度比例系数（rad/s per %diff），与转向时间常数、基准占空比相关：
// 当左右轮以 +/-p 做原地旋转（差为 2p）时：
//   w_true(p) ≈ (p / MOTOR_SPEED) * (1000/K_TURN_TIME) * (pi/180)
//   IR_TRACK_KW 满足：IR_TRACK_KW * (2p) = w_true(p)
// 故 IR_TRACK_KW ≈ (1000*pi/180) / (2 * K_TURN_TIME * MOTOR_SPEED)
#ifndef IR_TRACK_KW
#define IR_TRACK_KW ((1000.0f * IR_PI / 180.0f) / (2.0f * (IR_TURN_TIME_MS_PER_DEG) * (float)IR_BASE_MOTOR_SPEED))
#endif

typedef struct {
    float x_cm;      // 当前位置 X（cm）
    float y_cm;      // 当前位置 Y（cm）
    float theta;     // 朝向（弧度，0 指向 +X，pi/2 指向 +Y）
    int   baseSpeed; // 基础速度百分比（-100~100）
    uint8_t enabled; // 使能标志
} IRTrackState_t;

// 初始化状态与参数
void IRFollow_Init(void);

// 设置基础速度（-100~100）
void IRFollow_SetBaseSpeed(int speed);

// 使能/禁用模块
void IRFollow_Enable(uint8_t enable);

// 主循环：读取红外 -> 决策速度 -> 姿态更新 -> 节流发送位姿
void IRTrack_Loop(void);

#endif // IR_FOLLOW_H
