#ifndef __IR_FOLLOW_SCREEN_H
#define __IR_FOLLOW_SCREEN_H

#include <stdint.h>

// ============ 画布配置 ============
#ifndef CANVAS_X0
#define CANVAS_X0  20
#endif
#ifndef CANVAS_Y0
#define CANVAS_Y0  60
#endif
#ifndef CANVAS_SIZE
#define CANVAS_SIZE 200
#endif

// ============ 轨迹缓冲配置 ============
#define IR_TRAIL_MAX_POINTS 800
#define MINE_DISTANCE_CM 0.5f

// ============ 初始显示范围 ============
#define AREA_INIT_SIZE_CM 200.0f

#define MAX_EXEED_SIZE_CM 10.0f

// ============ 公共接口函数 ============

/**
 * @brief 初始化红外跟随显示系统
 */
void IR_Follow_Init(void);

/**
 * @brief 显示红外跟随页面
 */
void IR_Follow_Display(void);

/**
 * @brief 处理红外跟随逻辑（解析数据、绘制轨迹、显示信息）
 * 应在主循环中每帧调用
 */
void IR_Follow_Handle(void);

/**
 * @brief 停止红外跟随
 */
void IR_Follow_Stop(void);

/**
 * @brief 获取当前轨迹点数
 */
uint16_t IR_Follow_GetTrailCount(void);

/**
 * @brief 获取当前最新坐标（cm）
 * @param out_x: 输出 X 坐标指针
 * @param out_y: 输出 Y 坐标指针
 * @return 1 if valid data exists, 0 otherwise
 */
int IR_Follow_GetCurrentPos(float* out_x, float* out_y);

/**
 * @brief 获取当前世界显示范围大小（cm）
 */
float IR_Follow_GetWorldSize(void);

/**
 * @brief 清空所有轨迹数据（重新开始）
 */
void IR_Follow_ClearTrail(void);

#endif // __IR_FOLLOW_SCREEN_H
