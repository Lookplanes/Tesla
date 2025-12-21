#include "ir_follow_screen.h"
#include "lcd.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ============ 内部状态变量 ============

// 当前世界视窗边长
static float world_size_cm = AREA_INIT_SIZE_CM;

// 世界坐标系的偏移（用来支持负坐标）
// world_min_x 到 world_min_x + world_size_cm 就是当前显示范围
static float world_min_x = 0.0f;
static float world_min_y = 0.0f;


// 轨迹缓冲：同时保存世界坐标
static float ir_x_cm[IR_TRAIL_MAX_POINTS];
static float ir_y_cm[IR_TRAIL_MAX_POINTS];
// static uint16_t ir_px[IR_TRAIL_MAX_POINTS];
// static uint16_t ir_py[IR_TRAIL_MAX_POINTS];
static uint16_t ir_count = 0;

// 调试计数器
static uint32_t ir_rx_line_count = 0;

// 记录最后解析成功的数据用于调试
static float last_parsed_x = 0.0f;
static float last_parsed_y = 0.0f;
static uint32_t last_parse_time = 0;

// ============ 外部接口（来自 stm32f1xx_it.c） ============
extern UART_HandleTypeDef huart2;
extern void IR_StartReceive(void);
extern void IR_StopReceive(void);
extern int IR_GetChar(void);
extern uint32_t IR_GetRxTotal(void);

// ============ 内部辅助函数 ============

/**
 * @brief 将 cm 坐标映射到屏幕像素（世界坐标可以是任意值，映射相对于 world_min_x/y）
 */
static void IR_MapToCanvas(float x_cm, float y_cm, uint16_t* out_x, uint16_t* out_y)
{
    // 相对于当前世界视窗左下角的坐标
    float rel_x = x_cm - world_min_x;
    float rel_y = y_cm - world_min_y;
    
    if (rel_x < 0) rel_x = 0;
    if (rel_x > world_size_cm) rel_x = world_size_cm;
    if (rel_y < 0) rel_y = 0;
    if (rel_y > world_size_cm) rel_y = world_size_cm;

    float k = (float)CANVAS_SIZE / world_size_cm; // 缩放比例：cm -> 像素

    uint16_t px = (uint16_t)(rel_x * k) + CANVAS_X0;
    uint16_t py = (uint16_t)(CANVAS_Y0 + CANVAS_SIZE - (rel_y * k));

    *out_x = px;
    *out_y = py;
}

/**
 * @brief 获取原点 (0,0) 在屏幕上的像素位置
 */
static void IR_GetOriginPixel(uint16_t* out_x, uint16_t* out_y)
{
    IR_MapToCanvas(0.0f, 0.0f, out_x, out_y);
}

/**
 * @brief 根据需要扩展世界范围，支持负坐标和四方向扩展
 * 逻辑：始终让原点 (0,0) 在视窗内，同时包含新点
 */
static int IR_EnsureWorldContains(float x, float y)
{
    int expanded = 0;
    float world_max_x = world_min_x + world_size_cm;
    float world_max_y = world_min_y + world_size_cm;
    
    // 检查点是否在当前视窗外 (加上 MAX_EXEED_SIZE_CM 作为缓冲)
    // while (x < world_min_x || x > world_max_x || y < world_min_y || y > world_max_y) {
    while (x < (world_min_x - MAX_EXEED_SIZE_CM) || x > (world_max_x + MAX_EXEED_SIZE_CM) ||
           y < (world_min_y - MAX_EXEED_SIZE_CM) || y > (world_max_y + MAX_EXEED_SIZE_CM)) {
        // 需要扩展，先翻倍 world_size_cm
        float current_world_center_x = world_min_x + world_size_cm / 2.0f;
        float current_world_center_y = world_min_y + world_size_cm / 2.0f;

        world_size_cm *= 2.0f;
        expanded = 1;

        float mid_x = (0.0f + x) / 2.0f; 
        float mid_y = (0.0f + y) / 2.0f; 
        
        // 以中点为中心，计算新的左下角坐标
        world_min_x = mid_x - (world_size_cm / 2.0f);
        world_min_y = mid_y - (world_size_cm / 2.0f);
        
        // 更新当前的边界值
        world_max_x = world_min_x + world_size_cm;
        world_max_y = world_min_y + world_size_cm;
    }
    
    return expanded;
}

/**
 * @brief 按当前 world_size_cm 重新绘制整个轨迹
 */
static void IR_RedrawTrail(void)
{
    // 清除画布区域
    LCD_Fill(CANVAS_X0, CANVAS_Y0, CANVAS_X0 + CANVAS_SIZE, CANVAS_Y0 + CANVAS_SIZE, CYAN);
    // 重新画边框
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(CANVAS_X0, CANVAS_Y0, CANVAS_X0 + CANVAS_SIZE, CANVAS_Y0 + CANVAS_SIZE);

    // 绘制原点标记（更明显）
    uint16_t origin_x, origin_y;
    IR_GetOriginPixel(&origin_x, &origin_y);
    POINT_COLOR = GREEN;
    // 画一个大十字标记（从原点向四个方向各延伸 8 像素）
    LCD_DrawLine(origin_x - 8, origin_y, origin_x + 8, origin_y);
    LCD_DrawLine(origin_x, origin_y - 8, origin_x, origin_y + 8);
    // 画一个圆形矩形框突出原点
    LCD_DrawRectangle(origin_x - 4, origin_y - 4, origin_x + 4, origin_y + 4);
    // 在中心画一个大圆点
    for (int i = 0; i < 3; i++) {
        TP_Draw_Big_Point(origin_x, origin_y, GREEN);
    }

    if (ir_count == 0) return;

    uint16_t px_prev, py_prev, px, py;

    // 用当前 world_size_cm 映射第一个点
    IR_MapToCanvas(ir_x_cm[0], ir_y_cm[0], &px_prev, &py_prev);
    TP_Draw_Big_Point(px_prev, py_prev, BLUE);

    for (uint16_t i = 1; i < ir_count; i++) {
        IR_MapToCanvas(ir_x_cm[i], ir_y_cm[i], &px, &py);
        LCD_DrawLine(px_prev, py_prev, px, py);
        px_prev = px;
        py_prev = py;
    }
}


/**
 * @brief 从中断缓冲区读取并解析 "$POSI,int_x_ccm,int_y_ccm\n"
 */
static void IR_ParseFromBuffer(void)
{
    static char line[64];
    static uint8_t len = 0;
    
    int ch;
    while ((ch = IR_GetChar()) >= 0) {
        if (ch == '\n' || ch == '\r') {
            if (len > 0) {
                line[len] = '\0';
                ir_rx_line_count++; 

                float x = 0.0f, y = 0.0f;
                int parsed_ok = 0;

                // 1. 解析协议
                if (strncmp(line, "$POSI,", 6) == 0) {
                    char *p = line + 6;
                    char *comma = strchr(p, ',');
                    if (comma) {
                        *comma = '\0';
                        long x_ccm = atol(p);
                        long y_ccm = atol(comma + 1);
                        x = (float)x_ccm / 100.0f; // 换算为 cm
                        y = (float)y_ccm / 100.0f;
                        parsed_ok = 1;
                    }
                }

                if (parsed_ok) {
                    last_parsed_x = x;
                    last_parsed_y = y;
                    last_parse_time++;

                    int needs_redraw = 0;

                    // 2. 如果移动距离小于 MINE_DISTANCE_CM 则忽略此点不进入记录
                    if (ir_count > 0) {
                        float dx = x - ir_x_cm[ir_count - 1];
                        float dy = y - ir_y_cm[ir_count - 1];
                        if ((dx * dx + dy * dy) < (MINE_DISTANCE_CM * MINE_DISTANCE_CM)) {
                            goto skip_point;
                        }
                    }

                    // 3. 如果满了，抽稀
                    if (ir_count >= IR_TRAIL_MAX_POINTS) {
                        // 舍弃奇数点
                        for (uint16_t i = 0; i < IR_TRAIL_MAX_POINTS / 2; i++) {
                            ir_x_cm[i] = ir_x_cm[i * 2];
                            ir_y_cm[i] = ir_y_cm[i * 2];
                        }
                        ir_count = IR_TRAIL_MAX_POINTS / 2;
                        needs_redraw = 1; // 抽稀后连接关系变了，必须重绘
                    }

                    // 4. 视窗扩展检查
                    if (IR_EnsureWorldContains(x, y)) {
                        needs_redraw = 1;
                    }

                    // 5. 存储新点
                    ir_x_cm[ir_count] = x;
                    ir_y_cm[ir_count] = y;
                    ir_count++;

                    // 6. 绘图处理
                    if (needs_redraw) {
                        // 如果发生了【扩展】或【抽稀】，全量重绘
                        IR_RedrawTrail();
                    } else {
                        // 正常情况：增量绘制最后一条线段
                        if (ir_count >= 2) {
                            uint16_t px1, py1, px2, py2;
                            IR_MapToCanvas(ir_x_cm[ir_count - 2], ir_y_cm[ir_count - 2], &px1, &py1);
                            IR_MapToCanvas(ir_x_cm[ir_count - 1], ir_y_cm[ir_count - 1], &px2, &py2);
                            POINT_COLOR = BLACK; // 轨迹颜色
                            LCD_DrawLine(px1, py1, px2, py2);
                        } else if (ir_count == 1) {
                            // 第一个点画个点
                            uint16_t px, py;
                            IR_MapToCanvas(ir_x_cm[0], ir_y_cm[0], &px, &py);
                            TP_Draw_Big_Point(px, py, BLUE);
                        }
                    }
                }
            }
skip_point:
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)ch;
        } else {
            len = 0;
        }
    }
}


void IR_Follow_Init(void)
{
    // 重置所有状态
    world_size_cm = AREA_INIT_SIZE_CM;
    world_min_x = 0.0f;
    world_min_y = 0.0f;
    ir_count = 0;
    ir_rx_line_count = 0;
    last_parsed_x = 0.0f;
    last_parsed_y = 0.0f;
    last_parse_time = 0;
}

void IR_Follow_Display(void)
{
    // 清屏与标题
    LCD_Clear(CYAN);
    POINT_COLOR = RED;
    LCD_ShowString(40, 35, 200, 24, 24, (uint8_t*)"Infrared Follow");

    // 画布边框
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(CANVAS_X0, CANVAS_Y0, CANVAS_X0 + CANVAS_SIZE, CANVAS_Y0 + CANVAS_SIZE);
    
    // 提示与按钮
    POINT_COLOR = BLUE;

    LCD_ShowString(190, 10, 100, 24, 24, (uint8_t*)"Menu");
    POINT_COLOR = BLACK;

    // 初始化红外跟随系统
    IR_Follow_Init();

    // 先发送一些数据来激活双向通信
    char warmup[32];
    sprintf(warmup, "$100,%d#", 0);  // 发送一个停止命令作为预热
    HAL_UART_Transmit(&huart2, (uint8_t*)warmup, strlen(warmup), 100);
    HAL_Delay(200);
    
    // 启动中断接收
    IR_StartReceive();

    // 通知小车切换到 IR 跟随模式并启动
    char txbuf[32];
    sprintf(txbuf, "$200,%d#", 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
    HAL_Delay(100);
}

void IR_Follow_Handle(void)
{
    // 从中断缓冲区解析数据并绘制轨迹
    IR_ParseFromBuffer();
    
    // 每帧绘制原点标记
    uint16_t origin_x, origin_y;
    IR_GetOriginPixel(&origin_x, &origin_y);
    POINT_COLOR = GREEN;
    // 画原点大十字（更明显）
    LCD_DrawLine(origin_x - 6, origin_y, origin_x + 6, origin_y);
    LCD_DrawLine(origin_x, origin_y - 6, origin_x, origin_y + 6);
    TP_Draw_Big_Point(origin_x, origin_y, GREEN);
    
    // 显示统计信息与当前坐标
    char info1[128];
    char info2[128];
    
    // 第一行：世界大小 + 轨迹点数
    int world_size_int = (int)world_size_cm;
    uint32_t total_rx = IR_GetRxTotal();
    sprintf(info1, "WS:%dcm  Pts:%d  RX:%lu", world_size_int, ir_count, total_rx);
    LCD_Fill(10, 280, 310, 296, CYAN);
    POINT_COLOR = BLACK;
    LCD_ShowString(10, 280, 300, 16, 16, (uint8_t*)info1);

    // 第二行：当前小车坐标 和 原点位置
    if (ir_count > 0) {
        float cur_x = ir_x_cm[ir_count - 1];
        float cur_y = ir_y_cm[ir_count - 1];
        
        sprintf(info2, "X=%dcm  Y=%dcm; Origin:(%d,%d)", (int)cur_x, (int)cur_y, origin_x, origin_y);
        POINT_COLOR = BLUE;
    } else {
        sprintf(info2, "No data yet");
        POINT_COLOR = RED;
    }
    LCD_Fill(10, 300, 310, 316, CYAN);
    LCD_ShowString(10, 300, 300, 16, 16, (uint8_t*)info2);
}

void IR_Follow_Stop(void)
{
    // 停止中断接收
    IR_StopReceive();
    
    // 发送停止命令
    char txbuf[32];
    sprintf(txbuf, "$100,%d#", 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
    HAL_Delay(100);
}

uint16_t IR_Follow_GetTrailCount(void)
{
    return ir_count;
}

int IR_Follow_GetCurrentPos(float* out_x, float* out_y)
{
    if (ir_count > 0) {
        *out_x = ir_x_cm[ir_count - 1];
        *out_y = ir_y_cm[ir_count - 1];
        return 1;
    }
    return 0;
}

float IR_Follow_GetWorldSize(void)
{
    return world_size_cm;
}

void IR_Follow_ClearTrail(void)
{
    ir_count = 0;
    ir_rx_line_count = 0;
}
