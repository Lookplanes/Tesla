#include "stm32f1xx_hal.h"
#include "screen.h"
#include "lcd.h"
#include "key.h"
#include "touch.h"
#include "led.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "route_planning.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t mode = 0; //写注释

char DATA_TO_SEND[32];
uint8_t bt_connected=1;

// === Infrared follow 页面与轨迹显示 ===
#ifndef CANVAS_X0
#define CANVAS_X0  20
#endif
#ifndef CANVAS_Y0
#define CANVAS_Y0  60
#endif
#ifndef CANVAS_SIZE
#define CANVAS_SIZE 200
#endif

// 2m x 2m 区域映射到 CANVAS_SIZE 像素（左下为 (0,0)，右上为 (2m,2m)），SU7 坐标单位为 cm
#define AREA_SIZE_CM 200.0f

// 轨迹缓冲
#define IR_TRAIL_MAX_POINTS 400
static uint16_t ir_px[IR_TRAIL_MAX_POINTS];
static uint16_t ir_py[IR_TRAIL_MAX_POINTS];
static uint16_t ir_count = 0;

// === USART2 接收调试（用于验证是否收到 SU7 数据） ===
static uint32_t ir_rx_bytes = 0;
static uint32_t ir_rx_lines = 0;
static uint32_t ir_rx_last_tick = 0;
static char ir_last_line[64] = {0};

extern UART_HandleTypeDef huart2; // 用于接收 SU7 发送的 $POS,x,y

// 将 cm 坐标映射到屏幕像素（原点在画布左下角，y 轴向上）
static void IR_MapToCanvas(float x_cm, float y_cm, uint16_t* out_x, uint16_t* out_y)
{
    if (x_cm < 0) x_cm = 0; if (x_cm > AREA_SIZE_CM) x_cm = AREA_SIZE_CM;
    if (y_cm < 0) y_cm = 0; if (y_cm > AREA_SIZE_CM) y_cm = AREA_SIZE_CM;
    uint16_t px = (uint16_t)(x_cm / AREA_SIZE_CM * CANVAS_SIZE) + CANVAS_X0;
    uint16_t py = (uint16_t)(CANVAS_Y0 + CANVAS_SIZE - (y_cm / AREA_SIZE_CM * CANVAS_SIZE));
    *out_x = px; *out_y = py;
}

static void IR_Debug_DrawRxStatus(void)
{
    static uint32_t last_draw_tick = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_draw_tick) < 200) return;
    last_draw_tick = now;

    // 底部区域显示，不影响画布与按钮
    LCD_Fill(10, 250, 310, 279, CYAN);
    POINT_COLOR = BLACK;

    char l1[64];
    char l2[64];
    uint32_t age = (ir_rx_last_tick == 0) ? 0xFFFFFFFFu : (now - ir_rx_last_tick);
    snprintf(l1, sizeof(l1), "RX bytes:%lu lines:%lu age:%lums", (unsigned long)ir_rx_bytes, (unsigned long)ir_rx_lines, (unsigned long)age);
    snprintf(l2, sizeof(l2), "Last:%s", ir_last_line[0] ? ir_last_line : "(none)");

    LCD_ShowString(10, 250, 300, 16, 16, (uint8_t*)l1);
    LCD_ShowString(10, 265, 300, 16, 16, (uint8_t*)l2);
}

// 接收 "$POS,x,y\n"（浮点）或 "$POSI,x_ccm,y_ccm\n"（整数，0.01cm）
static void IR_UartPollAndParse(void)
{
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
        ir_rx_bytes++;
        ir_rx_last_tick = HAL_GetTick();
        static char line[64];
        static uint8_t len = 0;
        if (ch == '\n' || ch == '\r') {
            if (len > 0) {
                line[len] = '\0';
                // 记录原始行（用于调试显示）
                strncpy(ir_last_line, line, sizeof(ir_last_line) - 1);
                ir_last_line[sizeof(ir_last_line) - 1] = '\0';
                ir_rx_lines++;
                float x = 0, y = 0;
                uint8_t ok = 0;

                if (strncmp(line, "$POS,", 5) == 0) {
                    // 浮点协议：$POS,x,y
                    char* p = line + 5;
                    char* comma = strchr(p, ',');
                    if (comma) {
                        *comma = '\0';
                        char* end1 = NULL;
                        char* end2 = NULL;
                        x = strtof(p, &end1);
                        y = strtof(comma + 1, &end2);
                        ok = (end1 != p) && (end2 != (comma + 1));
                    }
                } else if (strncmp(line, "$POSI,", 6) == 0) {
                    // 整数协议：$POSI,x_ccm,y_ccm（0.01cm）
                    char* p = line + 6;
                    char* comma = strchr(p, ',');
                    if (comma) {
                        *comma = '\0';
                        char* end1 = NULL;
                        char* end2 = NULL;
                        long x_ccm = strtol(p, &end1, 10);
                        long y_ccm = strtol(comma + 1, &end2, 10);
                        if (end1 != p && end2 != (comma + 1)) {
                            x = (float)x_ccm / 100.0f;
                            y = (float)y_ccm / 100.0f;
                            ok = 1;
                        }
                    }
                }

                if (ok) {
                    uint16_t px, py;
                    IR_MapToCanvas(x, y, &px, &py);
                    if (ir_count < IR_TRAIL_MAX_POINTS) {
                        // 画轨迹：连接前后两个点
                        if (ir_count > 0) {
                            LCD_DrawLine(ir_px[ir_count - 1], ir_py[ir_count - 1], px, py);
                        } else {
                            TP_Draw_Big_Point(px, py, BLUE);
                        }
                        ir_px[ir_count] = px;
                        ir_py[ir_count] = py;
                        ir_count++;
                    }
                }
            }
            len = 0; // 重置缓冲
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)ch;
        } else {
            // 溢出，重置
            len = 0;
        }
    }
}

static void Display_InfraFollow(void)
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
    LCD_ShowString(20, 280, 120, 24, 24, (uint8_t*)"Menu");
    POINT_COLOR = BLACK;
    // 重置轨迹缓冲
    ir_count = 0;

    // 重置接收调试信息
    ir_rx_bytes = 0;
    ir_rx_lines = 0;
    ir_rx_last_tick = 0;
    ir_last_line[0] = '\0';

    // 通知小车切换到 IR 跟随模式并启动
    char txbuf[32];
    sprintf(txbuf, "$200,%d#", 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
    HAL_Delay(100);
}

static void InfraFollow_Handle(void)
{
    // 轮询接收并绘制轨迹
    IR_UartPollAndParse();
    IR_Debug_DrawRxStatus();
}

//触控操作
////////////////////////////////////////////////////////////////////////////////
//5个触控点的颜�?(电容触摸屏用)
const u16 POINT_COLOR_TBL[5]={RED,GREEN,BLUE,BROWN,GRED};
//电阻触摸屏测试函�?
void rtp_test(void)
{
    u8 key;
    u8 i=0;
    static uint8_t last_bt_connected = 0xFF; // 初始为无效值，确保第一次必刷新
    while(1)
    {
        key=KEY_Scan(0);
        tp_dev.scan(0);
        //显示蓝牙连接状态
        uint8_t bt_connected =(HAL_GPIO_ReadPin(STATE_GPIO_Port, STATE_Pin) == GPIO_PIN_SET);
        // if(bt_connected != last_bt_connected) {
        //     Display_ConnectionStatus(bt_connected);
        //     last_bt_connected = bt_connected;
        // }
        Display_ConnectionStatus(bt_connected);
        if(tp_dev.sta & TP_PRES_DOWN)
        {
            if(tp_dev.x[0] < lcddev.width && tp_dev.y[0] < lcddev.height)
            {
                switch(mode)
                {
                    case 0: // menu
                        //触摸屏逻辑
                        if(tp_dev.x[0] > (lcddev.width-24) && tp_dev.y[0] < 16){
                                screen_print();
                            }
                            // 路线规划菜单按钮区域
                            else if(tp_dev.x[0] >= 30 && tp_dev.x[0] <= 200 && tp_dev.y[0] >= 100 && tp_dev.y[0] <= 130){
                                mode=1;
                                Display_RoutePlanning();
                                RoutePlanning_Init();
                            }
                            // 红外跟随菜单按钮区域
                            else if(tp_dev.x[0] >= 30 && tp_dev.x[0] <= 200 && tp_dev.y[0] >= 150 && tp_dev.y[0] <= 180){
                                mode = 2;
                                Display_InfraFollow();
                            }
                            // SD卡存储菜单按钮区域
                            else if(tp_dev.x[0] >= 30 && tp_dev.x[0] <= 200 && tp_dev.y[0] >= 200 && tp_dev.y[0] <= 230){
                                Display_ShowSubMenu("SD card storage");
                            }
                        break;
                    case 1: // route planning
                            //画布区域
                            if(tp_dev.x[0] >= CANVAS_X0 && tp_dev.x[0] <= CANVAS_X0 + CANVAS_SIZE &&
                            tp_dev.y[0] >= CANVAS_Y0 && tp_dev.y[0] <= CANVAS_Y0 + CANVAS_SIZE) {
                                // 在画布上画点或记录路径
                                TP_Draw_Big_Point(tp_dev.x[0], tp_dev.y[0], RED);
                                // 你可以在这里添加路径点记录逻辑
                                RoutePlanning_AddPoint(tp_dev.x[0], tp_dev.y[0]);
                            }
                            // Clear 按钮区域
                            else if(tp_dev.x[0] >= 20 && tp_dev.x[0] <= 100 &&
                                    tp_dev.y[0] >= 280 && tp_dev.y[0] <= 304) {
                                 LCD_Fill(CANVAS_X0, CANVAS_Y0, CANVAS_X0 + CANVAS_SIZE, CANVAS_Y0 + CANVAS_SIZE, CYAN);
                                // 重新画边框
                                POINT_COLOR = BLACK;
                                LCD_DrawRectangle(CANVAS_X0, CANVAS_Y0, CANVAS_X0 + CANVAS_SIZE, CANVAS_Y0 + CANVAS_SIZE);
                                RoutePlanning_Clear();
                            }
                            // Start 按钮区域
                            else if(tp_dev.x[0] >= 100 && tp_dev.x[0] <= 180 &&
                                    tp_dev.y[0] >= 280 && tp_dev.y[0] <= 304) {

                                // Display_ShowMessage("Start Route!"); // 显示提示
                                RoutePlanning_FitByAngle();
                                RoutePlanning_Execute();
                                // char buf[32];
                                // int count = RoutePlanning_GetCount();
                                // sprintf(buf, "Points: %d", count);
                                // Display_ShowMessage(buf); // 显示路线点数

                                // for(int i = 0; i < count; i++) {
                                // RoutePoint pt = RoutePlanning_GetFitPoint(i);
                                // uint16_t px = (uint16_t)(pt.x / AREA_SIZE_M * CANVAS_SIZE) + CANVAS_X0;
                                // uint16_t py = (uint16_t)(CANVAS_Y0 + CANVAS_SIZE - (pt.y / AREA_SIZE_M * CANVAS_SIZE));
                                // TP_Draw_Big_Point(px, py, BLUE); // 用蓝色画出所有点
                                // }
                            }
                            
                            // Menu 按钮区域
                            else if(tp_dev.x[0] >= 180 && tp_dev.x[0] <= 280 &&
                                    tp_dev.y[0] >= 280 && tp_dev.y[0] <= 304) {
                                mode = 0;
                                Display_ShowMainMenu();
                            }
                            break;
                    case 2: // infrared follow
                            // Menu 按钮区域
                            if(tp_dev.x[0] >= 20 && tp_dev.x[0] <= 140 && tp_dev.y[0] >= 280 && tp_dev.y[0] <= 304) {
                                mode = 0;
                                Display_ShowMainMenu();
                                char txbuf[32];
                                sprintf(txbuf, "$100,%d#", 0);
                                HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
                                HAL_Delay(100);
                            }
                            break;
                        
                }
                // 清除
                
            }
        } else delay_ms(10);

        // 在红外跟随模式下，解析串口并更新轨迹
        if (mode == 2) {
            InfraFollow_Handle();
        }

        if(key == KEY0_PRES)
        {
            LCD_Clear(WHITE);
            TP_Adjust();
            TP_Save_Adjdata();
            Load_Drow_Dialog();
        }
        i++;
        if(i%20==0)LED0=!LED0;
    }
}

// 显示连接状态
void Display_ConnectionStatus(uint8_t connected)
{
    LCD_Fill(10, 10, 210, 26, CYAN); // 210=10+200, 26=10+16
    POINT_COLOR = BLUE;
    if (connected)
        LCD_ShowString(10, 10, 200, 16, 16, (uint8_t*)"connected");
    else
        LCD_ShowString(10, 10, 200, 16, 16, (uint8_t*)"connecting...");
}

// 显示主菜单
void Display_ShowMainMenu(void)
{
    LCD_Clear(CYAN);
    Display_ConnectionStatus(bt_connected);
    char buf[32];

    // 显示大标题
    POINT_COLOR = RED;
    LCD_ShowString(90, 50, 200, 24, 24, (uint8_t*)"Menu");

    // 路线规划
    POINT_COLOR = WHITE;
    LCD_Fill(30, 100, 200, 130, YELLOW); // 按钮底色
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(30, 100, 200, 130);
    sprintf(buf, "route planning");
    LCD_ShowString(60, 110, 160, 16, 16, (u8*)buf);
    // 红外跟随
    POINT_COLOR = WHITE;
    LCD_Fill(30, 150, 200, 180, RED);
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(30, 150, 200, 180);
    sprintf(buf, "infrared follow");
    LCD_ShowString(60, 160, 160, 16, 16, (u8*)buf);
    // SD卡存储
    POINT_COLOR = WHITE;
    LCD_Fill(30, 200, 200, 230, BLUE);
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(30, 200, 200, 230);
    sprintf(buf, "SD card storage");
    LCD_ShowString(60, 210, 160, 16, 16, (u8*)buf);
}

// 显示子菜单
void Display_ShowSubMenu(const char* title)
{
    LCD_Clear(CYAN);
    LCD_ShowString(20, 40, 200, 24, 24, (uint8_t*)title);
    LCD_ShowString(20, 200, 200, 24, 24, (uint8_t*)"Back");
}

void Display_RoutePlanning(void)
{
    LCD_Clear(CYAN);

    // 标题
    POINT_COLOR = RED;
    LCD_ShowString(40, 35, 200, 24, 24, (uint8_t*)"Route Planning");
    
    // 画布边框
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(
        CANVAS_X0,
        CANVAS_Y0,
        CANVAS_X0 + CANVAS_SIZE,
        CANVAS_Y0 + CANVAS_SIZE
    );

    // Back
    POINT_COLOR = YELLOW;
    LCD_ShowString(20, 280, 100, 24, 24, (uint8_t*)"Clear");
    POINT_COLOR = RED;
    LCD_ShowString(100, 280, 100, 24, 24, (uint8_t*)"Start");
    POINT_COLOR = BLUE;
    LCD_ShowString(180, 280, 100, 24, 24, (uint8_t*)"Menu");
}


// 显示提示信息
void Display_ShowMessage(const char* msg)
{
    LCD_ShowString(10, 220, 200, 24, 24, (uint8_t*)msg);
}
