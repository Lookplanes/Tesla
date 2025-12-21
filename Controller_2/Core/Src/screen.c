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
#include "ir_follow_screen.h"
#include <string.h>
#include <stdio.h>

uint8_t mode = 0; //写注释

char DATA_TO_SEND[32];
uint8_t bt_connected=1;

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
                                IR_Follow_Display();
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
                            if(tp_dev.x[0] >= 170 && tp_dev.x[0] <= 250 &&
                               tp_dev.y[0] >= 5   && tp_dev.y[0] <= 35) {
                                mode = 0;
                                IR_Follow_Stop();
                                Display_ShowMainMenu();
                            }
                            break;
                        
                }
                // 清除
                
            }
        } else delay_ms(10);

        // 在红外跟随模式下，解析串口并更新轨迹
        if (mode == 2) {
            IR_Follow_Handle();
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
    LCD_Fill(10, 10, 160, 26, CYAN);
    POINT_COLOR = BLUE;
    if (connected)
        LCD_ShowString(10, 10, 150, 16, 16, (uint8_t*)"connected");
    else
        LCD_ShowString(10, 10, 150, 16, 16, (uint8_t*)"connecting...");
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
