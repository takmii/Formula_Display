#ifndef FORMULA_SETUP_H
#define FORMULA_SETUP_H



#include <Arduino.h>
#include <TFT_eSPI.h>

#include <SPI.h>
#include <pins.h>
#include <sensorsSetup.h>
#include <variables.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include <WiFi.h>
#include "driver/twai.h"
#include <CAN_Messages.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_ILI9341.h>
#include <formulaDisplay.h>
#include <variables.h>

extern TFT_eSPI tft;


template <typename T>
void displayWrite(T text, __u8 size, __u16 x, __u16 y, __u32 color);
__u16 displayRGB(__u8 r, __u8 g, __u8 b);
__u16 displayHEX(const char* hexcode);

extern __u16 bg_color;

void writeCenterText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeTopCenterText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeBottomCenterText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeLeftText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeTopLeftText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeBottomLeftText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeRightText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeTopRightText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeBottomRightText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);

extern __u8 actual_screen;

void switchScreen(bool direction, __u16 bg_color);
__u8 screen1(bool clear);
__u8 screen2(bool clear);
__u8 screen3(bool clear);
__u8 debugScreen(bool clear);

void Calibracao(void *parameter);
void CAN_receiveTask(void *parameter);
void disableBluetooth();


void CAN_setSensor(const __u8 *canData, __u8 canPacketSize,__u32 canId);


void fn_Messages(__u8 data[MESSAGES_DLC]);
void fn_Data_01(__u8 data[DATA_01_DLC]);
void fn_Data_02(__u8 data[DATA_02_DLC]);
void fn_Data_03(__u8 data[DATA_03_DLC]);
void fn_Data_04(__u8 data[DATA_04_DLC]);
void fn_Data_05(__u8 data[DATA_05_DLC]);
void fn_Data_06(__u8 data[DATA_06_DLC]);
void fn_Data_07(__u8 data[DATA_07_DLC]);
void fn_Data_08(__u8 data[DATA_08_DLC]);
void fn_Data_09(__u8 data[DATA_09_DLC]);
void fn_Buffer_Ack(__u8 data[BUFFER_ACK_DLC]);
void fn_Debug(__u8 data[DEBUG_DLC]);

void init_twai();


template <typename T>
void sensorUpdate(T value, __u8 index);

#define D_WIFI true

#define DISPLAY_TIMER 10
#define CALIBRACAO_TIMER 500
#define SD_TASK_TIMER CAN_TASK_TIMER

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

#define TIMEBASE 100
#define BUFFER_LENGTH TIMEBASE/SD_TASK_TIMER
#define BUFFER_NUMBER 2
#define MAX_SENSORS 42
#define BUFFER_SIZE 7


#endif