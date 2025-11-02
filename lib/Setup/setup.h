#ifndef FORMULA_SETUP_H
#define FORMULA_SETUP_H


#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <pins.h>
#include <sensorsSetup.h>
#include <variables.h>
#include <vector>
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
#include <timeSetup.h>
#include <max6675.h>
#include <Formula_RPM_LED.h>

extern TFT_eSPI tft;


template <typename T>
void displayWrite(T text, __u8 size, __u16 x, __u16 y, __u32 color);
__u16 displayRGB(__u8 r, __u8 g, __u8 b);
__u16 displayHEX(const char* hexcode);

extern __u16 bg_color;

const float FtC = 0.55;

extern __u8 actual_screen;

extern __u16 rpm_led;

void mainScreen();
void screen2();
void screen3();
void debugScreen();
void setupScreen();

String getTimeHMS();

void sendCANMessage(uint8_t id, uint8_t *data, uint8_t dlc);

void Calibracao(void *parameter);
void ScreenManager(void *parameter);
void refreshRateTask(void *parameter);
void CAN_receiveTask(void *parameter);
void temperatureTask(void *parameter);
void disableBluetooth();

void displaySetScreen(uint8_t id);

void CAN_setSensor(const __u8 *canData, __u8 canPacketSize,__u32 canId);

bool checkButton(uint8_t pin);

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
void fn_Data_10(__u8 data[DATA_10_DLC]);
void fn_Buffer_Ack(__u8 data[BUFFER_ACK_DLC]);
void fn_timeSet(__u8 data[TIMESET_DLC]);
void fn_RPM(__u8 data[RPM_DLC]);
void fn_ACC(__u8 data[ACC_DLC]);
void fn_GYRO(__u8 data[GYRO_DLC]);
void fn_Debug(__u8 data[DEBUG_DLC]);
void sensor_ErrorHandler(__u8 data[ERROR_CHECK_DLC]);


void fn_Group_0(__u8 data[GROUP0_DLC]);
void fn_Group_1(__u8 data[GROUP1_DLC]);
void fn_Group_2(__u8 data[GROUP2_DLC]);
void fn_Group_3(__u8 data[GROUP3_DLC]);
void fn_Group_7(__u8 data[GROUP7_DLC]);
void fn_Group_8(__u8 data[GROUP8_DLC]);
void fn_Group_9(__u8 data[GROUP9_DLC]);
void fn_Group_15(__u8 data[GROUP15_DLC]);

float readTempC(MAX6675 * Sensor);
uint16_t floattoU16(float value, uint8_t precision_bits);

void init_twai();




template <typename T>
void sensorUpdate(T value, __u8 index);

#define D_WIFI true

#define DISPLAY_TIMER 10
#define TEMPERATURE_TIMER 200
#define CALIBRACAO_TIMER 500
#define SD_TASK_TIMER CAN_TASK_TIMER

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

#define TIMEBASE 100
#define BUFFER_LENGTH TIMEBASE/SD_TASK_TIMER
#define BUFFER_NUMBER 2
#define MAX_SENSORS 55
#define BUFFER_SIZE 7

#define BTN_RETURN GPIO_NUM_36
#define BTN_LEFT GPIO_NUM_39
#define BTN_RIGHT GPIO_NUM_34
#define BTN_SELECT GPIO_NUM_35

#define V_SO GPIO_NUM_19
#define V_CLK GPIO_NUM_18
#define CS_TEMP1 GPIO_NUM_4
#define CS_TEMP2 GPIO_NUM_16
#define CS_TEMP3 GPIO_NUM_17
#define CS_TEMP4 GPIO_NUM_15
#define CS_TEMP5 GPIO_NUM_2
#define CS_TEMP6 GPIO_NUM_0

#define RPM_LED_PIN GPIO_NUM_5

#define REFRESH_RATE 30
#define REFRESH_TIMER 1000/REFRESH_RATE

const uint8_t mainScreen_ID = 1;
const uint8_t screen2_ID = 2;
const uint8_t screen3_ID = 3;

const uint8_t screens[]={
    mainScreen_ID,
    screen2_ID,
    screen3_ID
};


const uint8_t setupScreen_ID = 10;
const uint8_t debugScreen_ID = 255;

const uint8_t font_size_const_y = 7;
const uint8_t font_size_const_x = 4;



#endif