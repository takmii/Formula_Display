#ifndef FORMULA_SETUP_H
#define FORMULA_SETUP_H



#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
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

void writeCenterText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeTopCenterText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeBottomCenterText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeLeftText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeTopLeftText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeBottomLeftText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeRightText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeTopRightText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);
void writeBottomRightText(const char * text, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y);

extern __u8 actual_screen;

void switchScreen(bool direction, __u16 bg_color);
__u8 screen1(bool clear);
__u8 screen2(bool clear);
__u8 screen3(bool clear);


#endif