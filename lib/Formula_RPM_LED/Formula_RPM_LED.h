#ifndef FORMULA_RPM_LED_H
#define FORMULA_RPM_LED_H

#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 10
#define BRIGHTNESS 255

extern Adafruit_NeoPixel pixels;

void updateRPM(unsigned short RPM);
void ledWriteColor(uint8_t i,uint8_t r, uint8_t g,uint8_t b);
void ledTurnOff();




#endif