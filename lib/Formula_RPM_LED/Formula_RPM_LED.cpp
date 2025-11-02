#include "Formula_RPM_LED.h"

void updateRPM(unsigned short RPM){
    const static unsigned short minValue=0;
    const static unsigned short maxValue=12000;
    const static unsigned short prop = (maxValue-minValue)/10;
    unsigned short RPM_Value = constrain(RPM, 0, maxValue);
    //static unsigned char n_test  = 0;
    uint8_t numLedsOn;
    numLedsOn=0;
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      if (RPM_Value >= minValue + (i) * prop) {
        numLedsOn = i + 1;
      }
      else{
        break;
      }
    }

  /*n_test++;
  if (n_test==100){
    n_test=0;
  }*/

    for (uint8_t i = 0; i < NUM_LEDS; i++) {
    if (i < numLedsOn) {
      if (i < 4)
        ledWriteColor(i,0,255,0); // Verde
      else if (i < 7)
        ledWriteColor(i,255,255,0); // Amarelo
      else
        ledWriteColor(i,255,0,0);// Vermelho
    } 
}
  pixels.show();
}

void ledWriteColor(uint8_t i,uint8_t r, uint8_t g,uint8_t b){
    pixels.setPixelColor(i, pixels.Color(r, g, b));
}

void ledTurnOff(){
  for (uint8_t i =0; i<NUM_LEDS; i++){
    pixels.setPixelColor(i, 0); // Desliga o LED
  }
}
