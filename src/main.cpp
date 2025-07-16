#include <setup.h>

__u16 bg_color = displayRGB(14,3,51);

void setup() {
  Serial.begin(115200);
  while(!Serial){};
  

  Serial.println("ILI9341 Test!"); 
 
  tft.init();
  tft.setRotation(0);
 

  tft.fillScreen(bg_color);
  actual_screen=screen1(0);

  //tft.drawLine(0,tft.height()/2,tft.width(),tft.height()/2,displayRGB(255,255,255));
  //tft.drawLine(tft.width()/2,0,tft.width()/2,tft.height(),displayRGB(255,255,255));

  
}

void loop() {
  switchScreen(1,bg_color);
  delay(100);
 
}



