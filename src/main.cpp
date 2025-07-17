#include <setup.h>

char sensorValues[BUFFER_NUMBER][BUFFER_LENGTH][MAX_SENSORS][BUFFER_SIZE];
uint32_t timeValues[BUFFER_NUMBER][BUFFER_LENGTH];

volatile bool buffer_write = 0;
volatile bool buffer_read;

uint8_t row_write = 0;
uint8_t row_read = 0;

Sensor Sensor1;
Sensor Sensor2;
Sensor Sensor3;
Sensor Sensor4;
Sensor Sensor5;
Sensor Sensor6;

const __u8 max_size = 7;
const __u8 factor2 = 3;
__u8 actual_screen;

TFT_eSPI tft = TFT_eSPI();

__u16 bg_color = displayRGB(14,3,51);

String printValues();

__u8 sensorLength;
TickType_t now;

bool string_flag;
String can_msg = "";

QueueHandle_t can_rx_queue;

void fn_Debug(__u8 data[DEBUG_DLC]);

bool debug_mode = 1;

void setup() {
  Serial.begin(115200);
  while(!Serial){}
  init_twai();
  disableBluetooth();
    if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  sensorLength = indexSetup();
  setSensorName();

  Serial.println("ILI9341 Test!"); 
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);

 
  tft.init();
  tft.setRotation(0);
 

  tft.fillScreen(bg_color);
  //actual_screen=screen1(0);

  xTaskCreatePinnedToCore(
      CAN_receiveTask, // Function
      "CAN RX Task",   // Name
      4096,            // Stack size
      NULL,            // Params
      3,               // Priority
      NULL,            // Task handle
      0                // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(
      Calibracao,    // Function to implement the task
      "Calibracao", // Name of the task
      2048,       // Stack size in words
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      1           // Core where the task should run (0 or 1)
  );

}

void loop() {
  //switchScreen(1,bg_color);
}

void sensorUpdate(float value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%.2f", value);
}

void sensorUpdate(uint8_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(uint16_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(uint32_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%d", value);
}

void sensorUpdate(uint64_t value, uint8_t index)
{
  snprintf(sensorValues[buffer_write][row_write][index], 7, "%llu", value);
}

void sensorUpdate(String value, uint8_t index)
{
  value.toCharArray(sensorValues[buffer_write][row_write][index], 7);
}


void CAN_receiveTask(void *parameter)
{
  twai_message_t message;
  uint32_t alerts;

  for (;;)
  {
    while (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK)
    {
      CAN_setSensor(message.data, message.data_length_code, message.identifier);
    }
  }
}

void disableBluetooth()
{
  btStop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
}

void CAN_setSensor(const __u8 *canData, __u8 canPacketSize, __u32 canId)
{
  __u8 pSize = canPacketSize;
  __u8 data[pSize];
  __u32 id = canId;
  memcpy(data, canData, pSize);

  switch (canId)
  {
  case MESSAGES_ID:
    fn_Messages(data);
    break;
  case DATA_01_ID:
    fn_Data_01(data);
    break;
  case DATA_02_ID:
    fn_Data_02(data);
    break;
  case DATA_03_ID:
    fn_Data_03(data);
    break;
  case DATA_04_ID:
    fn_Data_04(data);
    break;
  case DATA_05_ID:
    fn_Data_05(data);
    break;
  case DATA_06_ID:
    fn_Data_06(data);
    break;
  case DATA_07_ID:
    fn_Data_07(data);
    break;
  case DATA_08_ID:
    fn_Data_08(data);
    break;
  case DATA_09_ID:
    fn_Data_09(data);
    break;

  case BUFFER_ACK_ID:
    fn_Buffer_Ack(data);
    break;

  case DEBUG_ID:
    fn_Debug(data);
    break;

  default:
    Serial.print("ID: ");
    Serial.println(canId);
    Serial.println("Not Recognized");
    break;
  }
}

void fn_Messages(__u8 data[MESSAGES_DLC])
{
}

void fn_Data_01(__u8 data[DATA_01_DLC])
{
  __u16 r_vBat = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_intTemp = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_vRef = ((data[4] & 0x0F) << 8) + data[3];
  __u8 r_Gear = (data[4] >> 4) & 0x0F;

  float vBat = vBatSensor(r_vBat);
  float vRef = vRefSensor(r_vRef);
  String Gear = Gear_Pos(r_Gear);

  sensorUpdate(vBat, Voltage_Sensor.index);
  sensorUpdate(vRef, V_Ref_Sensor.index);
  sensorUpdate(Gear, Gear_Pos_Sens.index);

  /*Serial.print(r_Gear);
  Serial.print(" ");*/
  // Serial.println((xTaskGetTickCount() * 1000) / configTICK_RATE_HZ);
}

void fn_Data_02(__u8 data[DATA_02_DLC])
{
  __u16 r_Susp_FR = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_Susp_FL = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Susp_RR = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_Susp_RL = (data[5] << 4) + ((data[4] >> 4) & 0x0F);
  __u16 r_WheelAngle = ((data[7] & 0x0F) << 8) + data[6];

  float Susp_FR = suspSensor(r_Susp_FR);
  float Susp_FL = (r_Susp_FL);
  float Susp_RR = (r_Susp_RR);
  float Susp_RL = (r_Susp_RL);
  float WheelAngle = (r_WheelAngle);

  sensorUpdate(Susp_FR, Susp_Pos_FR_Sensor.index);
  sensorUpdate(Susp_FL, Susp_Pos_FL_Sensor.index);
  sensorUpdate(Susp_RR, Susp_Pos_RR_Sensor.index);
  sensorUpdate(Susp_RL, Susp_Pos_RL_Sensor.index);
  sensorUpdate(WheelAngle, SteerWheel_Pos_Sensor.index);
}

void fn_Data_03(__u8 data[DATA_03_DLC])
{

  __u16 r_Hall_FR = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_Hall_FL = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Hall_RR = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_Hall_RL = (data[5] << 4) + ((data[4] >> 4) & 0x0F);

  float Hall_FR = (r_Hall_FR);
  float Hall_FL = (r_Hall_FL);
  float Hall_RR = (r_Hall_RR);
  float Hall_RL = (r_Hall_RL);

  sensorUpdate(Hall_FR, Wheel_Spd_FR_Sensor.index);
  sensorUpdate(Hall_FL, Wheel_Spd_FL_Sensor.index);
  sensorUpdate(Hall_RR, Wheel_Spd_RR_Sensor.index);
  sensorUpdate(Hall_RL, Wheel_Spd_RL_Sensor.index);
}

void fn_Data_04(__u8 data[DATA_04_DLC])
{
}

void fn_Data_05(__u8 data[DATA_05_DLC])
{
}

void fn_Data_06(__u8 data[DATA_06_DLC])
{
  __u16 r_F_BrakelinePress = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_R_BrakelinePress = (data[5] << 4) + ((data[4] >> 4) & 0x0F);
  float F_BrakelinePress = (r_F_BrakelinePress);
  float R_BrakelinePress = (r_R_BrakelinePress);
  sensorUpdate(F_BrakelinePress, F_Brakeline_Pressure.index);
  sensorUpdate(R_BrakelinePress, R_Brakeline_Pressure.index);
}

void fn_Data_07(__u8 data[DATA_07_DLC])
{
}

void fn_Data_08(__u8 data[DATA_08_DLC])
{
}

void fn_Data_09(__u8 data[DATA_09_DLC])
{
}

void fn_Buffer_Ack(__u8 data[BUFFER_ACK_DLC])
{
  if (data[0]=='1'){
    timeValues[buffer_write][row_write] = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;
    row_write++;
    if (row_write >= BUFFER_LENGTH)
    {
      buffer_write = !buffer_write;
      row_write = 0;
    }
  }

}

void fn_Debug(__u8 data[DEBUG_DLC])
{
  if (!string_flag)
  {
    for (__u8 i = 0; i < DEBUG_DLC; i++)
    {
      if (data[i] == '$')
      {
        Serial.println(can_msg);
        string_flag = true;
        break;
      }
      else
      {
        can_msg += char(data[i]);
      }
    }
    if (string_flag)
    {
      can_msg = "";
      string_flag = false;
    }
  }
}

void init_twai()
{
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = CAN_TX_PIN,
      .rx_io = CAN_RX_PIN,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 5,
      .rx_queue_len = 5,
      .alerts_enabled = TWAI_ALERT_NONE,
      .clkout_divider = 0};

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }
}

void Calibracao(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CALIBRACAO_TIMER);
  for (;;)
  {
    debugScreen(0);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void
 switchScreen(bool direction, __u16 bg_color){
    switch (actual_screen){
        case 1:
            screen1(1);
            break;
        case 2:
            screen2(1);
            break;
        case 3:
            screen3(1);
            break;
    };

    if (direction){
        actual_screen++;
        if (actual_screen==4){
            actual_screen=1;
        }
    }
    else{
        actual_screen--;
        if (actual_screen==0){
            actual_screen=3;
        }
    }

    switch (actual_screen){
        case 0:
            debugScreen(0);
            break;
        case 1:
            screen1(0);
            break;
        case 2:
            screen2(0);
            break;
        case 3:
            screen3(0);
            break;
    };


}








// WRITE FUNCTIONS

__u16 displayRGB(__u8 r, __u8 g, __u8 b){
    __u16 red = (r >> 3) & 0x1F;   // Red: Shift by 3 to fit in 5 bits
    __u16 green = (g >> 2) & 0x3F; // Green: Shift by 2 to fit in 6 bits
    __u16 blue = (b >> 3) & 0x1F;  // Blue: Shift by 3 to fit in 5 bits
    return (blue << 11) | (green << 5) | red;
}

__u16 displayHEX(const char* hexcode) {
    if (hexcode[0] == '#') hexcode++;
    __u32 rgb = (uint32_t)strtol(hexcode, NULL, 16);
    return (((rgb >> 19) & 0x1F)) |(((rgb >> 10) & 0x3F) << 5)|(((rgb >> 3)  & 0x1F) << 11);
}

void writeCenterText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize/2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
    
}

void writeTopCenterText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize/2;
        factor_y=(factor2*textsize)-2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y-factor_y); // (text, x, y, font)
}

void writeBottomCenterText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize/2;
        factor_y=(factor2*textsize)-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}


void writeLeftText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= 2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}

void writeTopLeftText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= 2;
        factor_y=(factor2*textsize)-2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y-factor_y); // (text, x, y, font)
}

void writeBottomLeftText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= 1;
        factor_y=(factor2*textsize)-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(BL_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}


void writeRightText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}

void writeTopRightText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize-1;
        factor_y=(factor2*textsize)-2;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y-factor_y); // (text, x, y, font)
}

void writeBottomRightText(String value, __u8 textsize,const __u8 font_style, __u16 textcolor, __u16 x, __u16 y){
    const char * text = value.c_str();
    textsize = min(textsize,max_size);
    __u8 factor_x = 0;
    __u8 factor_y = 0;
    if (font_style==2){
        factor_x= textsize-1;
        factor_y=(factor2*textsize)-1;
    }
    tft.setTextFont(font_style);
    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(textcolor, bg_color); // Text color and background
    tft.setTextSize(textsize); // Scale 1 (default) to 7; doesn't affect all fonts
    tft.drawString(text,x+factor_x,y+factor_y); // (text, x, y, font)
}


__u8 screen1(bool clear){

    const char* text1 = "12000";
    __u8 size_1 = 7;
    __u8 font_1 = 1;
    __u16 pos_x_1 = tft.width()/2;
    __u16 pos_y_1 = 220;
    __u16 color_1 = displayRGB(255,255,255);

    const char* text2 = "RPM";
    __u8 size_2 = 2;
    __u8 font_2 = 1;
    __u16 pos_x_2 = tft.width()/2;
    __u16 pos_y_2 = tft.height();
    __u16 color_2 = displayRGB(255,255,255);

    if (!clear){
        writeBottomCenterText(text1,size_1,font_1,color_1,pos_x_1,pos_y_1);
        writeBottomCenterText(text2,size_2,font_2,color_2,pos_x_2,pos_y_2);
    }
    else{
        writeBottomCenterText(text1,size_1,font_1,bg_color,pos_x_1,pos_y_1);
        writeBottomCenterText(text2,size_2,font_2,bg_color,pos_x_2,pos_y_2);
    }
    return 1;
}

__u8 screen2(bool clear){
    
    const char* text1 = "Test 2";
    __u8 size_1 = 7;
    __u8 font_1 = 1;
    __u16 pos_x_1 = tft.width()/2;
    __u16 pos_y_1 = 220;
    __u16 color_1 = displayRGB(255,255,255);
 
    if (!clear){
        writeBottomCenterText(text1,size_1,font_1,color_1,pos_x_1,pos_y_1);
    }
    else{
        writeBottomCenterText(text1,size_1,font_1,bg_color,pos_x_1,pos_y_1);
    }
 

return 2;
}
__u8 screen3(bool clear){

    const char* text1 = "AAAAA";
    __u8 size_1 = 7;
    __u8 font_1 = 1;
    __u16 pos_x_1 = tft.width()/2;
    __u16 pos_y_1 = 220;
    __u16 color_1 = displayRGB(255,255,255);
 
    if (!clear){
        writeBottomCenterText(text1,size_1,font_1,color_1,pos_x_1,pos_y_1);
    }
    else{
        writeBottomCenterText(text1,size_1,font_1,bg_color,pos_x_1,pos_y_1);
    }

return 3;
}

__u8 debugScreen(bool clear){

    __u8 size_1 = 4;
    __u8 font_1 = 1;

    __u16 pos_x_1 = 0;
    __u16 pos_y_1 = 0;

    __u16 pos_x_2 = tft.width();
    __u16 pos_y_2 = 0;

    __u16 pos_x_3 = 0;
    __u16 pos_y_3 = tft.height();

    __u16 pos_x_4 = tft.width();
    __u16 pos_y_4 = tft.height();

    __u16 pos_x_5 = tft.width()/2;
    __u16 pos_y_5 = tft.height()/2;




    __u16 color_1 = displayRGB(255,255,255);

    if (Sensor1.oldValue!="$"){
    writeTopLeftText(Sensor1.oldValue,size_1,font_1,bg_color,pos_x_1,pos_y_1);
    }
    if (Sensor2.oldValue!="$"){
    writeTopRightText(Sensor2.oldValue,size_1,font_1,bg_color,pos_x_2,pos_y_2);
    }
    if (Sensor3.oldValue!="$"){
    writeBottomLeftText(Sensor3.oldValue,size_1,font_1,bg_color,pos_x_3,pos_y_3);
    }
    if (Sensor4.oldValue!="$"){
    writeBottomRightText(Sensor4.oldValue,size_1,font_1,bg_color,pos_x_4,pos_y_4);
    }

    if (Sensor5.oldValue!="$"){
    writeBottomRightText(Sensor5.oldValue,size_1,font_1,bg_color,pos_x_5,pos_y_5);
    }

    Sensor1 = Wheel_Spd_FR_Sensor;
    Sensor2 = Wheel_Spd_FL_Sensor;
    Sensor3 = Wheel_Spd_RR_Sensor;
    Sensor4 = Wheel_Spd_RL_Sensor;
    Sensor5 = SteerWheel_Pos_Sensor; 
    bool print =0;
    uint8_t time=0;
    if (row_write==0){
      while(row_write==0&&time<5){
        vTaskDelay(pdMS_TO_TICKS(1));
        time++;
      }
    }
    if (row_write!=0){
      print =1;
    }
    if (print){
      char *value1 = sensorValues[buffer_write][row_write-1][Sensor1.index];
      char *value2 = sensorValues[buffer_write][row_write-1][Sensor2.index];
      char *value3 = sensorValues[buffer_write][row_write-1][Sensor3.index];
      char *value4 = sensorValues[buffer_write][row_write-1][Sensor4.index];
      char *value5 = sensorValues[buffer_write][row_write-1][Sensor5.index];

      if (value1[0]!='\0'){
        Sensor1.oldValue = value1;
      }
      if (value2[0]!='\0'){
        Sensor2.oldValue = value2;
      }
      if (value3[0]!='\0'){
        Sensor3.oldValue = value3;
      }
      if (value4[0]!='\0'){
        Sensor4.oldValue = value4;
      }
      if (value5[0]!='\0'){
        Sensor5.oldValue = value5;
      }
        writeTopLeftText(Sensor1.oldValue,size_1,font_1,color_1,pos_x_1,pos_y_1);
        writeTopRightText(Sensor2.oldValue,size_1,font_1,color_1,pos_x_2,pos_y_2);
        writeBottomLeftText(Sensor3.oldValue,size_1,font_1,color_1,pos_x_3,pos_y_3);
        writeBottomRightText(Sensor4.oldValue,size_1,font_1,color_1,pos_x_4,pos_y_4);
        writeCenterText(Sensor5.oldValue,size_1,font_1,color_1,pos_x_5,pos_y_5);

  }

return 0;
}
