#include <setup.h>

bool debug_mode = 1;

class Display
{
public:
  bool first_update = true;
  class DObj
  {
    String value = "$";
    String oldValue;
    bool isWritten;
    signed char factor_x = 0;
    signed char factor_y = 0;
    const unsigned char max_size = 7;
    const unsigned char factor2 = 3;
    unsigned char object_type = 0;
    unsigned char screen_id = 0;
    unsigned short x_length = 0;
    unsigned short y_length = 0;
    unsigned short x_old_length = 0;
    unsigned short y_old_length = 0;

  public:
    unsigned char size = 4;
    unsigned char font = 1;
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short length;
    unsigned short height;
    unsigned short color = displayRGB(255, 255, 255);
    unsigned char datum;

  public:
    DObj(unsigned short x_, unsigned short y_) : screen_id(Display::getInstance().getCurrentScreen()), pos_x(x_), pos_y(y_)
    {
      Display::getInstance().registerObject(this);
    }

    unsigned char getScreen() const
    {
      return screen_id;
    }

    static unsigned short displayRGB(unsigned char r, unsigned char g, unsigned char b)
    {
      unsigned short red = (r >> 3) & 0x1F;   // Red: Shift by 3 to fit in 5 bits
      unsigned short green = (g >> 2) & 0x3F; // Green: Shift by 2 to fit in 6 bits
      unsigned short blue = (b >> 3) & 0x1F;  // Blue: Shift by 3 to fit in 5 bits
      return (blue << 11) | (green << 5) | red;
    }

    static unsigned short displayHEX(const char *hexcode)
    {
      if (hexcode[0] == '#')
        hexcode++;
      unsigned int rgb = (unsigned int)strtol(hexcode, NULL, 16);
      return (((rgb >> 19) & 0x1F)) | (((rgb >> 10) & 0x3F) << 5) | (((rgb >> 3) & 0x1F) << 11);
    }

    void writeCenterText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = this->size / 2;
        this->factor_y = 0;
      }
      this->datum = MC_DATUM;
    }

    void writeTopCenterText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = this->size / 2;
        this->factor_y = -((this->factor2 * this->size) - 2);
      }
      this->datum = TC_DATUM;
    }

    void writeBottomCenterText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = this->size / 2;
        this->factor_y = (this->factor2 * this->size) - 1;
      }
      this->datum = BC_DATUM;
    }

    void writeLeftText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = 2;
        this->factor_y = 0;
      }
      this->datum = ML_DATUM;
    }

    void writeTopLeftText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = 2;
        this->factor_y = -((this->factor2 * this->size) - 2);
      }
      this->datum = TL_DATUM;
    }

    void writeBottomLeftText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = 1;
        this->factor_y = (this->factor2 * this->size) - 1;
      }
      this->datum = BL_DATUM;
    }

    void writeRightText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = this->size - 1;
        this->factor_y = 0;
      }
      this->datum = MR_DATUM;
    }

    void writeTopRightText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = this->size - 1;
        this->factor_y = -((this->factor2 * this->size) - 2);
      }
      this->datum = TR_DATUM;
    }

    void writeBottomRightText(String value)
    {
      this->object_type = 1;
      this->value = value;
      this->size = min(this->size, this->max_size);
      if (this->font == 2)
      {
        this->factor_x = this->size - 1;
        this->factor_y = (this->factor2 * this->size) - 1;
      }
      this->datum = BR_DATUM;
    }

    void drawXline(unsigned short length)
    {
      this->object_type = 2;
      this->x_length = length;
    }

    void drawYline(unsigned short length)
    {
      this->object_type = 3;
      this->y_length = length;
    }

    void refresh()
    {
      if (this->object_type == 1)
      {
        if (this->value != this->oldValue || Display::getInstance().first_update == true)
        {
          tft.setTextFont(this->font);
          tft.setTextDatum(this->datum);
          tft.setTextColor(bg_color, bg_color);                                                               // Text color and background
          tft.setTextSize(this->size);                                                                        // Scale 1 (default) to 7; doesn't affect all fonts
          tft.drawString(this->oldValue.c_str(), this->pos_x + this->factor_x, this->pos_y + this->factor_y); // (text, x, y, font)
          tft.setTextColor(this->color, bg_color);
          tft.drawString(this->value.c_str(), this->pos_x + this->factor_x, this->pos_y + this->factor_y); // (text, x, y, font)
          this->oldValue = this->value;
        }
      }
      else if (this->object_type == 2)
      {
        if (this->x_length != this->x_old_length || Display::getInstance().first_update == true)
        {
          tft.drawFastHLine(this->pos_x, this->pos_y, this->x_length, this->color);
        }
      }
      else if (this->object_type == 3)
      {
        if (this->y_length != this->y_old_length || Display::getInstance().first_update == true)
        {
          tft.drawFastVLine(this->pos_x, this->pos_y, this->y_length, this->color);
        }
      }
    }
  };

  void registerObject(DObj *obj)
  {
    objects.push_back(obj);
  }

  static Display &getInstance()
  {
    static Display instance;
    return instance;
  }

  static void refreshAll()
  {
    for (auto obj : getInstance().objects)
    {
      if (obj->getScreen() == getInstance().getCurrentScreen())
      {
        obj->refresh();
      }
    }
    if (getInstance().first_update == true)
    {
      getInstance().first_update = false;
    }
  }

  void setScreen(uint8_t screen)
  {
    tft.fillScreen(bg_color);
    current_screen = screen;
    getInstance().first_update = true;
  }

  uint8_t getCurrentScreen() const
  {
    return current_screen;
  }

private:
  unsigned char current_screen = 0;
  std::vector<DObj *> objects;
};

using DisplayObject = Display::DObj;

__u16 bg_color = DisplayObject::displayRGB(14, 3, 51);

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

__u8 actual_screen;

TFT_eSPI tft = TFT_eSPI();

bool timeSet = false;

String printValues();

__u8 sensorLength;
TickType_t now;

bool string_flag;
String can_msg = "";

QueueHandle_t can_rx_queue;

void fn_Debug(__u8 data[DEBUG_DLC]);

uint16_t yPrintln(const DisplayObject *obj);
uint16_t negyPrintln(const DisplayObject *obj);

MAX6675 s_BrakeTempFR(V_CLK, CS_TEMP1, V_SO);
MAX6675 s_BrakeTempFL(V_CLK, CS_TEMP2, V_SO);
MAX6675 s_BrakeTempRR(V_CLK, CS_TEMP3, V_SO);
MAX6675 s_BrakeTempRL(V_CLK, CS_TEMP4, V_SO);
MAX6675 s_FirewallTemp1(V_CLK, CS_TEMP5, V_SO);
MAX6675 s_FirewallTemp2(V_CLK, CS_TEMP6, V_SO);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }
  init_twai();
  disableBluetooth();
  pinMode(BTN_LEFT, INPUT);
  pinMode(BTN_RIGHT, INPUT);
  pinMode(BTN_RETURN, INPUT);
  pinMode(BTN_SELECT, INPUT);

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
      Calibracao,   // Function to implement the task
      "Calibracao", // Name of the task
      2048,         // Stack size in words
      NULL,         // Task input parameter
      1,            // Priority of the task
      NULL,         // Task handle
      1             // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      ScreenManager,    // Function to implement the task
      "Screen Manager", // Name of the task
      2048,             // Stack size in words
      NULL,             // Task input parameter
      1,                // Priority of the task
      NULL,             // Task handle
      1                 // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      refreshRateTask, // Function to implement the task
      "Refresh",       // Name of the task
      4096,            // Stack size in words
      NULL,            // Task input parameter
      2,               // Priority of the task
      NULL,            // Task handle
      1                // Core where the task should run (0 or 1)
  );

  xTaskCreatePinnedToCore(
      temperatureTask, // Function to implement the task
      "Temperature",   // Name of the task
      4096,            // Stack size in words
      NULL,            // Task input parameter
      1,               // Priority of the task
      NULL,            // Task handle
      1                // Core where the task should run (0 or 1)
  );

  if (!debug_mode)
  {
    displaySetScreen(mainScreen_ID);
  }
}

void loop()
{
  // switchScreen(1,bg_color);
}

void sensorUpdate(float value, uint8_t index)
{
  sensorIndex[index]->value = String(value, 2);
}

void sensorUpdate(uint8_t value, uint8_t index)
{
  sensorIndex[index]->value = String(value);
}

void sensorUpdate(uint16_t value, uint8_t index)
{
  sensorIndex[index]->value = String(value);
}

void sensorUpdate(uint32_t value, uint8_t index)
{
  sensorIndex[index]->value = String(value);
}

void sensorUpdate(uint64_t value, uint8_t index)
{
  sensorIndex[index]->value = String((uint64_t)value);
}

void sensorUpdate(int8_t value, uint8_t index)
{
  sensorIndex[index]->value = String(value);
}

void sensorUpdate(int16_t value, uint8_t index)
{
  sensorIndex[index]->value = String(value);
}

void sensorUpdate(int32_t value, uint8_t index)
{
  sensorIndex[index]->value = String(value);
}

void sensorUpdate(int64_t value, uint8_t index)
{
  sensorIndex[index]->value = String((int64_t)value);
}

void sensorUpdate(String value, uint8_t index)
{
  sensorIndex[index]->value = value;
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

  case DATA_10_ID:
    fn_Data_10(data);
    break;

  case BUFFER_ACK_ID:
    fn_Buffer_Ack(data);
    break;

  case DEBUG_ID:
    fn_Debug(data);
    break;

  /*case RPM_ID:
    fn_RPM(data);
    break;*/

  case ACC_ID:
    fn_ACC(data);
    break;

  case GYRO_ID:
    fn_GYRO(data);
    break;
  case TIMESET_ID:
    fn_timeSet(data);
    break;

  case BASE_ID + GROUP0_ID:
    fn_Group_0(data);
    break;

  case BASE_ID + GROUP1_ID:
    fn_Group_1(data);
    break;

  case BASE_ID + GROUP2_ID:
    fn_Group_2(data);
    break;

  case BASE_ID + GROUP3_ID:
    fn_Group_3(data);
    break;

  case BASE_ID + GROUP7_ID:
    fn_Group_7(data);
    break;

  case BASE_ID + GROUP8_ID:
    fn_Group_8(data);
    break;

  case BASE_ID + GROUP9_ID:
    fn_Group_9(data);
    break;

  case BASE_ID + GROUP15_ID:
    fn_Group_15(data);
    break;

  default:
    //Serial.print("ID: ");
    //Serial.println(canId);
    //Serial.println("Not Recognized");
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
  float intTemp = internalTemp(r_intTemp);
  String Gear = Gear_Pos(r_Gear);

  sensorUpdate(vBat, Voltage_Sensor.index);
  sensorUpdate(intTemp, Internal_Temperature_Sensor.index);
  sensorUpdate(vRef, V_Ref_Sensor.index);
  sensorUpdate(Gear, Gear_Pos_Sens.index);
}

void fn_Data_02(__u8 data[DATA_02_DLC])
{
  __u16 r_Susp_FR = ((data[1] & 0x0F) << 8) + data[0];
  __u16 r_Susp_FL = (data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_Susp_RR = ((data[4] & 0x0F) << 8) + data[3];
  __u16 r_Susp_RL = (data[5] << 4) + ((data[4] >> 4) & 0x0F);
  __u16 r_WheelAngle = ((data[7] & 0x0F) << 8) + data[6];

  float Susp_FR = suspSensor(r_Susp_FR);
  float Susp_FL = suspSensor(r_Susp_FL);
  float Susp_RR = suspSensor(r_Susp_RR);
  float Susp_RL = suspSensor(r_Susp_RL);
  String WheelAngle = SteeringWheel.steeringWheelValue(r_WheelAngle);

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
  //__u16 r_Oil_Pressure = ((data[1] & 0x0F) << 8) + data[0];
  //__u16 r_Oil_Temp = (data[2] << 4) + ((data[1] >> 4) & 0x0F);

  //float Oil_Pressure = (r_Oil_Pressure);
  // float Oil_Temp = (r_Oil_Temp);

  //sensorUpdate(Oil_Pressure, Oil_Pressure_Sensor.index);
  // sensorUpdate(Oil_Temp, Oil_Temperature_Sensor.index);
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

void fn_Data_10(__u8 data[DATA_10_DLC])
{
  static const double a = 0.00130654;
  static const double b = 0.000259714;
  static const double c = -1.08959E-08;

  __u16 r_MAP1=((data[1] & 0x0F) << 8) + data[0];
  __u16 r_MAP2=(data[2] << 4) + ((data[1] >> 4) & 0x0F);
  __u16 r_MAF=((data[4] & 0x0F) << 8) + data[3];
  __u16 r_OilTemp=(data[5] << 4) + ((data[4] >> 4) & 0x0F);

  float MAP1 = mapSensor(r_MAP1);
  float MAP2 = mapSensor(r_MAP2);
  float MAF = mafSensor(r_MAF);
  float OilTemp = tempOilSensor(r_OilTemp,a,b,c);

  sensorUpdate(MAP1,MAP1_Sensor.index);
  sensorUpdate(MAP2,MAP2_Sensor.index);
  sensorUpdate(MAF,MAF_Sensor.index);
  sensorUpdate(OilTemp,Oil_Temperature_Sensor.index);
}

/*void fn_RPM(__u8 data[RPM_DLC])
{
  uint16_t RPM = ((data[1] & 0x3F) << 8) | data[0];
  sensorUpdate(RPM, RPM_Sensor.index);
}*/

void fn_ACC(__u8 data[ACC_DLC])
{
  int16_t AccX = (int16_t)(data[1] << 8 | data[0]);
  int16_t AccY = (int16_t)(data[3] << 8 | data[2]);
  int16_t AccZ = (int16_t)(data[5] << 8 | data[4]);
  int16_t AccMod = (int16_t)(data[7] << 8 | data[6]);

  float aX = (float)AccX / 16384;
  float aY = (float)AccY / 16384;
  float aZ = (float)AccZ / 16384;
  float aMod = (float)AccMod / 16384;
  sensorUpdate(aX, Accel_X.index);
  sensorUpdate(aY, Accel_Y.index);
  sensorUpdate(aZ, Accel_Z.index);
  sensorUpdate(aMod, Accel.index);
}

void fn_GYRO(__u8 data[GYRO_DLC])
{
  int16_t GyX = (int16_t)(data[1] << 8 | data[0]);
  int16_t GyY = (int16_t)(data[3] << 8 | data[2]);
  int16_t GyZ = (int16_t)(data[5] << 8 | data[4]);

  float roll_rate = 1.3;
  float pitch_rate = -3.25;
  float yaw_rate = -1.12;
  float RateRoll, RatePitch, RateYaw;

  RateRoll = (float)GyX / 65.5;
  RatePitch = (float)GyY / 65.5;
  RateYaw = (float)GyZ / 65.5;

  RateRoll -= roll_rate;
  RatePitch -= pitch_rate;
  RateYaw -= yaw_rate;

  sensorUpdate(RateRoll, Gyro_X.index);
  sensorUpdate(RatePitch, Gyro_Y.index);
  sensorUpdate(RateYaw, Gyro_Z.index);
}

void fn_timeSet(__u8 data[TIMESET_DLC])
{
  uint8_t day = data[0] & 0x1F;
  uint8_t month = (data[0] >> 5) & 0x07;
  month |= (data[1] & 0x01) << 3;

  uint16_t year = (data[1] >> 1) & 0x7F;
  year |= ((data[2] & 0x1F) << 7);

  uint8_t hour = (data[2] >> 5) & 0x07;
  hour |= (data[3] & 0x03) << 3;

  uint8_t minute = (data[3] >> 2) & 0x3F;
  uint8_t second = data[4] & 0x3F;

  struct tm t;
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  t.tm_isdst = 0;
  time_t timeSinceEpoch = mktime(&t);
  struct timeval tv = {.tv_sec = timeSinceEpoch, .tv_usec = 0};
  settimeofday(&tv, nullptr);
  uint8_t ack[TIMESET_ACK_DLC] = {1};
  timeSet = true;
  sendCANMessage(TIMESET_ACK_ID, ack, TIMESET_ACK_DLC);
}

void fn_Buffer_Ack(__u8 data[BUFFER_ACK_DLC])
{
  if (data[0] == '1')
  {
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

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
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
    if (debug_mode)
    {
      debugScreen();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void ScreenManager(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(REFRESH_TIMER);
  static uint8_t screen_selector;
  static bool setup_mode = false;
  static const uint8_t num_screens = sizeof(screens) / sizeof(screens[0]);
  static bool pressed1 = false;
  static bool pressed2 = false;
  static bool pressed3 = false;
  static bool pressed4 = false;
  static uint8_t current_index = 0;
  for (;;)
  {

    if (!debug_mode)
    {

      screen_selector = Display::getInstance().getCurrentScreen();
      bool state1 = checkButton(BTN_RIGHT);
      bool state2 = checkButton(BTN_LEFT);
      bool state3 = checkButton(BTN_RETURN);
      bool state4 = checkButton(BTN_SELECT);

      if ((state1 && !pressed1) && (state2 && !pressed2))
      {
        if (!setup_mode)
        {
          setup_mode = true;
          pressed1 = true;
          pressed2 = true;
        }
      }

      if (state1 && !pressed1)
      {
        current_index = (current_index + 1) % num_screens;
        screen_selector = screens[current_index];
        pressed1 = true;
      }
      else if (!state1 && pressed1)
      {
        pressed1 = false;
      }

      if (state2 && !pressed2)
      {
        if (current_index == 0)
        {
          current_index = num_screens - 1;
        }
        else
        {
          current_index--;
        }
        screen_selector = screens[current_index];
        pressed2 = true;
      }
      else if (!state2 && pressed2)
      {
        pressed2 = false;
      }
      if (setup_mode)
      {
        setupScreen();
      }

      else
      {
        switch (screen_selector)
        {
        case mainScreen_ID:
          mainScreen();
          break;
        case screen2_ID:
          screen2();
          break;
        case screen3_ID:
          screen3();
          break;
        };
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

bool checkButton(uint8_t pin)
{
  return digitalRead(pin) == HIGH;
}

void refreshRateTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(REFRESH_TIMER);
  for (;;)
  {
    Display::refreshAll();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// WRITE FUNCTIONS

void mainScreen()
{
  displaySetScreen(mainScreen_ID);

  static DisplayObject Gear(tft.width() / 2, tft.height() / 2);
  Gear.size = 7;
  Gear.writeCenterText(Gear_Pos_Sens.value);

  static DisplayObject Gear_Text(tft.width() / 2, yPrintln(&Gear));
  Gear_Text.size = 2;
  Gear_Text.writeTopCenterText("GEAR");

  static DisplayObject RPM(0, 0);
  RPM.writeTopLeftText(RPM_Sensor.value);

  static DisplayObject RPM_Text(0, yPrintln(&RPM));
  RPM_Text.size = 2;
  RPM_Text.writeTopLeftText("RPM");

  static DisplayObject TimeHMS(tft.width(), 0);
  TimeHMS.size = 3;
  TimeHMS.writeTopRightText(getTimeHMS());

  static DisplayObject IntTemp(tft.width(), yPrintln(&TimeHMS));
  IntTemp.size = 2;
  IntTemp.writeTopRightText(Internal_Temperature_Sensor.value + "C");

  static DisplayObject Line1(0, 60);
  Line1.drawXline(tft.width());

  static DisplayObject Line2(0, tft.height() - 60);
  Line2.drawXline(tft.width());
}

void screen2()
{
  displaySetScreen(screen2_ID);
  static DisplayObject text2(tft.width() / 2, 220);
  text2.writeBottomCenterText("Test 2");
}
void screen3()
{
  displaySetScreen(screen3_ID);
  static DisplayObject text3(tft.width() / 2, 220);
  text3.writeBottomCenterText("AAAAA");
}

void displaySetScreen(uint8_t id)
{
  if (Display::getInstance().getCurrentScreen() != id)
  {
    Display::getInstance().setScreen(id);
  }
}

void debugScreen()
{
  displaySetScreen(debugScreen_ID);

  /*static DisplayObject AccX(0, 0);
  static DisplayObject AccX_Text(0, yPrintln(&AccX));

  static DisplayObject AccY(tft.width() / 2, 0);
  static DisplayObject AccY_Text(tft.width() / 2, yPrintln(&AccY));

  static DisplayObject AccZ(tft.width(), 0);
  static DisplayObject AccZ_Text(tft.width(), yPrintln(&AccZ));

  static DisplayObject GyroX(0, tft.height());
  static DisplayObject GyroX_Text(0, negyPrintln(&GyroX));

  static DisplayObject GyroY(tft.width() / 2, tft.height());
  static DisplayObject GyroY_Text(tft.width() / 2, negyPrintln(&GyroY));

  static DisplayObject GyroZ(tft.width(), tft.height());
  static DisplayObject GyroZ_Text(tft.width(), negyPrintln(&GyroZ));

  static DisplayObject Amod(tft.width() / 2, tft.height() / 2);

  static DisplayObject wAngle(0, tft.height() / 2);
  static DisplayObject wAngle_Text(0, yPrintln(&wAngle));

  static DisplayObject OilPress(tft.width(), tft.height() / 2);
  static DisplayObject OilPress_Text(tft.width(), yPrintln(&OilPress));

  AccX.size = 3;
  AccX_Text.size = 2;

  AccY.size = 3;
  AccY_Text.size = 2;

  AccZ.size = 3;
  AccZ_Text.size = 2;

  GyroX.size = 3;
  GyroX_Text.size = 2;

  GyroY.size = 3;
  GyroY_Text.size = 2;

  GyroZ.size = 3;
  GyroZ_Text.size = 2;

  Amod.size = 3;

  wAngle.size = 3;
  wAngle_Text.size = 2;

  OilPress.size = 3;
  OilPress_Text.size = 2;

  AccX.writeTopLeftText(Accel_X.value);
  AccX_Text.writeTopLeftText("AccX");

  AccY.writeTopCenterText(Accel_Y.value);
  AccY_Text.writeTopCenterText("AccY");

  AccZ.writeTopRightText(Accel_Z.value);
  AccZ_Text.writeTopRightText("AccZ");

  GyroX.writeBottomLeftText(Gyro_X.value);
  GyroX_Text.writeBottomLeftText("GyroX");

  GyroY.writeBottomCenterText(Gyro_Y.value);
  GyroY_Text.writeBottomCenterText("GyroY");

  GyroZ.writeBottomRightText(Gyro_Z.value);
  GyroZ_Text.writeBottomRightText("GyroZ");

  wAngle.writeLeftText(SteerWheel_Pos_Sensor.value);
  wAngle_Text.writeLeftText("Wheel");

  OilPress.writeRightText(Oil_Pressure_Sensor.value);
  OilPress_Text.writeRightText("Oil Press");

  Amod.writeCenterText(Accel.value);*/


  /*static DisplayObject Temp_BrakeFL(0, 0);
  static DisplayObject Temp_BrakeFL_Text(0, yPrintln(&Temp_BrakeFL));

  static DisplayObject Temp_BrakeFR(tft.width(), 0);
  static DisplayObject Temp_BrakeFR_Text(tft.width(), yPrintln(&Temp_BrakeFR));

  static DisplayObject Temp_BrakeRL(0, tft.height());
  static DisplayObject Temp_BrakeRL_Text(0, negyPrintln(&Temp_BrakeRL));

  static DisplayObject Temp_BrakeRR(tft.width(), tft.height());
  static DisplayObject Temp_BrakeRR_Text(tft.width(), negyPrintln(&Temp_BrakeRR));

  static DisplayObject Firewall_Temp1(0, tft.height() / 2);
  static DisplayObject Firewall_Temp1_Text(0, yPrintln(&Firewall_Temp1));

  static DisplayObject Firewall_Temp2(tft.width(), tft.height() / 2);
  static DisplayObject Firewall_Temp2_Text(tft.width(), yPrintln(&Firewall_Temp1));

  Temp_BrakeFL.size = 3;
  Temp_BrakeFL_Text.size = 2;

  Temp_BrakeFR.size = 3;
  Temp_BrakeFR_Text.size = 2;

  Temp_BrakeRL.size = 3;
  Temp_BrakeRL_Text.size = 2;

  Temp_BrakeRR.size = 3;
  Temp_BrakeRR_Text.size = 2;

  Firewall_Temp1.size = 3;
  Firewall_Temp1_Text.size = 2;

  Firewall_Temp2.size = 3;
  Firewall_Temp2_Text.size = 2;

  Temp_BrakeFL.writeTopLeftText(Disk_Temp_FL_Sensor.value + "C");
  Temp_BrakeFL_Text.writeTopLeftText("Brake FL");

  Temp_BrakeFR.writeTopRightText(Disk_Temp_FR_Sensor.value + "C");
  Temp_BrakeFR_Text.writeTopRightText("Brake FR");

  Temp_BrakeRL.writeBottomLeftText(Disk_Temp_RL_Sensor.value + "C");
  Temp_BrakeRL_Text.writeBottomLeftText("Brake RL");

  Temp_BrakeRR.writeBottomRightText(Disk_Temp_RR_Sensor.value + "C");
  Temp_BrakeRR_Text.writeBottomRightText("Brake RR");

  Firewall_Temp1.writeLeftText(Firewall_Temperature1_Sensor.value + "C");
  Firewall_Temp1_Text.writeLeftText("Firewall 1");

  Firewall_Temp2.writeRightText(Firewall_Temperature2_Sensor.value + "C");
  Firewall_Temp2_Text.writeRightText("Firewall 2");*/

  static DisplayObject MAP1(0, 0);
  static DisplayObject MAP1_Text(0, yPrintln(&MAP1));

  static DisplayObject MAP2(tft.width(), 0);
  static DisplayObject MAP2_Text(tft.width(), yPrintln(&MAP2));

  static DisplayObject MAF(0, tft.height());
  static DisplayObject MAF_Text(0, negyPrintln(&MAF));

  static DisplayObject Oil_Temp(tft.width(), tft.height());
  static DisplayObject Oil_Temp_Text(tft.width(), negyPrintln(&Oil_Temp));

  static DisplayObject wAngle(tft.width()/2, tft.height() / 2);
  static DisplayObject wAngle_Text(tft.width()/2, yPrintln(&wAngle));

  MAP1.size = 3;
  MAP1_Text.size = 2;

  MAP2.size = 3;
  MAP2_Text.size = 2;

  MAF.size = 3;
  MAF_Text.size = 2;

  Oil_Temp.size = 3;
  Oil_Temp_Text.size = 2;

  wAngle.size = 3;
  wAngle_Text.size = 2;


  MAP1.writeTopLeftText(MAP1_Sensor.value);
  MAP1_Text.writeTopLeftText("MAP 1");

  MAP2.writeTopRightText(MAP2_Sensor.value);
  MAP2_Text.writeTopRightText("MAP 2");

  MAF.writeBottomLeftText(MAF_Sensor.value);
  MAF_Text.writeBottomLeftText("MAF");

  Oil_Temp.writeBottomRightText(Oil_Temperature_Sensor.value + "C");
  Oil_Temp_Text.writeBottomRightText("Oil Temp");

  wAngle.writeCenterText(SteerWheel_Pos_Sensor.value);
  wAngle_Text.writeCenterText("Wheel");

}

void setupScreen()
{
}

String getTimeHMS()
{
  String time_string = "";
  if (timeSet)
  {
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_hour < 10)
    {
      time_string += "0";
    }
    time_string += String(timeinfo.tm_hour);
    time_string += ":";
    if (timeinfo.tm_min < 10)
    {
      time_string += "0";
    }
    time_string += String(timeinfo.tm_min);
    time_string += ":";
    if (timeinfo.tm_sec < 10)
    {
      time_string += "0";
    }
    time_string += String(timeinfo.tm_sec);
    return time_string;
  }
  else
  {
    time_string = "00:00:00";
    return time_string;
  }
}

void sendCANMessage(uint8_t id, uint8_t *data, uint8_t dlc)
{
  twai_message_t message;
  message.identifier = id;
  message.flags = 0;
  message.data_length_code = dlc;
  for (int i = 0; i < dlc; i++)
  {
    message.data[i] = data[i];
  }
  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
}

uint16_t yPrintln(const DisplayObject *obj)
{
  float size = (float)obj->size;
  if (obj->datum == TC_DATUM || obj->datum == TL_DATUM || obj->datum == TR_DATUM)
  {
    size = size;
  }
  if (obj->datum == MC_DATUM || obj->datum == ML_DATUM || obj->datum == MR_DATUM)
  {
    size = size / 2;
  }
  if (obj->datum == BC_DATUM || obj->datum == BL_DATUM || obj->datum == BR_DATUM)
  {
    size = 1;
  }
  return (uint16_t)(obj->pos_y + 7 + (size * font_size_const));
}

uint16_t negyPrintln(const DisplayObject *obj)
{
  float size = (float)obj->size;
  if (obj->datum == TC_DATUM || obj->datum == TL_DATUM || obj->datum == TR_DATUM)
  {
    size = size;
  }
  if (obj->datum == MC_DATUM || obj->datum == ML_DATUM || obj->datum == MR_DATUM)
  {
    size = size / 2;
  }
  if (obj->datum == BC_DATUM || obj->datum == BL_DATUM || obj->datum == BR_DATUM)
  {
    size = 1;
  }
  return (uint16_t)(obj->pos_y - (7 + (size * font_size_const)));
}

void temperatureTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint8_t data[8];
  const TickType_t xFrequency = pdMS_TO_TICKS(TEMPERATURE_TIMER);
  for (;;)
  {
    // Ler sensor 1
    float DiskTemp_FR = readTempC(&s_BrakeTempFR);
    sensorUpdate(DiskTemp_FR, Disk_Temp_FR_Sensor.index); // Front Right
    uint16_t DT_FR = floattoU16(DiskTemp_FR, 2);

    // Ler sensor 2
    float DiskTemp_FL = readTempC(&s_BrakeTempFL);
    sensorUpdate(DiskTemp_FL, Disk_Temp_FL_Sensor.index); // Front Left
    uint16_t DT_FL = floattoU16(DiskTemp_FL, 2);

    // Ler sensor 3
    float DiskTemp_RR = readTempC(&s_BrakeTempRR);
    sensorUpdate(DiskTemp_RR, Disk_Temp_RR_Sensor.index); // Rear Right
    uint16_t DT_RR = floattoU16(DiskTemp_RR, 2);

    // Ler sensor 4
    float DiskTemp_RL = readTempC(&s_BrakeTempRL);
    sensorUpdate(DiskTemp_RL, Disk_Temp_RL_Sensor.index); // Rear Left
    uint16_t DT_RL = floattoU16(DiskTemp_RL, 2);

    // Ler sensor 5
    float FirewallTemp_1 = readTempC(&s_FirewallTemp1);
    sensorUpdate(FirewallTemp_1, Firewall_Temperature1_Sensor.index);
    uint16_t DT_F1 = floattoU16(FirewallTemp_1, 2);

    // Ler sensor 6
    float FirewallTemp_2 = readTempC(&s_FirewallTemp2);
    sensorUpdate(FirewallTemp_2, Firewall_Temperature2_Sensor.index);
    uint16_t DT_F2 = floattoU16(FirewallTemp_2, 2);

    data[0] = DT_FR & 0xFF;
    data[1] = ((DT_FR >> 8) & 0x07) | ((DT_FL & 0x1F) << 3);
    data[2] = ((DT_FL >> 5) & 0x3F) | ((DT_RR & 0x03) << 6);
    data[3] = (DT_RR >> 2) & 0xFF;
    data[4] = ((DT_RR >> 10) & 0x01) | ((DT_RL & 0x7F) << 1);
    data[5] = ((DT_RL >> 7) & 0x0F) | ((DT_F1 & 0x0F) << 4);
    data[6] = ((DT_F1 >> 4) & 0x1F) | ((DT_F2 & 0x07) << 5);
    data[7] = (DT_F2 >> 3) & 0xFF;

    sendCANMessage(TEMP_ID, data, TEMP_DLC);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float readTempC(MAX6675 *Sensor)
{
  float data = Sensor->readCelsius();
  data = std::isnan(data) ? 0.0f : data;
  return data;
}

uint16_t floattoU16(float value, uint8_t precision_bits)
{
  uint16_t retValue = floor(value);
  float dif = value - retValue;
  uint16_t scale = 1 << precision_bits;
  uint16_t precision = dif * scale;
  retValue = (precision == scale) ? retValue + 1 : retValue;
  retValue = (retValue << precision_bits) + (precision % scale);
  return retValue;
}

void fn_Group_0(__u8 data[GROUP0_DLC])
{
  __u16 r_seconds = word(data[0], data[1]);
  __u16 r_pw1 = word(data[2], data[3]);
  __u16 r_pw2 = word(data[4], data[5]);
  __u16 r_RPM = word(data[6], data[7]);

  float seconds = MS2_Calibration(r_seconds, 1, 1);
  float pw1 = MS2_Calibration(r_pw1, 1, 1000);
  float pw2 = MS2_Calibration(r_pw2, 1, 1000);
  float RPM = MS2_Calibration(r_RPM, 1, 1);

  sensorUpdate(seconds, MS2_Sec.index);
  sensorUpdate(pw1, MS2_Bank1.index);
  sensorUpdate(pw2, MS2_Bank2.index);
  sensorUpdate(RPM, RPM_Sensor.index);
}

void fn_Group_1(__u8 data[GROUP1_DLC])
{
  __s16 r_Fin_Ign_Sprk_Adv = word(data[0], data[1]);
  __u8 r_BatchFire_Inj_Events = data[2];
  __u8 r_EngineStatus = data[3];
  __u8 r_Bank1_AFR_Tgt = data[4];
  __u8 r_Bank2_AFR_Tgt = data[5];

  float Fin_Ign_Sprk_Adv = MS2_Calibration(r_Fin_Ign_Sprk_Adv, 1, 10);
  float BatchFire_Inj_Events = MS2_Calibration(r_BatchFire_Inj_Events, 1, 1);
  float EngineStatus = MS2_Calibration(r_EngineStatus, 1, 1);
  float Bank1_AFR_Tgt = MS2_Calibration(r_Bank1_AFR_Tgt, 1, 10);
  float Bank2_AFR_Tgt = MS2_Calibration(r_Bank2_AFR_Tgt, 1, 10);

  sensorUpdate(Fin_Ign_Sprk_Adv, MS2_Fin_Ign_Sprk_Adv.index);
  sensorUpdate(BatchFire_Inj_Events, MS2_BatchFire_Inj_Events.index);
  sensorUpdate(EngineStatus, MS2_EngineStatus.index);
  sensorUpdate(Bank1_AFR_Tgt, MS2_Bank1_AFR_Tgt.index);
  sensorUpdate(Bank2_AFR_Tgt, MS2_Bank2_AFR_Tgt.index);
}

void fn_Group_2(__u8 data[GROUP2_DLC])
{

  __s16 r_Baro = word(data[0], data[1]);
  __s16 r_MAP = word(data[2], data[3]);
  __s16 r_MAT = word(data[4], data[5]);
  __s16 r_CLT = word(data[6], data[7]);

  float Baro = MS2_Calibration(r_Baro, 1, 10);
  float MAP = MS2_Calibration(r_MAP, 1, 10);
  float MAT = (MS2_Calibration(r_MAT, 1, 10) - 32) * FtC;
  float CLT = (MS2_Calibration(r_CLT, 1, 10) - 32) * FtC;

  sensorUpdate(Baro, MS2_Baro_Press.index);
  sensorUpdate(MAP, MS2_MAP.index);
  sensorUpdate(MAT, MS2_MAT.index);
  sensorUpdate(CLT, MS2_CLT.index);
}

void fn_Group_3(__u8 data[GROUP3_DLC])
{
  __s16 r_TPS = word(data[0], data[1]);
  __s16 r_Voltage = word(data[2], data[3]);
  __s16 r_AFR1 = word(data[4], data[5]);
  __s16 r_AFR2 = word(data[6], data[7]);

  float TPS = MS2_Calibration(r_TPS, 1, 10);
  float Voltage = MS2_Calibration(r_Voltage, 1, 10);
  float AFR1 = MS2_Calibration(r_AFR1, 1, 10);
  float AFR2 = MS2_Calibration(r_AFR2, 1, 10);

  sensorUpdate(TPS, MS2_TPS.index);
  sensorUpdate(Voltage, MS2_Voltage.index);
  sensorUpdate(AFR1, MS2_AFR1.index);
  sensorUpdate(AFR2, MS2_AFR2.index);
}

void fn_Group_7(__u8 data[GROUP7_DLC])
{
  __s16 r_cold_Adv = word(data[0], data[1]);
  __s16 r_TPS_rate = word(data[2], data[3]);
  __s16 r_MAP_rate = word(data[4], data[5]);
  __s16 r_RPM_rate = word(data[6], data[7]);

  float cold_Adv = MS2_Calibration(r_cold_Adv, 1, 10);
  float TPS_rate = MS2_Calibration(r_TPS_rate, 1, 10);
  float MAP_rate = MS2_Calibration(r_MAP_rate, 1, 1);
  float RPM_rate = MS2_Calibration(r_RPM_rate, 10, 1);

  sensorUpdate(cold_Adv, MS2_Cold_Adv.index);
  sensorUpdate(TPS_rate, MS2_TPS_Rate.index);
  sensorUpdate(MAP_rate, MS2_MAP_Rate.index);
  sensorUpdate(RPM_rate, MS2_RPM_Rate.index);
}

void fn_Group_8(__u8 data[GROUP8_DLC])
{
  __s16 r_MAF_Load = word(data[0], data[1]);
  __s16 r_Fuel_Load = word(data[2], data[3]);
  __s16 r_Fuel_Correction = word(data[4], data[5]);
  __s16 r_MAF = word(data[6], data[7]);

  float MAF_Load = MS2_Calibration(r_MAF_Load, 1, 10);
  float Fuel_Load = MS2_Calibration(r_Fuel_Load, 1, 10);
  float Fuel_Correction = MS2_Calibration(r_Fuel_Correction, 1, 10);
  float MAF = MS2_Calibration(r_MAF, 1, 100);

  sensorUpdate(MAF_Load, MS2_MAF_Load.index);
  sensorUpdate(Fuel_Load, MS2_Fuel_Load.index);
  sensorUpdate(Fuel_Correction, MS2_Fuel_Correction.index);
  sensorUpdate(MAF, MS2_MAF.index);
}

void fn_Group_9(__u8 data[GROUP9_DLC])
{
  __s16 r_O2_V1 = word(data[0], data[1]);
  __s16 r_O2_V2 = word(data[2], data[3]);
  __u16 r_Main_Dwell = word(data[4], data[5]);
  __u16 r_Trailing_Dwell = word(data[6], data[7]);

  float O2_V1 = MS2_Calibration(r_O2_V1, 1, 100);
  float O2_V2 = MS2_Calibration(r_O2_V2, 1, 100);
  float Main_Dwell = MS2_Calibration(r_Main_Dwell, 1, 10);
  float Trailing_Dwell = MS2_Calibration(r_Trailing_Dwell, 1, 10);

  sensorUpdate(O2_V1, MS2_O2_V1.index);
  sensorUpdate(O2_V2, MS2_O2_V2.index);
  sensorUpdate(Main_Dwell, MS2_Main_Ign_Dwell.index);
  sensorUpdate(Trailing_Dwell, MS2_Trailing_Ign_Dwell.index);
}

void fn_Group_15(__u8 data[GROUP15_DLC])
{
  __s16 r_OilPress = word(data[0], data[1]);
  //__s16 r_O2_V2 = word(data[2],data[3]);

  float OilPress = MS2_Calibration(r_OilPress, 1, 10);

  sensorUpdate(OilPress, Oil_Pressure_Sensor.index);
}