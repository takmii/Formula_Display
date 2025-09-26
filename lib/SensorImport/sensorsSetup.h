#ifndef SENSORS_SETUP_H
#define SENSORS_SETUP_H
#include <Arduino.h>
#include <variables.h>

#define V_4095 3.3
#define A_20V 24818
#define A_5_5V 6825
const float VRefMax = 5.1615;

class Sensor {
    public:
      String value ="$";
      String name;
      __u8 index;
    };

extern Sensor Voltage_Sensor;
extern Sensor Internal_Temperature_Sensor;
extern Sensor V_Ref_Sensor;
extern Sensor Gear_Pos_Sens;
extern Sensor RPM_Sensor;
extern Sensor Susp_Pos_FR_Sensor;
extern Sensor Susp_Pos_FL_Sensor;
extern Sensor Susp_Pos_RR_Sensor;
extern Sensor Susp_Pos_RL_Sensor;
extern Sensor SteerWheel_Pos_Sensor;
extern Sensor Accel_X;
extern Sensor Accel_Y;
extern Sensor Accel_Z;
extern Sensor Accel;
extern Sensor Gyro_X;
extern Sensor Gyro_Y;
extern Sensor Gyro_Z;
extern Sensor Wheel_Spd_FR_Sensor;
extern Sensor Wheel_Spd_FL_Sensor;
extern Sensor Wheel_Spd_RR_Sensor;
extern Sensor Wheel_Spd_RL_Sensor;
extern Sensor Disk_Temp_FR_Sensor;
extern Sensor Disk_Temp_FL_Sensor;
extern Sensor Disk_Temp_RR_Sensor;
extern Sensor Disk_Temp_RL_Sensor;
extern Sensor F_Brakeline_Pressure;
extern Sensor R_Brakeline_Pressure;
extern Sensor Caliper_Pressure_FR_Sensor;
extern Sensor Caliper_Pressure_FL_Sensor;
extern Sensor Caliper_Pressure_RR_Sensor;
extern Sensor Caliper_Pressure_RL_Sensor;
extern Sensor Throttle_Position_Sensor;
extern Sensor Brake_Position_Sensor;
extern Sensor Fuel_Pressure_Sensor;
extern Sensor Fuel_Temperature_Sensor;
extern Sensor Oil_Temperature_Sensor;
extern Sensor Oil_Pressure_Sensor;
extern Sensor Intercooler_Temperature_Sensor;
extern Sensor Intercooler_Pressure_Sensor;
extern Sensor In_Cooling_Temperature_Sensor;
extern Sensor Out_Cooling_Temperature_Sensor;
extern Sensor MAP1_Sensor;
extern Sensor MAP2_Sensor;
extern Sensor MAF_Sensor;
extern Sensor Cylinder_1_Pressure_Sensor;
extern Sensor Cylinder_2_Pressure_Sensor;
extern Sensor MS2_Sec;
extern Sensor MS2_Bank1;
extern Sensor MS2_Bank2;
extern Sensor MS2_Baro_Press;
extern Sensor MS2_MAP;
extern Sensor MS2_MAT;
extern Sensor MS2_CLT;
extern Sensor MS2_TPS;
extern Sensor MS2_Voltage;
extern Sensor MS2_AFR1;
extern Sensor MS2_AFR2;
extern Sensor MS2_Lambda;
extern Sensor MS2_Cold_Adv;
extern Sensor MS2_TPS_Rate;
extern Sensor MS2_MAP_Rate;
extern Sensor MS2_RPM_Rate;
extern Sensor MS2_MAF_Load;
extern Sensor MS2_Fuel_Load;
extern Sensor MS2_Fuel_Correction;
extern Sensor MS2_MAF;
extern Sensor MS2_O2_V1;
extern Sensor MS2_O2_V2;
extern Sensor MS2_Main_Ign_Dwell;
extern Sensor MS2_Trailing_Ign_Dwell;
extern Sensor MS2_Fin_Ign_Sprk_Adv;
extern Sensor MS2_BatchFire_Inj_Events;
extern Sensor MS2_EngineStatus;
extern Sensor MS2_Bank1_AFR_Tgt;
extern Sensor MS2_Bank2_AFR_Tgt;
extern Sensor Firewall_Temperature1_Sensor;
extern Sensor Firewall_Temperature2_Sensor;
extern Sensor Wing_Extensometer_1_Sensor;
extern Sensor Wing_Extensometer_2_Sensor;
extern Sensor Wing_Extensometer_3_Sensor;
extern Sensor Wing_Extensometer_4_Sensor;
extern Sensor SD_Status;
extern Sensor AccGyro_Status;
    

// Formato para adicionar mais sensores " extern Sensor SensorVariable "

extern Sensor* sensorIndex[];

__u8 indexSetup();
void setSensorName();

float vRef_Proportion(uint16_t data);
String Gear_Pos(__u8 value);
float LinearSensor(__u16 value, float a, float b);
float TempSensor(__u16 value, __u32 R1, __u32 R2, float c1, float c2, float c3);
float vBatSensor(__u16 value);
float vRefSensor(__u16 value);
float internalTemp(__u16 value);
float suspSensor(__u16 value);
float mapSensor(__u16 value);
float mafSensor(__u16 value);
float tempSensor(__u16 value,double a,double b,double c);
float tempOilSensor(__u16 value,double a,double b,double c);
//float wheelAngleSensor(__u16 value);
unsigned short degreesofPrecision(uint16_t data, float max_Value, float decimal);
signed short AccAxisCalibration();
signed short GyroAxisCalibration();
float MS2_Float_Calibration(unsigned short value, unsigned short m, unsigned short d);
unsigned short MS2_U16_Calibration(unsigned short value, unsigned short m, unsigned short d);
unsigned char MS2_U8_Calibration(unsigned short value, unsigned short m, unsigned short d);
signed short MS2_S16_Calibration(unsigned short value, unsigned short m, unsigned short d);
signed char MS2_S8_Calibration(unsigned short value, unsigned short m, unsigned short d);

class SW_Settings
{
private:
  signed short dValueDir;
  signed short dValueEsq;
  double center;
  double oldvalues[3] = {0, 0, 0};
  double oldconv[3] = {0, 0, 0};
  unsigned char regions[3] = {0, 0, 0};
  bool hypZone = 1;

public:
  unsigned short mValueEsq;
  unsigned short mValueDir;

  SW_Settings(double centerDeg, uint16_t mValEsq, uint16_t mValDir)
      : mValueEsq(2*mValEsq), mValueDir(2*mValDir), center(centerDeg)
  {
    dValueEsq = -static_cast<signed short>(360 - mValueDir); // Logica invertida
    dValueDir = static_cast<signed short>(360 - mValueEsq);
  }
    unsigned char getRegion(double value)
  {
    unsigned char region;
    if (value > dValueEsq && value < dValueDir)
    {
      region = 1;
    }
    else if (value <= dValueEsq)
    {
      region = 0;
    }
    else
    {
      region = 2;
    }
    return region;
  }

  unsigned char getOldRegion()
  {
    unsigned char sum = 0;
    unsigned char n = 0;
    for (unsigned char i = 0; i < 3; i++)
    {
      if (regions[i] != 0)
      {
        sum += regions[i];
        n++;
      }
    }
    return sum / n;
  }

  double getOldValues()
  {
    double sum = 0;
    unsigned char n = 0;
    for (unsigned char i = 0; i < 3; i++)
    {
      if (oldvalues[i] != 0)
      {
        sum += oldvalues[i];
        n++;
      }
    }
    return sum / n;
  }

  double getOldConvs()
  {
    double sum = 0;
    unsigned char n = 0;
    for (unsigned char i = 0; i < 3; i++)
    {
      if (oldconv[i] != 0)
      {
        sum += oldconv[i];
        n++;
      }
    }
    return sum / n;
  }

  void setOldRegion(unsigned char value)
  {
    for (unsigned char i = 2; i > 0; i--)
    {
      regions[i] = regions[i - 1];
    }
    regions[0] = value;
  }

    void setOldValues(double value)
  {
    for (unsigned char i = 2; i > 0; i--)
    {
      oldvalues[i] = oldvalues[i - 1];
    }
    oldvalues[0] = value;
  }

      void setOldConv(double value)
  {
    for (unsigned char i = 2; i > 0; i--)
    {
      oldconv[i] = oldconv[i - 1];
    }
    oldconv[0] = value;
  }
  
  String steeringWheelValue(unsigned short r_value)
  {
    float prop = vRef_Proportion(r_value);
    double value = (prop*360);

    static bool hypothetical = 1;
    static unsigned char current_region = 0;
    static unsigned char old_region;
    static double old_value;
    static double old_conv;
    String retValue = "";
    double conv_value = fmodf(((value - center) + 540), 360) - 180;

    if (hypothetical)
    {
      if (conv_value > dValueEsq && conv_value < dValueDir)
      {
        hypothetical = 0;
        hypZone = 0;
      }
    }
    else
    {
      old_region = getOldRegion();
      old_value = getOldValues();
      old_conv = getOldConvs();
      if (conv_value > dValueEsq && conv_value < dValueDir)
      {
        hypZone = 0;
        current_region = getRegion(conv_value);
      }
      else{
        hypZone = 1;
        conv_value = fmodf(((value - old_value) + 540), 360) - 180 + old_conv;
        current_region = getRegion(conv_value);
        if (current_region != old_region && current_region!= 1)
        {
          hypothetical = 1;
        }
      }
    } 
    retValue = String(conv_value / 2);
    if (hypothetical)
    {
      retValue = retValue + "?";
    }
    setOldRegion(current_region);
    setOldValues(value);
    setOldConv(conv_value);
    return retValue;
  }
};

extern SW_Settings SteeringWheel;

#endif