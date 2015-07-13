#include <ADXL345.h>
//#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
//#include <MS561101BA.h>
#include <I2Cdev.h>
//#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define DEBUG_RAW 0 //Turn on/off debug logging
#define DEBUG_YPR 0
#define DEBUG_KAL 1
#define XBEE_ID "5"

//Three instances for roll, pitch, yaw
Kalman kalmanP; // Create the Kalman instances
Kalman kalmanR;
Kalman kalmanY;
String inputString = ""; 
boolean stringComplete = false;

//Angle calculated using Kalman Filter
double kal_roll, kal_pitch, kal_yaw;
double roll, pitch, yaw;
char inChar;

//Raw sensor readongs array
float val[11];
//Quarternion array
float ypr[3];
uint32_t timer;

FreeIMU razor_imu = FreeIMU();

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  while(inChar != '?')
  {
    inChar = Serial.read();
  }
  Serial.println("xb"+(String)XBEE_ID);
  
  delay(500);
  //parameter enables/disables fast mode
  razor_imu.init(true);
  delay(500);
  razor_imu.getValues(val);  
  
  while(inChar != 'a')
  {
    inChar = Serial.read();
  }
  
  
  //Pitch, roll, yaw calculations
  razor_imu.getYawPitchRoll(ypr, val);
  
  //Set starting angle for filters
  kalmanR.setAngle(ypr[2]*RAD_TO_DEG);
  kalmanP.setAngle(ypr[1]*RAD_TO_DEG);
  kalmanY.setAngle(ypr[0]*RAD_TO_DEG);
  
  timer = micros();
  inputString.reserve(5);
}

void loop()
{
  if(inChar == '?')
  {
    Serial.println("xb"+(String)XBEE_ID);
  }
  
  razor_imu.getValues(val);
  double dt = (double)(micros() - timer)/1000000;
  timer = micros();
  
  //Calculate pitch, roll, yaw
  razor_imu.getYawPitchRoll(ypr, val);

  double roll_rate = val[3] * 0.007634;// / 131.0;
  double pitch_rate = val[4] * 0.007634;// / 131.0;
  double yaw_rate = val[5] * 0.007634;// / 131.0;
  
  //Feed values into the kalman object
  //And get the filtered angle back
  kalmanR.setAngle(ypr[2]);
  kal_roll = kalmanR.getAngle(ypr[2], roll_rate, dt);
  
  kalmanP.setAngle(ypr[1]);
  kal_pitch = kalmanP.getAngle(ypr[1], pitch_rate, dt);
  
  kalmanY.setAngle(ypr[0]);
  kal_yaw = kalmanY.getAngle(ypr[0], yaw_rate, dt);
  
  
  //Printing raw values (For debugging)
  if(DEBUG_RAW){
    for(int i=0; i<11; i++)
    {
      Serial.print(val[i]);
      Serial.print("\t");
    }
  }
  
  if(DEBUG_YPR){
    //Serial.print("YPR: ");
    Serial.print((String)ypr[0] + "," + 
                 (String)ypr[1] + "," +
                 (String)ypr[2] + ","); 
    //Serial.print(ypr[0]);
    //Serial.print(",");
    //Serial.print(ypr[1]);
    //Serial.print(",");
    //Serial.print(ypr[2]);
  }
  
  if(DEBUG_KAL){
    //Serial.print("YPR: ");
    
    //if(inChar == 'a')
    //{
      //Serial.println((String)kal_yaw + "," + (String)kal_pitch + "," + (String)kal_roll);
      Serial.println((String)ypr[0] + "," + (String)ypr[1] + "," + (String)ypr[2]);
      //inChar = 0;
    //}
  
    //Serial.print(",");
    //Serial.print(kal_pitch);
    //Serial.print(",");
    //Serial.println(kal_roll);
  }
  
  
}

void serialEvent() {
  
  inChar = (char)Serial.read();
  
}
  

