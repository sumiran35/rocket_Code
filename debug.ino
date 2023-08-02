
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
MPU6050 mpu;
#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t) for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define DPRINTSFN(StrSize, Name, ...) \
  { \
    char S[StrSize]; \
    Serial.print("\t"); \
    Serial.print(Name); \
    Serial.print(" "); \
    Serial.print(dtostrf((float)__VA_ARGS__, S)); \
  }  //StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t) if (false)
#define DPRINTSFN(...)  //blank line
#define DPRINTLN(...)   //blank line
#endif
