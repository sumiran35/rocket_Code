#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
MPU6050 mpu;
float Yaw, Pitch, Roll;
Servo pitchservo;
Servo rollservo;
float PIDvalue1;
float PIDvalue2;
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



#define LED_PIN 13  //

// supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// -4232  -706  1729  173 -94 37
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = { -3185, -1032, 1358, -40, -11, 60 };


// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0;  // tests if the interrupt is triggering
int IsAlive = -20;  // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte StartUP = 100;   // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize();  // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();  // same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char* StatStr[5]{ "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4" };

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return;  //only try 10 times
    delay(1000);
    MPU6050Connect();  // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  ");
  Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING);  //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus();         // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000);      // Let it Stabalize
  mpu.resetFIFO();  // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false;  // wait for next interrupt FF
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() {  // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) {  // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW);                    // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();                               // clear the buffer and start over
  } else {
    while (fifoCount >= packetSize) {            // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize);  // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath();                                     // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Blink the Light
  }
}


// ================================================================
// ===                        MPU Math                          ===
// ================================================================
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] * 180.0 / M_PI);
  Roll = (ypr[2] * 180.0 / M_PI);
  // DPRINTSTIMER(100) {
  //   DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
  //   DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
  //   DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
  //   DPRINTLN();
  // }
}
// ================================================================
// ===                         PID CODE                         ===
// ================================================================


float PID(float initerror, float* integralgain, float* olderror) {  //function for the PID
  float gain, derivativegain, totalgain;                            //Declaring PID variales
  gain = initerror * 0.5;                                           // Gain which is the initial error ( multiplied by a factor for sensitivity)(the error is how much the rocket has tilted off the axis)
  derivativegain = ((initerror - *olderror) / 17) * 0.2;            // Derivative gain is just the rate of change of the error over time
  *integralgain = *integralgain + (initerror * 0.01);               //integral gain is the addition of the errors over time
  totalgain = -(gain + derivativegain + *integralgain);             //Total Gain is the additions of all the previous gains
  *olderror = initerror;                                            //setting the old error as the new error so we are able to calculate the derivative gain
  //Serial.print("integralgain:");Serial.println(*integralgain);
  return totalgain;  //the function returns the total gain
}


// ================================================================
// ===.          PIDController - Pitch and Roll Values          ===
// ================================================================


//It sets up the maximum amount that the servos can move using 0 as the servo arm is rotated at the max to the right
//and the other value that is tested to the to the maximum value the servo can move to and the midpoint
//when you give it the 3 values of start midpoint and end you are calibrating the servos
//the code works by setting the vale that the MPU gets as 0 and it funnels the drift of the rocket into the PID
//it sets the values recorded after 7 seconds as the origine point and if it shifts to one direction it gives positive values
//and to the other direction it gives negative values. The servos cannot take negative values so you have to flip the negative into positive and
// make the servo go the opposite way this is what these 2 functions are doing by instead of setting the midpoint as 0 it sets it at the midpoint recorded

float PIDController(float PIDValue, float minValue, float midPoint, float maxValue) {
  float value = maxValue - midPoint;  // Difference between the maximum value and mid point.
  if (PIDValue < -value) {
    PIDValue = -value;
  } else if (PIDValue < 1) {
    PIDValue = PIDValue + value;
  } else if (PIDValue > midPoint) {
    PIDValue = midPoint;
  } else if (0 < PIDValue < midPoint) {
    PIDValue = value + PIDValue;
  }
  return PIDValue;
}

// ================================================================
// ===                Controlling the Servos                    ===
// ================================================================

//we got the values from the PID controller and filtered them to get the actual values of how much the servo should move in each direction
//now it's time to give the servo the commands for it to move and make sure that the PID values given do not exceed the maximum
//degrees a servo can move in order not to damage the PID or the servo

//change your values here for max angle of change value remember this is how much the servo will turn not the gimbal itself
float max_value_roll = 30;   //setting the max value the servos can move
float max_value_pitch = 30;  //both can have different values so it is safer to just put 2 different values for pitch and roll
int min_value = 0;           //setting the minimum value a servo can move
void servocontrol(float pitchcorrection, float rollcorrection) {
  // Pitch Correction
  if (min_value <= pitchcorrection <= max_value_pitch) {  //if the value is between the 2 min and max values write the pitch angle
    pitchservo.write(pitchcorrection);
  } else if (pitchcorrection > max_value_pitch) {  //if the value has reached the maximum just write the maximum
    pitchservo.write(max_value_pitch);
  } else if (pitchcorrection < min_value) {  //if the value has reached a minimum just write the minimum
    pitchservo.write(min_value);
  }
  // Roll Correction
  if (min_value <= rollcorrection <= max_value_roll) {  //if the value is between the 2 min and max values write the pitch angle
    rollservo.write(rollcorrection);
  } else if (rollcorrection > max_value_roll) {  //if the value has reached the maximum just write the maximum
    rollservo.write(max_value_roll);
  } else if (rollcorrection < min_value) {  //if the value has reached a minimum just write the minimum
    rollservo.write(min_value);
  }

  delay(50);
}
// ================================================================
// ===                         Setup                            ===
// ================================================================
float rollinit, pitchinit;
float timer = 0;
void setup() {
  //connecting the MPU to the arduino and attaching the servos
  Serial.begin(115200);  //115200
  while (!Serial)
    ;
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
  pitchservo.attach(9);
  rollservo.attach(10);
}
// ================================================================
// ===                          Loop                            ===
// ================================================================
int begin_control = 0;

// Setting up the initial errors
float pitcherror = 0, pitchintegralgain = 0, pitchservoangle = 0, previouspitcherror = 0;
float rollerror = 0, rollintegralgain = 0, rollservoangle = 0, previousrollerror = 0;

void loop() {
  if (mpuInterrupt) {  // wait for MPU interrupt or extra packet(s) available
    GetDMP();
    timer = timer + 100;
    if (timer == 70000) {  //timer to set the initial value that the rocket will adjust to it is 7 seconds it can be increased or decreased as pleased
      begin_control = 1;
      //settin the initial values of roll and pitch that will be used as reference for the code
      rollinit = Roll;
      pitchinit = Pitch;
    }
    //after timer runs the control begins
    if (begin_control == 1) {
      //getting the initial pitch and the initial roll
      pitcherror = Pitch - pitchinit;
      rollerror = Roll - rollinit;
      //sending the values of the errors to the PID
      PIDvalue1 = PID(pitcherror, &pitchintegralgain, &previouspitcherror);
      PIDvalue2 = PID(rollerror, &rollintegralgain, &previousrollerror);
      //giving PID values to the function that gives values to the servos
      pitchservoangle = PIDController(PIDvalue1, 0, 13, 30);
      rollservoangle = PIDController(PIDvalue2, 0, 18, 30);
      //writing the values to the servos
      servocontrol(pitchservoangle, rollservoangle);
    }
    //printing onto the serial monitor t see if something is wrong
    Serial.print("Pitch angle: ");
    Serial.println(pitchservoangle);
    Serial.print("Roll angle: ");
    Serial.println(rollservoangle);
  }
}
