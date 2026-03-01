//Project Title: Car Gyro
//Author: Daan Smit
//Date: 26-2-2026
//Version: 1

 /* Description:
 * Creating a Car Gyro using blackpill with mpu6050
 * 
 *
 * Hardware:
 *   - F411CE blackpill
 *   - ER5A
 *   - Radiomaster pocket
 *   - MPU6050
 *
 * Software/Libraries:
 *  - Wire.h
*   - STM32Servo
 *
 * Credits/References:
 *
 * License:
 *   - non
 */

//========================================================================================================================//
//                                                 Libraries                                                              //                                                                 
//========================================================================================================================//

#include <Wire.h>
#include <Servo.h>

//========================================================================================================================//
//                                                 Defines                                                                //                                                                 
//========================================================================================================================//

#define MPU 0x68

//========================================================================================================================//
//                                                 Global Variables                                                       //                                                                 
//========================================================================================================================//

const int ch1Pin = PA12;
const int ch2Pin = PA11;
const int ch3Pin = PA10;
const int ch4Pin = PA9;
const int ch5Pin = PA8;

const int servoPin = PB3;
const int escPin = PB10;

const float sampleTime = 0.004;

uint32_t currentTime, previousTime;
uint loopTimer;

float X_Rate, Y_Rate, Z_Rate;
float X_Rate_Calibration, Y_Rate_Calibration, Z_Rate_Calibration;
int RateCalibrationNumber;

volatile uint32_t rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4; // time when pwm signal rises
volatile uint32_t channel_1_pulsewidth, channel_2_pulsewidth, channel_3_pulsewidth, channel_4_pulsewidth;

float DesiredYaw;
float ErrorYaw;
float inputYaw;
float PrevErrorYaw = 0;
float PrevItermYaw = 0;

const int KpYaw = 2;
const int KiYaw = 0;
const int KdYaw = 0;

Servo steeringServo;
Servo motorControl;

//========================================================================================================================//
//                                                 Function prototypes                                                    //                                                                 
//========================================================================================================================//

void Init_Gyro();
void Gyro_Calibration();
void Get_Gyro_Data();
void DesiredRate();
void Error_Calculation();
int32_t getRadioPWM(int channel_number);

//========================================================================================================================//
//                                                      Setup                                                             //                                                                 
//========================================================================================================================//

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Connected ready to print");

  // Set channel pin pull up
  pinMode(ch1Pin, INPUT_PULLUP);
  pinMode(ch2Pin, INPUT_PULLUP);
  pinMode(ch3Pin, INPUT_PULLUP);
  pinMode(ch4Pin, INPUT_PULLUP);
  pinMode(ch5Pin, INPUT_PULLUP);

  // Initialize Gyro and run calibration
  Init_Gyro();
  Gyro_Calibration();

  steeringServo.attach(PB3);
  steeringServo.writeMicroseconds(1500);  // Center position using 1500 uS

  motorControl.attach(PB10);

  // interupt on pin with ISR on mode Change pwm rising edge
  attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
  delay(250);


  int32_t ch3 = getRadioPWM(3);
  
  // Check if the throttle is inbetween 1020 OR 1050
  // if its under 1020 the while loop is true and dont exit it ensures it is not reading a fault in the receiver
  // if its above 1050 the loop is true and it ensures the throttle stick is high
  while (ch3 < 1050 || ch3 > 1100) {
    delay(4);
    ch3 = getRadioPWM(3);
  }
  

  loopTimer=micros();
}


//========================================================================================================================//
//                                                        Loop                                                            //                                                                 
//========================================================================================================================//

void loop() {
  // Get Rate Data
  Get_Gyro_Data();
  X_Rate -= X_Rate_Calibration;
  Y_Rate -= Y_Rate_Calibration;
  Z_Rate -= Z_Rate_Calibration;


  // Call the PID functions
  DesiredRate();
  Error_Calculation();
  PID_Yaw();
  steeringServo.writeMicroseconds(inputYaw);

  int32_t ch3 = getRadioPWM(3);
  ch3 = constrain(ch3, 1000, 2000);
  motorControl.writeMicroseconds(ch3);



  while (micros() - loopTimer < 4000);  // 250 Hz T = 0.004s loop
  loopTimer=micros();
}

//========================================================================================================================//
//                                                  Function Init_Gyro                                                    //                                                                 
//========================================================================================================================//

void Init_Gyro () {
  // Init I2C hardware and set STM32 as master with fast mode
  Wire.begin();
  Wire.setClock(400000);

  // Disable sleep mode (default on when powering up)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set DLPF at 10 Hz
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Set Full Scale Range at +-500 dps
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

//========================================================================================================================//
//                                                 Function Gyro_Calibration                                              //                                                                 
//========================================================================================================================//

void Gyro_Calibration () {
  // Get 2000 samples for each axis
  for(RateCalibrationNumber = 0; RateCalibrationNumber < 2000;  RateCalibrationNumber++) {
    Get_Gyro_Data();
    X_Rate_Calibration += X_Rate;
    Y_Rate_Calibration += Y_Rate;
    Z_Rate_Calibration += Z_Rate;
    delay(1);
  }

  // Divide with 2000 to remove drift
  X_Rate_Calibration /= 2000;
  Y_Rate_Calibration /= 2000;
  Z_Rate_Calibration /= 2000;
}

//========================================================================================================================//
//                                                  Function Get_Gyro_Data                                                //                                                                 
//========================================================================================================================//

void Get_Gyro_Data () {
  // Go to GYRO_XOUT[15:8]
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();

  // Read 6 bytes
  Wire.requestFrom(MPU, 6);
  int16_t raw_X_Rate = Wire.read() << 8 | Wire.read();
  int16_t raw_Y_Rate = Wire.read() << 8 | Wire.read();
  int16_t raw_Z_Rate = Wire.read() << 8 | Wire.read();

  X_Rate = (float)raw_X_Rate/65.5;
  Y_Rate = (float)raw_Y_Rate/65.5;
  Z_Rate = (float)raw_Z_Rate/65.5;
}

//========================================================================================================================//
//                                                 Function DesiredRate                                                   //                                                                 
//========================================================================================================================//

void DesiredRate () {
  int32_t ch1 = getRadioPWM(1);
  DesiredYaw = 0.4*(ch1-1500);
}

//========================================================================================================================//
//                                                 Function Error_Calculation                                             //                                                                 
//========================================================================================================================//

void Error_Calculation() {
  ErrorYaw = DesiredYaw - Z_Rate;
}

//========================================================================================================================//
//                                                  Function PID_Yaw                                                      //                                                                 
//========================================================================================================================//

void PID_Yaw() {
  // Local variables
  float u = 0;
  float Pterm = KpYaw * ErrorYaw;
  float Iterm = PrevItermYaw + (KiYaw * ErrorYaw * sampleTime);
  if (Iterm > 400) Iterm = 400;
  if (Iterm < -400) Iterm = -400;
  float Dterm = KdYaw * ((ErrorYaw - PrevErrorYaw)/sampleTime);
  float PID_Output = Pterm + Iterm + Dterm;
  u = PID_Output;
  inputYaw = u;
  PrevItermYaw = Iterm;
  PrevErrorYaw = ErrorYaw;
  inputYaw = constrain(PID_Output, -500, 500) + 1500;
}

//========================================================================================================================//
//                            Get current radio commands from interrupt routines                                          //                                                                 
//========================================================================================================================//

int32_t getRadioPWM(int channel_number) {
  int32_t returnPWM = 0; // Initialize variabel
  if (channel_number == 1) {
    returnPWM = channel_1_pulsewidth;
  }
  else if (channel_number == 2) {
    returnPWM = channel_2_pulsewidth;
  }
  else if (channel_number == 3) {
    returnPWM = channel_3_pulsewidth;
  }
  else if (channel_number == 4) {
    returnPWM = channel_4_pulsewidth;
  }
  return returnPWM;
}


//========================================================================================================================//
//                                                 INTERRUPT HANDLERS                                                     //                                                                 
//========================================================================================================================//

void getCh1 () {
  int state = digitalRead(ch1Pin);
  if (state == HIGH) {
    rising_edge_start_1 = micros();
  }
  else {
    channel_1_pulsewidth = micros() - rising_edge_start_1;
  }
}

void getCh2 () {
  int state = digitalRead(ch2Pin);
  if (state == HIGH) {
    rising_edge_start_2 = micros();
  }
  else {
    channel_2_pulsewidth = micros() - rising_edge_start_2;
  }
}

void getCh3 () {
  int state = digitalRead(ch3Pin);
  if (state == HIGH) {
    rising_edge_start_3 = micros();
  }
  else {
    channel_3_pulsewidth = micros() - rising_edge_start_3;
  }
}

void getCh4 () {
  int state = digitalRead(ch4Pin);
  if (state == HIGH) {
    rising_edge_start_4 = micros();
  }
  else {
    channel_4_pulsewidth = micros() - rising_edge_start_4;
  }
}