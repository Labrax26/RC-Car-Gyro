//Project Title: Car Gyro
//Author: Daan Smit
//Date: 24-2-2026
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

//========================================================================================================================//
//                                                 Defines                                                                //                                                                 
//========================================================================================================================//

#define MPU 0x68
#define SCL_PIN PB6
#define SDA_PIN PB7

//========================================================================================================================//
//                                                 Global Variables                                                       //                                                                 
//========================================================================================================================//

float X_Rate, Y_Rate, Z_Rate;
int RateCalibrationNumber;
float X_Rate_Calibration, Y_Rate_Calibration, Z_Rate_Calibration;


//========================================================================================================================//
//                                                 Function prototypes                                                    //                                                                 
//========================================================================================================================//

void Init_Gyro();
void Gyro_Calibration();
void Get_Gyro_Data();

//========================================================================================================================//
//                                                      Setup                                                             //                                                                 
//========================================================================================================================//

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(2000);
  Serial.println("Connected ready to print");
  
  // Initialize Gyro and run calibration
  Init_Gyro();
  Gyro_Calibration();
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

  Serial.print("X_Rate:   ");
  Serial.print(X_Rate);
  Serial.print("    Y_Rate:   ");
  Serial.print(Y_Rate);
  Serial.print("    Z_Rate:   ");
  Serial.println(Z_Rate);

  // 20 ms delay
  delay(20);
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