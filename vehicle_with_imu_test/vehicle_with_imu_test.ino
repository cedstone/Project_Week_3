// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
// http://playground.arduino.cc/Main/MPU-6050?action=sourceblock&num=1

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float AcgX, AcgY, AcgZ;

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup(){
  Wire.begin();           // begin talking to the IMU
  wakeIMU();              // mpu6050 starts in power-saving mode
  Serial.begin(57600);    // begin talking to the motor controller
  lcd.begin(16, 2);       // 2 lines, 16 columns
}
void loop(){
  readIMU();              // Fetch data from IMU
  lcd.clear();
  lcd.setCursor(0, 0);                    // First line
  lcd.print((int)getSlopeDirection());    // Direction along the surface of maximum downwards gradient
  lcd.setCursor(0, 1);                    // second line
  lcd.print((int)runningAverage(getGradient()));  // How steep is the slope?
  delay(250);
}

void wakeIMU(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void readIMU(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


float getSlopeDirection(){
  float rads = 0;
  float degs = 0;
  AcgX = (float)AcX/16384;
  AcgY = (float)AcY/16384;
  rads = (float)atan(AcgY/AcgX);
  degs = toDegs(rads);
  if(AcgX > 0 )
  {
    degs = -degs;
  }
  else
  {
    if (AcgY < 0)
    {
      degs = 180 - degs;
    }
    else
    {
      degs = -180 - degs;
    }
  }
  return degs;
}

float getGradient(){
  float rads = 0;
  float degs = 0;
  AcgZ = (float)AcZ/16384;          // Correct to units of 9.81m/s^2
  /* 
   * Potential for bugginess: 
   * If AcgZ is greater than 1, then acos(AcgZ) is NaN 
   * BUT if AcgZ is exactly 1, then acos(AcgZ) is zero
   * and if acos(AcgZ) is zero, then degs is zero
   * if degs is zero then degs to a negative power (-1.6) is infinite
   * because 0^-1.6 == 1/(0^1.6) == 1/0
   * To avoid this problem, the code below limits AcgZ to 0.99
   * It's hacky, but it works!
   */
  if (AcgZ >= 1) { AcgZ = 0.99; }       
  if (AcgZ <= -1) { AcgZ = -0.99; }
  rads = (float)acos(AcgZ);       // Get gradient, assuming vehicle is at rest, on earth, at sea level
  degs = toDegs(rads);            // Convert from radians to degrees
  degs =  degs - (500*(float)pow(degs, -1.6));    // Empirically found formula to correct error in angle
  if (degs < 0 ) { degs = 0; }                    // Clamp positive, the gradient can't have a negative magnitude!
  return degs;
}

float toDegs(float rads){
  float degs = 360*(rads/6.2832);     // Is there a pre-definined valuye of pi we can use?
  return degs;
}


float runningAverage(float M) {
  // This function from
  // http://playground.arduino.cc/Main/RunningAverage
  // Modified to work with floats
  #define LM_SIZE 5
  static float LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum / count;
}
