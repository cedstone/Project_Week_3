// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
// http://playground.arduino.cc/Main/MPU-6050?action=sourceblock&num=1

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float AcgX, AcgY, AcgZ;

void setup(){
  Wire.begin();
  wakeIMU();
  Serial.begin(9600);
}
void loop(){
  readIMU();
  //Serial.print("AcX = "); Serial.print(AcgX);
  //Serial.print(" | AcY = "); Serial.print(AcgY);
  //Serial.print(" | AcZ = "); Serial.print(AcgZ);
  //Serial.print(" | GyX = "); Serial.print(GyX);
  //Serial.print(" | GyY = "); Serial.print(GyY);
  //Serial.print(" | GyZ = "); Serial.println(GyZ);
  Serial.print(getSlopeDirection());
  Serial.print("\t");
  Serial.println(runningAverage(getGradient()));
  delay(100);
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
  AcgZ = (float)AcZ/16384;
  if (AcgZ > 1) { AcgZ = 1; }
  rads = (float)acos(AcgZ);
  degs = toDegs(rads);
  degs = degs - 10; // Hack to avoid as-yet unexplained error
  return degs;
}

float toDegs(float rads){
  float degs = 360*(rads/6.28);
  return degs;
}


float runningAverage(float M) {
  // From http://playground.arduino.cc/Main/RunningAverage
  #define LM_SIZE 10
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
