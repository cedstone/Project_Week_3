#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int stage = 99;    //on floor
const int m = 134;  // one metre
unsigned long oldtime = 0;
unsigned long oldyaw = 0;

void setup() {
    lcd.begin(16, 2);
    lcd.print("yaw settling");
    Serial.begin(57600);    // begin talking to the motor controller
    setupimu();
    delay(20000);
    Serial.print("#Sb025,025");
}


void loop() {
    //int oldencodervalue = 0;
    if (!dmpReady) return;
    updateimu();

    while (!mpuInterrupt && fifoCount < packetSize) {
      int pitch = (int)(ypr[1] * 180/M_PI);
      int yaw = (int)(ypr[0] * 180/M_PI);
      lcd.setCursor(0, 1);
      lcd.print(yaw); 
      lcd.print("   ");  
      lcd.setCursor(5, 1);
      lcd.print(pitch); 
      lcd.print("     ");  
      if ((stage == 0) && (abs(pitch) > 15)){
          stage = 1;    // on ramp
          Serial.print("#Sb025,025");
      }
      if ((stage == 1) && (abs(pitch) < 15)){
          stage = 2;    // on top of ramp
          Serial.print("#Sb025,025");
          oldtime = millis();
      }
       if ((stage == 2) && (millis() > (oldtime+1))) 
      {
        stage = 3;
        oldtime = millis();
       Serial.print("#Sb000,000");       
      }

      if ((stage == 99) && (millis < 20000))
      {
        
      }
      
      if ((stage == 3) && (millis() > (oldtime+5000)))
      {
        stage = 4;
        oldtime = millis();
        oldyaw = yaw;
        Serial.print("#d1f");
        Serial.print("#d2r");
        Serial.print("#Sb025,025");
      }
      if ((stage == 4) && (yaw == oldyaw-1) && (millis() > (oldtime+1000)))
     {
      Serial.print("#Sb000,000");
     }
      
    }
}

void setupimu()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. 

    //Serial.begin(115200);

    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
}

void updateimu()
{
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //Serial.print("ypr\t");
        //Serial.print(ypr[0] * 180/M_PI);
        //Serial.print("\t");
        //Serial.print(ypr[1] * 180/M_PI);
        //Serial.print("\t");
        //Serial.println(ypr[2] * 180/M_PI);

    }
}

long unsigned int readEncoder (int side) {
  long unsigned int encoder = 0;
  
  // request the encoder value for motor 1 (assuming 1=2)
  if (side == 1)
  {
     Serial.write("#e1");
  }
  else
  {
    Serial.write("#e2");
  }
  
  // wait breifly for response
  delay(50);

  encoder = Serial.read();
  encoder += (Serial.read()<<8);
  encoder += (Serial.read()<<16);
  encoder += (Serial.read()<<24);
  return encoder;
}

