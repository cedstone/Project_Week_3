#include <Wire.h>
#define uchar unsigned char
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);
#include <SoftwareSerial.h> // Include the Software Serial library to talk to the robot
SoftwareSerial robot(6,7);              // Configure the software serial connection to the robot

uchar t;
uchar incomingvalue;
//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
uchar data[16];
uchar blackdata[] = {0, 0, 0, 0, 0, 0, 0, 0};
uchar whitedata[] = {255, 255, 255, 255, 255, 255, 255, 255};

#define button0 9
#define button1 10

float Kp = 0;
float Ki = 0;
float Kd = 0;
float errorOld = 0;
float error = 0;
float errorSum = 0;
float errorNew = 0;
float pidoutput = 0;
#define Ka Kp    // The coefficient we're adjusting at the moment;
#define adjustStep 1

float leftMotorSpeed = 0;
float rightMotorSpeed = 0;
// Default speed of the motors, this is modified by the PID output
#define leftMotorBaseSpeed 20
#define rightMotorBaseSpeed 20

// Speed limits of the motors
#define min_speed -30
#define max_speed 50

void setup()
{
 lcd.begin(16, 2);
 lcd.print("Button test");
 pinMode(button0, INPUT_PULLUP);
 pinMode(button1, INPUT_PULLUP);
  
 Wire.begin(); // join i2c bus (address optional for master)
 Serial.begin(9600); // start serial for output
 robot.begin(57600);  // Start serial for output to robot
 t = 0;
 callibrate();
}

void loop()
{
 getRawData();
 updatePID();
 updateDisplay();
 pidoutput = PID((long)weightedAverage(data));
 //
leftMotorSpeed = leftMotorBaseSpeed + pidoutput;     // Calculate the modified motor speed
rightMotorSpeed = rightMotorBaseSpeed - pidoutput;

if(leftMotorSpeed > 0)
{
  leftMotorSpeed = constrain(leftMotorSpeed, 0, max_speed);
  robot.write("#D1f");
  robot.write("#S1");
  robot.print((int)leftMotorSpeed);
}
else
{
  leftMotorSpeed = constrain(leftMotorSpeed, min_speed, 0);
  robot.write("#D1r");
  robot.write("#S1");
  robot.print(-(int)leftMotorSpeed);
}
if(rightMotorSpeed > 0)
{
  rightMotorSpeed = constrain(rightMotorSpeed, 0, max_speed);
  robot.write("#D2f");
  robot.write("#S2");
  robot.print((int)rightMotorSpeed);
}
else
{
  rightMotorSpeed = constrain(rightMotorSpeed, min_speed, 0);
  robot.write("#D2r");
  robot.write("#S2");
  robot.print(-(int)rightMotorSpeed);
}
 
 delay(5);
}

void printData(){
 for(int i=0; i<=14; i+=2){
  Serial.print(data[i]);
  Serial.print(", ");
 }
 Serial.println("");
}

// Find the position, right to left, of the line
int weightedAverage(uchar data[]){
  float sum1 = 0;
  float sum2 = 0;
  float result = 0;
  for(int i=0; i<=14; i+=2){
    sum1 += (i*(255-data[i]));
  }
  for(int i=0; i<=14; i+=2){
    sum2 += (255-data[i]);
  }
  result = sum1/sum2;
  //result = map(result, 0, 14, -127, 127);
  result *= ((float)255/14);
  result -= 127;
  return (int)result;
}


void updatePID(void)
{
  if(digitalRead(button0) == LOW){
    Ka += adjustStep;
    updateDisplay();
  }
  if(digitalRead(button1) == LOW) {
    Ka -= adjustStep;
    updateDisplay();
  }
}

void updateDisplay(void)
{
  lcd.setCursor(0, 0);
  lcd.print("Error = ");
  lcd.print(weightedAverage(data));
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print("Ka = ");
  lcd.print((int)Ka); 
  lcd.print("          ");
}

void getRawData(void){
 Wire.requestFrom(9, 16); // request 16 bytes from slave device #9
 while (Wire.available()) // slave may send less than requested
 {
   incomingvalue = Wire.read();   // Read value
   // Constrain and map value to fit between the limits of calibration values
   // The "/2" terms are there because the odd-numbered bytes of raw data are useless, and not recorded in blackdata and whitedata
   data[t] = constrain(incomingvalue, blackdata[t/2], whitedata[t/2]);
   data[t] = map(data[t], blackdata[t/2], whitedata[t/2], 0, 255);
   if (t < 15)
   t++;
   else
   t = 0;
 }
}

void callibrate(){
 Serial.println("Expose to black surface, then push black button");
 lcd.clear();
 lcd.print("* Callibration *");
 lcd.setCursor(0, 1);
 lcd.print("    black...    ");
 while(digitalRead(button0) == HIGH){};
 for(int i=0; i<=7; i++){
    getRawData();
    blackdata[i] = data[2*i];
 }
 Serial.print("Recorded black values as ");
 printData();
 Serial.println("Expose to white surface, then push white button");
 lcd.clear();
 lcd.print("* Callibration *");
 lcd.setCursor(0, 1);
 lcd.print("    white...    ");
 while(digitalRead(button1) == HIGH){};
 for(int i=0; i<=7; i++){
    getRawData();
    whitedata[i] = data[2*i];
 }
 Serial.print("Recorded white values as ");
 printData();
 lcd.clear();
 lcd.print("* Callibration *");
 lcd.setCursor(0, 1);
 lcd.print("      done     ");
 delay(500);
}

float PID(long errorNew)
{
  // PID loop
  errorOld = error;        // Save the old error for differential component
  error = errorNew;        // Calculate the error in position
  errorSum += error;
  float proportional = error * Kp;  // Calculate the components of the PID
  float integral = errorSum * Ki;
  float differential = (error - errorOld) * Kd;
  long output = proportional + integral + differential;  // Calculate the result
  return output;
}

