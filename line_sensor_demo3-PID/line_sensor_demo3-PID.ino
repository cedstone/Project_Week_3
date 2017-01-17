#include <Wire.h>
#include <SoftwareSerial.h> 

#define uchar unsigned char
uchar t;
uchar incomingvalue;
//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
uchar data[16];
uchar blackdata[] = {0, 0, 0, 0, 0, 0, 0, 0};
uchar whitedata[] = {255, 255, 255, 255, 255, 255, 255, 255};

SoftwareSerial robot(7, 8);              // Configure the software serial connection to the robot

#define leftMotorBaseSpeed 20
#define rightMotorBaseSpeed 20
#define min_speed -30
#define max_speed 50
float error, errorSum, errorOld;  // Variables for the PID loop
float leftMotorSpeed, rightMotorSpeed; // Variables to hold the current motor speed (+-100%)
float Kp, Ki, Kd; // Variables to store the PID constants

void setup()
{
  delay(100);
 Wire.begin(); // join i2c bus (address optional for master)
 Serial.begin(9600); // start serial for output
 t = 0;
 Serial.println("Recording black at 25, white at 50");
}
void loop()
{
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

if(millis() < 5100){
  Serial.println(millis()/100);
  // Counting during calibration
}
else{
  printData();
  Serial.println(weightedAverage(data));
}

//Callibrate at specific times
 if(millis()>2500 && millis()<2600){
    for(int i=0; i<=7; i++){
      blackdata[i] = data[2*i];
   }
   Serial.print("Recorded black values as ");
   printData();
 }
 if(millis()>5000 && millis()<5100){
    for(int i=0; i<=7; i++){
      whitedata[i] = data[2*i];
   }
   Serial.print("Recorded white values as ");
   printData();
 }
 
 float output = PID(weightedAverage(data)); // Calculate the PID output.

  leftMotorSpeed = leftMotorBaseSpeed + output;     // Calculate the modified motor speed
  rightMotorSpeed = rightMotorBaseSpeed - output;

  // Apply new speed and direction to each motor
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
//==============

// Function to calculate the PID output. This should probably return a long instead of a float...
float PID(long lineDist)
{
  // PID loop
  errorOld = error;        // Save the old error for differential component
  error = 51.5 - lineDist;  // Calculate the error in position
  errorSum += error;

  float proportional = error * Kp;  // Calculate the components of the PID
  
  float integral = errorSum * Ki;
 
  float differential = (error - errorOld) * Kd;

  long output = proportional + integral + differential;  // Calculate the result

  return output;
}

// To be completed as part of the task
// Function to allow the user to change the PID constants using ....?
// K Value that is changing should be displayed on the LCD screen
/*
void updatePID(void)
{
  robot.print("#Hb"); // Halt both motors so it's easier to change the values

  // Store the current PID constants

  
  // Update PID Constant
    
 
  // Display the changing constant on the LCD

}
*/


