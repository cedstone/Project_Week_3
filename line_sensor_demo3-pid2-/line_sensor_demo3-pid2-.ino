/* Code for following lines using Sunfounder reflective sensor array
    UoN FoE, EEE Dept.
    by Group 14 (Eric Larkins):
    Yu Hei Gordon Wong
    Daniel Maclaren
    Kenny Clarke
    and Christopher Stone
    January 2017
*/

#define uchar unsigned char   // Not really important, but saves typing
#include <Wire.h>             // I2C library for talking to sunfounder module over
#include <LiquidCrystal.h>    // Drives LCD using parallel interface
#include <SoftwareSerial.h>   // Use pins other than 0 and 1 to command motor controller board

LiquidCrystal lcd(12, 11, 6, 5, 4, 3);  // Pins used to connect LCD
SoftwareSerial robot(8, 7);             // Pins used to connect motor controller

uchar t;                      // Array index for reflectivity data
uchar incomingvalue;          // The most recently recieved buye
uchar data[16];               // The bytes recieved from the line-following sensor
uchar blackdata[] = {0, 0, 0, 0, 0, 0, 0, 0};                     // The lower callibration bounds (will be changed later)
uchar whitedata[] = {255, 255, 255, 255, 255, 255, 255, 255};     //  ...upper...

#define button0 9             // Which pins are our buttons connected to?
#define button1 10
#define button2 13
int editing = 0;              // Which of the PID constants are we editing at the moment? (0, 1 or 2)
#define adjustStep 0.1        // Amount by which to increment or decrement the PID constants

long unsigned int encoderValue[] = {0, 0};
long unsigned int encoderValueOld[] = {0, 0};
int encoderDistance = 0;
int distanceCentimetres = 0;


// PID coefficients (can be changed during run)
#define Ku 2.24
float Kp = 0.6; //0.8*Ku;            // 2.0 works o r0.4
float Ki = 0.05;               // large values cause continuous spinning
float Kd = 1.0; //125*Ku;          // 0.5 works, so do negative values?
// Variables used in PID function (These should probably not be global)
int errorOld = 0;           // The previous error value, used to calculate the derivative
int error = 0;              // How far are we from the line? (from weighted average)
int errorSum = 0;           // Sum of past errors (apprximates intergral)
float pidoutput = 0;          // Result of PID calculation

float leftMotorSpeed = 0;     // Self explanatory
float rightMotorSpeed = 0;
#define min_speed -20
#define max_speed 30
#define leftMotorBaseSpeed 20 // Default speed (if going straight forward)
#define rightMotorBaseSpeed 20

boolean go = true;

void setup()
{
  lcd.begin(16, 2);                  // 2 rows, 16 columns
  pinMode(button0, INPUT_PULLUP);    // Use arduino's built-in pullups to save on wiring
  pinMode(button1, INPUT_PULLUP);

  Wire.begin();                      // Start I2C bus for talking to SunFounder module
  Serial.begin(9600);                // Open serial link to PC (over USB)
  robot.begin(57600);                // Open serial link to motor controller
  callibrate();                      // Prompt user to place on white, black surfaces
}

void loop()
{
  getRawData();                      // Fetch data from line sensor
  handleButtons();                   // Check if any of the buttons have been pressed, and react accordingly
  updateDisplay();                   // Send error and PID values to LCD
  setMotorSpeed();                   // perform WA, and use PID to choose motor speeds
  updateDistance();
  //printData();
  //delay(200);

}

void updateDistance(){
   encoderValue[0] = readEncoder(1);
   encoderValue[1] = encoderValue[0];   // Hack because encoders interfere with each other magnetically
   //encoderValue[1] = readEncoder(2);
   Serial.print(encoderValue[0]);
   Serial.print(", ");
   Serial.println(encoderValue[1]);
   if (rightMotorSpeed > 0){
     encoderDistance -= (encoderValueOld[0] - encoderValue[0]);
   }
   else {
     encoderDistance += (encoderValueOld[0] - encoderValue[0]);
   }
   if (leftMotorSpeed > 0){
     encoderDistance -= (encoderValueOld[1] - encoderValue[1]);
   }
   else {
     encoderDistance += (encoderValueOld[1] - encoderValue[1]);
   }
  distanceCentimetres = encoderDistance / 1.7; 
  encoderValueOld[0] =  encoderValue[0];
  encoderValueOld[1] =  encoderValue[1];
}

long unsigned int readEncoder (int side) {
  long unsigned int encoder = 0;
  if (side == 1){
     robot.write("#e1");
  }
  else{
    robot.write("#e2");
  }
  // wait breifly for response
  delay(50);
  encoder = robot.read();
  encoder += (robot.read()<<8);
  encoder += (robot.read()<<16);
  encoder += (robot.read()<<24);
  //Serial.println(encoder);
  return encoder;
}


void printData()
{
  // Send constrained and ranged line sensor data to PC
  for (int i = 0; i <= 14; i += 2)
  {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println("");
}

int weightedAverage(uchar data[])
{
  // Find the position, right to left, of the line - now with 100% fewer floats!
  int sum1 = 0;
  int sum2 = 0;
  int result = 0;
  for (int i = 0; i <= 14; i += 2){
    sum1 += (i * (255 - data[i]));
  }
  for (int i = 0; i <= 14; i += 2){
    sum2 += (255 - data[i]);
  }
  result = sum1 / sum2;
  result *= 18;     // map to range
  result -= 127;    // zero is central
  return result;
}

void setMotorSpeed(){
  pidoutput = PID((long)weightedAverage(data));
  leftMotorSpeed = leftMotorBaseSpeed + pidoutput;
  rightMotorSpeed = rightMotorBaseSpeed - pidoutput;
  if(go){
    if (leftMotorSpeed > 0){
      leftMotorSpeed = constrain(leftMotorSpeed, 0, max_speed);
      robot.write("#D1f");
      robot.write("#S1");
      robot.print((int)leftMotorSpeed);
    }
    else if (leftMotorSpeed < 0){
      leftMotorSpeed = constrain(leftMotorSpeed, min_speed, 0);
      robot.write("#D1r");
      robot.write("#S1");
      robot.print(-(int)leftMotorSpeed);
    }
    if (rightMotorSpeed > 0){
      rightMotorSpeed = constrain(rightMotorSpeed, 0, max_speed);
      robot.write("#D2f");
      robot.write("#S2");
      robot.print((int)rightMotorSpeed);
    }
    else if (rightMotorSpeed < 0){
      rightMotorSpeed = constrain(rightMotorSpeed, min_speed, 0);
      robot.write("#D2r");
      robot.write("#S2");
      robot.print(-(int)rightMotorSpeed);
    }
  }
}

void handleButtons(void){
  if (digitalRead(button0) == LOW){
    if (editing == 0) {
      Kp += adjustStep;
    }
    if (editing == 1) {
      Ki += adjustStep;
    }
    if (editing == 2) {
      Kd += adjustStep;
    }
    updateDisplay();
    delay(100);
  }
  if (digitalRead(button1) == LOW){
    if (editing == 0) {
      Kp -= adjustStep;
    }
    if (editing == 1) {
      Ki -= adjustStep;
    }
    if (editing == 2) {
      Kd -= adjustStep;
    }
    updateDisplay();
    delay(100);
  }
  if (digitalRead(button2) == LOW){
    editing++;
    if (editing >= 3) {
      editing = 0;
    }
    updateDisplay();
    robot.write("#hb");
    go = !go;
    delay(500);
  }
}

void updateDisplay(void){
  /*
  lcd.setCursor(0, 0);
  lcd.print("Error = ");
  lcd.print(weightedAverage(data));
  lcd.print("        ");
  */
  lcd.setCursor(0, 0);
  lcd.print("Distance = ");
  lcd.print(distanceCentimetres);
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.setCursor(1, 1);
  lcd.print("p");
  lcd.print((int)(Kp * 10));
  lcd.print("   ");
  lcd.setCursor(7, 1);
  lcd.print("i");
  lcd.print((int)(Ki * 10));
  lcd.print("   ");
  lcd.setCursor(12, 1);
  lcd.print("d");
  lcd.print((int)(Kd * 10));
  lcd.print("          ");
  int pointerPos = 0;
  if (editing == 0) {
    pointerPos = 0;
  }
  if (editing == 1) {
    pointerPos = 6;
  }
  if (editing == 2) {
    pointerPos = 11;
  }
  lcd.setCursor(pointerPos, 1);
  lcd.print(">");
}

void getRawData(void){
  // Read reflectivity values from line-following array
  t = 0;
  Wire.requestFrom(9, 16); // request 16 bytes from slave device #9
  while (Wire.available()) // slave may send less than requested
  {
    incomingvalue = Wire.read();   // Read value
    // Constrain and map value to fit between the limits of calibration values
    // The "/2" terms are there because the odd-numbered bytes of raw data are useless, and not recorded in blackdata and whitedata
    data[t] = constrain(incomingvalue, blackdata[t / 2], whitedata[t / 2]);
    data[t] = map(data[t], blackdata[t / 2], whitedata[t / 2], 0, 255);
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
  while (digitalRead(button0) == HIGH) {};
  for (int i = 0; i <= 7; i++) {
    getRawData();
    blackdata[i] = data[2 * i];
  }
  Serial.print("Recorded black values as ");
  printData();
  Serial.println("Expose to white surface, then push white button");
  lcd.clear();
  lcd.print("* Callibration *");
  lcd.setCursor(0, 1);
  lcd.print("    white...    ");
  while (digitalRead(button1) == HIGH) {};
  for (int i = 0; i <= 7; i++) {
    getRawData();
    whitedata[i] = data[2 * i];
  }
  Serial.print("Recorded white values as ");
  printData();
  lcd.clear();
  lcd.print("* Callibration *");
  lcd.setCursor(0, 1);
  lcd.print("      done     ");
  while (digitalRead(button2) == HIGH) {};
  delay(1000);
}

float PID(long error){
  error = - error;
  errorSum = (errorSum + error) * 0.95;
  float proportional = (float)error * Kp;
  float integral = errorSum * Ki;
  //integral = constrain(integral, -1000, 1000);
  float differential = (error - errorOld) * Kd;
  long output = proportional + integral + differential;
  errorOld = error;
  return output;
}

