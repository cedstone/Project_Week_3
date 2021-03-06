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
SoftwareSerial robot(6, 7);             // Pins used to connect motor controller

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

// PID coefficients (can be changed during run)
float Kp = 2.5;   // 2.0 works
float Ki = 0.0;   // non-zero values cause continuous spinning
float Kd = 0.0;   // 0.5 works
// Variables used in PID function (These should probably not be global)
float errorOld = 0;           // The previous error value, used to calculate the derivative
float errorNew = 0;           // The new error value...
float error = 0;              // How far are we from the line? (from weighted average)
float errorSum = 0;           // Sum of past errors (apprximates intergral)
float pidoutput = 0;          // Result of PID calculation

float leftMotorSpeed = 0;     // Self explanatory
float rightMotorSpeed = 0;
#define min_speed -25
#define max_speed 25
#define leftMotorBaseSpeed 20 // Default speed (if going straight forward)
#define rightMotorBaseSpeed 20

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

void setMotorSpeed()
{
  pidoutput = PID((long)weightedAverage(data));
  leftMotorSpeed = leftMotorBaseSpeed + pidoutput;
  rightMotorSpeed = rightMotorBaseSpeed - pidoutput;

  if (leftMotorSpeed > 0)
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
  if (rightMotorSpeed > 0)
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


int weightedAverage(uchar data[])
{
  // Find the position, right to left, of the line
  float sum1 = 0;
  float sum2 = 0;
  float result = 0;
  for (int i = 0; i <= 14; i += 2)
  {
    sum1 += (i * (255 - data[i]));
  }
  for (int i = 0; i <= 14; i += 2)
  {
    sum2 += (255 - data[i]);
  }
  result = sum1 / sum2;
  result *= ((float)255 / 14);
  result -= 127;
  return (int)result;
}


void handleButtons(void)
{
  if (digitalRead(button0) == LOW)
  {
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
  if (digitalRead(button1) == LOW)
  {
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
  if (digitalRead(button2) == LOW) {
    editing++;
    if (editing >= 3) {
      editing = 0;
    }
    updateDisplay();
    delay(500);
  }
}

void updateDisplay(void)
{
  lcd.setCursor(0, 0);
  lcd.print("Error = ");
  lcd.print(weightedAverage(data));
  lcd.print("        ");
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

void getRawData(void)
{
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

void callibrate()
{
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
  delay(3000);
}

float PID(long errorNew)
{
  errorNew *= -1;
  errorOld = error;        // Save the old error for differential component
  error = errorNew;        // Calculate the error in position
  errorSum += error;
  float proportional = error * Kp;
  float integral = errorSum * Ki;
  float differential = (error - errorOld) * Kd;
  long output = proportional + integral + differential;
  return output;
}

