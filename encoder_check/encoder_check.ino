#include <LiquidCrystal.h>
#include <SoftwareSerial.h>   // Use pins other than 0 and 1 to command motor controller board
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);  // Pins used to connect LCD
SoftwareSerial robot(8, 7);             // Pins used to connect motor controller
long unsigned int encoderValue1 = 0;
long unsigned int encoderValue2 = 0;

void setup() {
  robot.begin(57600);  // Connection to the motore-controller board
  delay(1000);           // Wait for the car to be put down on the floor
  robot.print("#Sb020,020");   // 25% speed on each motor
  lcd.begin(16, 2);
}

void loop() {
  encoderValue1 = readEncoder(1);
  encoderValue2 = readEncoder(2);
  lcd.setCursor(0, 0);
  lcd.print(encoderValue1);
  lcd.print("          ");
  lcd.setCursor(0, 1);
  lcd.print(encoderValue2);
  lcd.print("          ");
  delay(100);
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
  return encoder;
}

