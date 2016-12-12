#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

long unsigned int encoderValue1 = 0;
long unsigned int encoderValue2 = 0;
const int m = 134;  // one metre

void setup() {
  Serial.begin(57600);  // Connection to the motore-controller board
  delay(1000);           // Wait for the car to be put down on the floor
  Serial.print("#Sb000,100");   // 25% speed on each motor
  Serial.print("#d1f");         // both motors forward
  Serial.print("#d2f");
  lcd.begin(16, 2);
}

void loop() {
  encoderValue1 = readEncoder(1);
  encoderValue2 = readEncoder(2);
  lcd.setCursor(0, 0);
  lcd.print(encoderValue1);
  lcd.print("  ");
  lcd.print(encoderValue2);
  
  if(encoderValue1 < 10*m)
  {
      Serial.print("#d1f");
      Serial.print("#d2f");
  }
  
  else
  {
    Serial.print("#hb");      // Stop
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

