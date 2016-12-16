long unsigned int encoderValue = 0;
const int m = 130;  // one metre
const int ra = 37;  // a 90 degree turn

void setup() {
  Serial.begin(57600);  // Connection to the motore-controller board
  delay(500);           // Wait for the car to be put down on the floor
  Serial.print("#Sb025,025");   // 25% speed on each motor
  Serial.print("#d1f");         // both motors forward
  Serial.print("#d2f");
}

void loop() {
  encoderValue = readEncoder();
  if(encoderValue <= 3*m)
  {
      Serial.print("#d1f");
      Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)) && encoderValue >  (3*m))
  {
      Serial.print("#d1r");
      Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)) && encoderValue >  (3*m)+(ra/2))
  {
      Serial.print("#d1f");
      Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)+ra) && encoderValue >  ((3*m)+(ra/2)+(1.7*m)))
  {
      Serial.print("#d1r");
      Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)+ra+m) && encoderValue > ((3*m)+(ra/2)+(1.7*m)+ra))
  {
    Serial.print("#d1f");
    Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m))  && encoderValue > ((3*m)+(ra/2)+(1.7*m)+ra+m))
  {
      Serial.print("#Sb050,020");
      Serial.print("#d1f");
      Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)+(1.8*m)) && encoderValue > ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)))
  {
    Serial.print("#Sb025,025");
    Serial.print("#d1f");
    Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)+(1.8*m)+ra) && encoderValue > ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)+(1.8*m)))
  {
    Serial.print("#d1r");
    Serial.print("#d2f");
  }
  else if(encoderValue <= ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)+(1.8*m)+ra+(2*m)) && encoderValue > ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)+(1.8*m)+ra))
  {
    Serial.print("#d1f");
    Serial.print("#d2f");
  }
  else if(encoderValue >= ((3*m)+(ra/2)+(1.7*m)+ra+m+(1.35*m)+(1.8*m)+ra+(2*m)))
  {
  Serial.print("#hb");
  }
}
      
long unsigned int readEncoder () {
  long unsigned int encoder = 0;
  
  // request the encoder value for motor 1 (assuming 1=2)
  Serial.write("#e1");
  
  // wait breifly for response
  delay(50);

  encoder = Serial.read();
  encoder += (Serial.read()<<8);
  encoder += (Serial.read()<<16);
  encoder += (Serial.read()<<24);
  return encoder;
}

