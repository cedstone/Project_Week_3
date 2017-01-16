#include <Wire.h>
#define uchar unsigned char
uchar t;
//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
uchar data[16];
uchar blackdata[8];
uchar whitedata[8];
void setup()
{
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
 data[t] = Wire.read(); // receive a byte as character
 if (t < 15)
 t++;
 else
 t = 0;
 }

//printData();
if(millis() < 5000){
  Serial.println(millis()/100);
}

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
 
 delay(500);
}

void printData(){
 for(int i=0; i<=14; i+=2){
  Serial.print(data[i]);
  Serial.print(" ");
 }
 Serial.println("");
}

