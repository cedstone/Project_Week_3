#include <Wire.h>
#define uchar unsigned char
uchar t;
uchar incomingvalue;
//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
uchar data[16];
uchar blackdata[] = {0, 0, 0, 0, 0, 0, 0, 0};
uchar whitedata[] = {255, 255, 255, 255, 255, 255, 255, 255};
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

if(millis() < 5000){
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
 
 delay(500);
}

void printData(){
 for(int i=0; i<=14; i+=2){
  Serial.print(data[i]);
  Serial.print(" ");
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

