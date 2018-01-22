#include<stdlib.h>

void setup()
{
 Serial.begin(9600);
 Serial.println("Starting");
}

void loop()
{
  char buf[10];
  float temp = -0.51;
  dtostrf(temp,3,1,buf);
}
