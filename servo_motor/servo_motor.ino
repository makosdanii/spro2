#include "HX711.h"

HX711 myScale;

uint8_t dataPin = 6;
uint8_t clockPin = 7;

uint32_t start, stop;
volatile float f;


void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("LIBRARY VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

  myScale.begin(dataPin, clockPin);
  myScale.set_offset(141007); 
  myScale.set_scale(1989.465332);
}

void loop()
{
  float mass =  myScale.get_units(5);
  Serial.println(mass);
  //calibrate();
}
