#include "HX711.h" // includes the library for the load cell 

// set-up of the pins 
const int DT_pin_DOUT = 3; // "const" means the pins cannot be changed PD3
const int SCK_pin_CLK = 2; // PD2

// variables
int load_cell_reading; 

HX711 scale; // from the library ? 

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600); // the BAUDRATE
  scale.begin (DT_pin_DOUT, SCK_pin_CLK); // using the library and the two pins connected to the load cell
  scale.set_scale ();
  scale.tare ();
  æ
}

void loop() {
  // put your main code here, to run repeatedly:
  if (scale.is_ready()) {
    scale.set_scale (); // using the library 
    Serial.println ("remove any weight from the load cell and tare"); // message to be printed in the serial monitor ("ln" adds a new line)
    delay (5000); // delay of 5 seconds, so any weight can be removed 
    scale.tare (); // using the library to tare the load cell 
    Serial.println ("place know weight on the scale");
    delay(5000); 
    load_cell_reading = scale.get_units(10); // result of averages 10 readings, subtracts the current OFFSET, divides by the current SCALE factor
    Serial.println ("load cell reading: "); 
    Serial.print (load_cell_reading); 
  } 
  else {
    serial.println ("No Load Cell connected"); 
  }
}
