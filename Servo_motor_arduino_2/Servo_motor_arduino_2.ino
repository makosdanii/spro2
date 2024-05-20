#include <Servo.h> // Arduino Servo library to run standard servo motors 

Servo the_servo; // creates the servo object named "the_servo" with help of the library 
// -> responsible for for controlling one servo once attached to an output pin and commands are sent to it 

int servo_position = 0; // global variable that allows control of the servo's position 

void setup() { // put your setup code here, to run once:
  
  the_servo.attach (6); // attaches the PWM signal pin (here chose pin 6) to the servo object
  the_servo.write (0);

}

void loop() { // put your main code here, to run repeatedly: -> essentially the "while (1) loop" is vis.studio code 
  
  for (servo_position = 0; servo_position <= 180; servo_position ++) { // roatates the servo motor with increments of 1 degree
    the_servo.write (servo_position); // rotates the servo motor by the angle of servo_position
    delay (100); // delay in ms to wait for the servo motor to get into position 
  }

  delay (1000); 
  the_servo.write (0); 
  delay (2000); 
  
}

