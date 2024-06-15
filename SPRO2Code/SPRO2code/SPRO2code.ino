#include <AS5048A.h>
#include <Linear2DRegression.hpp>
#include <Servo.h>
#include <Arduino.h>
#include "HX711.h"
#define TORQUE_THRESHOLD 0.01
#define NUMBER_STRING 1001

#define I_OFFSET 508.7
#define I_SCALE -0.06944

#define NO_DATA_POINTS 200

#define ROTOR_RADIUS 0.04

Linear2DRegression *Exp1Fit = new Linear2DRegression();
Linear2DRegression *Exp2Fit = new Linear2DRegression();

Servo actuator;

HX711 torqueSensor;

float measure_motor_constantsCurrent[7] = {0.93, 0.95, 0.99, 1.03, 1.1, 1.12, 1.13};
float measure_motor_constantsVELOCITY[7] = { 41.86, 71.38, 96.61, 120.02, 157.49, 204.95, 230};


uint8_t dataPin = 3;
uint8_t clockPin = 2;


float E = 6;
float R = 0.35;

float Af = 0;
float Bf = 0;

float At, Bt;

float Kt, Kb;

float J, t;

float v_start, v_end, t_start, t_end; // for nextion graph

AS5048A angleSensor(10, false);

long I_Sum;
float I = 0;

float IAvg = 0;

int I_n = 0;

float torqueOffset = 0;

void setup()
{
  Serial.begin(9600);
  delay(5000);

  print_nextion("page 0");

  // Serial.print("page1.x0.val=");
  // print_nextion_float(1.0); //Printing the value of Kt

  //Serial.println("MOTOR TEST STAND");
  torqueSensor.begin(dataPin, clockPin);
  torqueSensor.set_offset(141007);
  torqueSensor.set_scale(1989.465332);
  torqueOffset = torqueSensor.get_units(10);

  

  actuator.attach(7);
  actuator.write(180);

  angleSensor.begin();

  pinMode(A3, INPUT);
  pinMode(6, OUTPUT);
  At = -0.00056;
  Bt = 0.16138;
  Kt = (Bt) / ((E / R) - Bf);
  Kb = -R * ((At / Kt) + Af);

  float timeTo = moment_of_inertia();
  measure_motor_constants();

  measure_torque();
  float constF = Bf * Kt;
  float varF = Af * Kt;
  J = -(varF + (Kb * Kt / R)) * timeTo / log(0.1);

  print_nextion("page 1");

  Serial.print("page2.x2.val=");
  long debug_Kb = (long)(Kb*1000);
  print_nextion_float(debug_Kb);

  Serial.print("page2.x3.val=");
  long debug_Kt = (long)(Kt*1000);
  print_nextion_float(debug_Kt); // Printing the value of Kt

  Serial.print("page2.x4.val=");
  long debug_J = (long)(J*10000);
  print_nextion_float(debug_J); //Printing of moment of inertia
  Serial.print(J);

  //Serial.print("Constant friction:  ");
  //Serial.println(constF, 6);
  //Serial.print("Variable friction:  ");
  //Serial.println(varF, 6);
}

void loop()
{

  // print_nextion("page2");
  // delay(5000);
  // print_nextion("page1");
  // delay(5000);

  // // measure_motor_constants();
  // // moment_of_inertia();
  // // measure_torque();

  // Serial.print("page2.x2.val=");
  // long debug_Kb = (long)(Kb*1000000);
  // print_nextion(debug_Kb); // Printing the value of Kb


  // Serial.print("page2.x3.val=");
  // long debug_Kt = (long)(Kt*1000000);
  // print_nextion(debug_Kt); // Printing the value of Kt


  // Serial.print("page2.x4.val=");
  // long debug_J = (long)(J*10000000);
  // print_nextion(debug_J); //Printing of moment of inertia



}

void motorFullSpeed() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, 1);
  digitalWrite(7, 1);
}

void motorSlowSpeed() {
  digitalWrite(6, 0);
  pinMode(6, INPUT);

  pinMode(7, OUTPUT);
  digitalWrite(7, 1);
}

void motorOff() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, 0);
  digitalWrite(7, 0);
}

float velocity_average(int repeats) {
  int current_repeat = 0;
  float velocityAvg = 0;
  bool endOfRotation = 0;
  float angularVelocity = 0;

  float initialVal = angleSensor.getRotationInRadians();
  float val = 5;
  while (val < (2 * 3.1415) - 0.1) {
    val = angleSensor.getRotationInRadians();
  }

  long long lastTime = micros();
  while (current_repeat < repeats) {
    val = angleSensor.getRotationInRadians();

    if (val < 0.2) {
      endOfRotation = 1;
    }
    if (endOfRotation && (val > (2 * 3.1415) - 0.2)) {
      endOfRotation = 0;

      angularVelocity = 1000000 * (2 * 3.1415) / (micros() - lastTime);

      velocityAvg += angularVelocity;
      lastTime = micros();
      current_repeat++;
    }

  }
  return velocityAvg / repeats;
}

float velocity_instant(int interval) {
  long long startTime = micros();
  float startAngle = angleSensor.getRotationInRadians();
  while (micros() - startTime < interval) {
  }

  float delta = angleSensor.getRotationInRadians() - startAngle;

  if (delta > 0) {
    delta = (2 * 3.1415) - delta;
  }
  delta = abs(delta);
  return delta;
}


float get_Current() {
  float I = 0;
  for (int i = 0; i < 20000; i ++) {
    I += analogRead(A3);
  }

  return ((I / 20000.0) - I_OFFSET) * I_SCALE;
}


float get_Torque(int repeats) {
  float reading = -(torqueSensor.get_units(repeats) - torqueOffset);
  reading = (reading / 1000) * 9.81;
  reading *= ROTOR_RADIUS;

  return reading;
}

void measure_motor_constants() {
  float velocity = 0;
  float current = 0;

  for (int i = 0; i < 5; i++) {
    motorFullSpeed();
    delay(6000);
 
    current = get_Current();
    velocity = velocity_average(20);
    // Serial.print(current);
    // Serial.print("\t");
    Serial.print("page1.x1.val=");
    print_nextion_float(velocity); //Printing velocity
    //Serial.println(velocity);
    Exp1Fit->addPoint(velocity, current);

    motorSlowSpeed();
    delay(6000);
 
    current = get_Current();
    velocity = velocity_average(20);
    //Serial.print(current);
    //Serial.print("\t");
    //Serial.println(velocity);
    Serial.print("page1.x1.val=");
    print_nextion_float(velocity); //Printing velocity
    Exp1Fit->addPoint(velocity, current);


  }
  Bf = Exp1Fit->calculate(0);
  Af = (Exp1Fit->calculate(1)) - Bf;

  // Serial.println("LINE: ");
  // Serial.println(Af, 6);
  // Serial.println(Bf, 6);
  Exp1Fit->reset();
}

void measure_torque() {
  motorFullSpeed();
  delay(2000);
  //Serial.println("EXP 2");
  int currentAngle = 180;

  int steps = 4;
  int stepDelta = 20;

  while (currentAngle > -1) {
    actuator.write(currentAngle);
    if (get_Torque(1) > TORQUE_THRESHOLD) {
      break;
    }
    currentAngle--;
    delay(200);
  }

  actuator.write(180);
  //Serial.println("HIT");
  currentAngle = 180;
  for (int i = 0; i < steps; i++) {
    currentAngle -= stepDelta;
    actuator.write(currentAngle);
    delay(5000);
    float v = 0;
    float t = 0;
    for (int j = 0; j < NO_DATA_POINTS; j++) {
      t = get_Torque(1);
      v = velocity_average(1);
      if (j == 0){
        v_start = v;
        t_start = t;
      }
      if (j == NO_DATA_POINTS - 1){
        v_end = v;
        t_end = t;
      }
        // Printing the values
      Serial.print("page1.x0.val=");
      //long debug_t = (long)(t*1000);
      print_nextion_float(t);// Printing the value of Force
      
      Serial.print("page1.x1.val=");
      //long debug_v = (long)(v*100);
      print_nextion_float(v); // Printing the value of Speed

      Exp2Fit->addPoint(v, t);
    }
  }

  Bt = Exp2Fit->calculate(0);
  At = (Exp2Fit->calculate(1)) - Bt;
  //Serial.println("LINE FIT:");
  //Serial.print(At, 5);
  //Serial.print("  ");
  //Serial.println(Bt, 5);

  Kt = (Bt) / ((E / R) - Bf);
  Kb = -R * ((At / Kt) + Af);
  //Serial.print("Kt: ");
  //Serial.print(Kt, 6);
  //Serial.println("Nm/A");
  //Serial.print("Kb :");
  //Serial.print(Kb, 6);
  //Serial.println("V/(rad*s)");

  motorOff();
  actuator.write(180);

}

float moment_of_inertia() {
  float vLast = 0;
  float expoV[100];
  long long expoTime[100];
  motorOff();
  delay(6000);
  motorFullSpeed();
  long long expoStartTime = micros();

  for (int i = 0; i < 100; i++) {

    expoV[i] = velocity_average(1);
    expoTime[i] = micros() - expoStartTime;
    long debug = expoTime[i];
    //Serial.print(debug);
    //Serial.print("\t");
    //Serial.println(expoV[i]);
  }

  float finalVelocity = velocity_average(30);
  motorOff();

  float expoThreshold = 0.9;

  float deltaToThreshold = 1;
  int expoIndex = 0;

  for (int i = 0; i < 100; i++) {
    if (abs((expoV[i] / finalVelocity) - expoThreshold) < deltaToThreshold) {
      deltaToThreshold = abs((expoV[i] / finalVelocity) - expoThreshold);
      expoIndex = i;
    }
  }

  //Serial.print("TIME TO 90%:  ");
  float timeTo = expoTime[expoIndex] / 1000000.0;
  //Serial.println(timeTo);
  return timeTo;
}

void ExperimentA() {
  for (int i = 0; i < 7; i++) {
    Exp1Fit->addPoint(measure_motor_constantsVELOCITY[i], measure_motor_constantsCurrent[i]);
  }

  Bf = Exp1Fit->calculate(0);
  Af = (Exp1Fit->calculate(1)) - Bf;
  Exp1Fit->reset();
}

void GetValue(const char* pagename, uint32_t* value) {
    char readBuffer[100];
    uint32_t readValue = 0;
    int typeExpected = 0;

    // print_nextion("get " + pagename + ".val"); // Send the get command.
    _delay_ms(20);

    for (int i = 0; i < 8; i++) {
//        Serial.read("%c", &readBuffer[i]);
        if (readBuffer[i] == 0x71) { // Expect number string.
            typeExpected = NUMBER_STRING;
            readBuffer[0] = 0x71; // Move indicator to front, just to keep the nice format.
            break;
        }
    }

    if (typeExpected == NUMBER_STRING) {
        for (int i = 1; i < 8; i++) {
//            Serial.read("%c", &readBuffer[i]);
        }

        if (readBuffer[0] == 0x71 && readBuffer[5] == 0xFF && readBuffer[6] == 0xFF && readBuffer[7] == 0xFF) { // This is a complete number return.
            readValue = readBuffer[1] | (readBuffer[2] << 8) | (readBuffer[3] << 16) | (readBuffer[4] << 24);
        }
    }

    *value = readValue;
}


void print_nextion(char* text){
  Serial.print(text); 
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}

void print_nextion_float(float value){
  long value_long = value * 1000;
  Serial.print(value_long); 
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
}
