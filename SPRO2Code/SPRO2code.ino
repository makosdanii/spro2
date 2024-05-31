#include <AS5048A.h>
#include <Linear2DRegression.hpp>
#include <Servo.h>
#include "HX711.h"
#define TORQUE_THRESHOLD 0.01

#define I_OFFSET 508.7
#define I_SCALE -0.06944

#define ROTOR_RADIUS 0.04

Linear2DRegression *Exp1Fit = new Linear2DRegression();
Linear2DRegression *Exp2Fit = new Linear2DRegression();

Servo actuator;

HX711 torqueSensor;

float experiment1Current[7] = {0.93, 0.95, 0.99, 1.03, 1.1, 1.12, 1.13};
float experiment1VELOCITY[7] = { 41.86, 71.38, 96.61, 120.02, 157.49, 204.95, 230};


uint8_t dataPin = 3;
uint8_t clockPin = 2;


float E = 6;
float R = 0.35;

float Af = 0;
float Bf = 0;

float At = 0;
float Bt = 0;

float Kt = 0;
float Kb = 0;


AS5048A angleSensor(10, false);

long I_Sum;
float I = 0;

float IAvg = 0;

int I_n = 0;

float torqueOffset = 0;



void setup()
{
  Serial.begin(115200);
  Serial.println("MOTOR TEST STAND");
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
  //  Serial.print("A1: ");
  //  Serial.println(varF, 6);
  //  Serial.print("B1: ");
  //  Serial.println(constF, 6);
  Serial.println(Af);
  float timeTo = Experiment3();
  Experiment1();


  Experiment2();
  float constF = Bf * Kt;
  float varF = Af * Kt;
  float J = -(Af + (Kb * Kt / R)) * timeTo / log(0.1);
  Serial.print("Moment of Inertia:  ");
  Serial.println(J, 6);

  Serial.print("Constant friction:  ");
  Serial.println(constF, 6);
  Serial.print("Variable friction:  ");
  Serial.println(varF, 6);
}

void loop()
{



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

void Experiment1() {

  float velocity = 0;
  float current = 0;
  for (int i = 0; i < 5; i++) {
    motorFullSpeed();
    delay(6000);
 
    current = get_Current();
    velocity = velocity_average(20);
    Serial.print(current);
    Serial.print("\t");
    Serial.println(velocity);
    Exp1Fit->addPoint(velocity, current);

    motorSlowSpeed();
    delay(6000);
 
    current = get_Current();
    velocity = velocity_average(20);
    Serial.print(current);
    Serial.print("\t");
    Serial.println(velocity);
    Exp1Fit->addPoint(velocity, current);


  }
  Bf = Exp1Fit->calculate(0);
  Af = (Exp1Fit->calculate(1)) - Bf;

  Serial.println("LINE: ");
  Serial.println(Af, 6);
  Serial.println(Bf, 6);
  Exp1Fit->reset();
}

void Experiment2() {
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
    for (int j = 0; j < 200; j++) {
      t = get_Torque(1);
      v = velocity_average(1);
      Serial.print(t, 5);
      Serial.print("\t");
      Serial.println(v);
      Exp2Fit->addPoint(v, t);
    }
  }

  Bt = Exp2Fit->calculate(0);
  At = (Exp2Fit->calculate(1)) - Bt;
  Serial.println("LINE FIT:");
  Serial.print(At, 5);
  Serial.print("  ");
  Serial.println(Bt, 5);

  Kt = (Bt) / ((E / R) - Bf);
  Kb = -R * ((At / Kt) + Af);
  Serial.print("Kt: ");
  Serial.print(Kt, 6);
  Serial.println("Nm/A");
  Serial.print("Kb :");
  Serial.print(Kb, 6);
  Serial.println("V/(rad*s)");

  motorOff();
  actuator.write(180);

}

float Experiment3() {

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
    Serial.print(debug);
    Serial.print("\t");
    Serial.println(expoV[i]);
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

  Serial.print("TIME TO 90%:  ");
  float timeTo = expoTime[expoIndex] / 1000000.0;
  Serial.println(timeTo);
  return timeTo;
}

void ExperimentA() {
  for (int i = 0; i < 7; i++) {
    Exp1Fit->addPoint(experiment1VELOCITY[i], experiment1Current[i]);
  }
  Bf = Exp1Fit->calculate(0);
  Af = (Exp1Fit->calculate(1)) - Bf;
  Exp1Fit->reset();
}
