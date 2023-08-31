#include "ADS1X15.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

Servo elevator1;
// Min 50, max 130, mid 90
int servoPin = 4;

long sensorLongValue = 0;
long sensorShortValue = 0;
long sensorAVG = 0;

double setpoint, input, output;
// Target is a setpoint value, sets the nominal sinking of the vehicle
double target = 128;
int kp = 2;
int ki = 5;
int kd = 1;
PID elevator1Pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// External ADC
ADS1115 waterLvl(0x48);

unsigned long prevTime = 0;
unsigned long currTime = 0;
int counter = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("Serial started");

  elevator1.attach(servoPin);
  elevator1.write(90);

  elevator1Pid.SetMode(AUTOMATIC);

  waterLvl.begin();
  waterLvl.setGain(0);     // 6.144 volt
  waterLvl.setDataRate(4); // 0 = slow   4 = medium   7 = fast
  waterLvl.requestADC(0);
}

void loop() {

  waterLvl.requestADC(0); // request a new one
  sensorLongValue = waterLvl.getValue();
  waterLvl.requestADC(1); // request a new one
  sensorShortValue = waterLvl.getValue();
  Serial.print("\tLong: ");
  Serial.print(sensorLongValue);
  Serial.print("\tShort: ");
  Serial.print(sensorShortValue);
  delay(500);

  // sensorAVG = (sensorAVG + sensorValue) / 2;

  input = map(sensorAVG, 0, 1023, 0, 255);
  setpoint = target;
  elevator1Pid.Compute();

  // For debugging only
  // delay(500);
  // Serial.println("*******");
  // Serial.println(sensorAVG);

  // For measuring cycle time
  // currTime = millis();
  // Serial.println(currTime - prevTime);
  // prevTime = currTime;

  // elevator1.write(map(output, 0, 255, 50, 130));
}
