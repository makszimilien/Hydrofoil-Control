#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

Servo elevator1;
// Min 50, max 130, mid 90
int servoInput = 0;
int servoPin = 2;

// Min 390, max 430
int sensorValue = 0;
int sensorPin = 28;
int sensorAVG = 0;

int pwmPin = 3;
unsigned long pwmValue = 0;

double setpoint, input, output;
int kp = 2;
int ki = 10;
int kd = 2;
PID elevator1Pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(pwmPin, INPUT);

  Serial1.begin(115200);
  Serial1.println("Serial started");

  elevator1.attach(servoPin);
  elevator1.write(90);

  setpoint = 15;
  elevator1Pid.SetMode(AUTOMATIC);
}

void loop() {
  // if (Serial1.available() > 0) {
  //   servoInput = Serial1.parseInt();
  // }
  // Serial1.println(servoInput);
  // elevator1.write(servoInput);

  sensorValue = (analogRead(sensorPin) - 390) / ((430 - 390) / 20);
  sensorAVG = (sensorAVG + sensorValue) / 2;
  // Serial1.println(sensorAVG);
  // delay(500);

  // pwmValue = pulseIn(pwmPin, HIGH);
  // Serial1.println(pwmValue);

  input = sensorAVG;
  elevator1Pid.Compute();
  Serial1.println(output);
}
