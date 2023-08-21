#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

Servo elevator1;
// Min 50, max 130, mid 90
int servoInput = 0;
int servoPin = 2;

int sensorValue = 0;
int sensorPin = 28;
int sensorAVG = 0;

int pwmPin = 3;
unsigned long pwmRead = 0;
int pwmValue = 0;
float control = 0;
int factor = 20; // %, sets the effects of the PWM input on the PID setpoint

double setpoint, input, output;
// target is a setpoint value, sets the nominal sinking of the vehicle
double target = 50;
int kp = 2;
int ki = 5;
int kd = 1;
PID elevator1Pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(pwmPin, INPUT);

  Serial1.begin(115200);
  Serial1.println("Serial started");

  elevator1.attach(servoPin);
  elevator1.write(90);

  setpoint = 128; // Controls the sinking to the point where the water level is
                  // at the middle of the sensor
  elevator1Pid.SetMode(AUTOMATIC);
}

void loop() {
  sensorValue = analogRead(sensorPin);
  sensorAVG = (sensorAVG + sensorValue) / 2;
  pwmRead = pulseIn(pwmPin, HIGH);
  if (pwmRead >= 990 && pwmRead <= 2010) {
    pwmValue = map(pwmRead, 990, 2010, 0, 255);
    control = (pwmValue - 128) / 100.0 * factor;
  } else
    control = 0;

  input = map(sensorAVG, 0, 1023, 0, 255);
  setpoint = target + control;
  elevator1Pid.Compute();

  // Serial1.println(pwmRead);
  // Serial1.println(pwmValue - 128);
  // Serial1.println(control);
  Serial1.println(setpoint);
  Serial1.println(input);
  Serial1.println(output);
  Serial1.println("*******");
  delay(500); // For serial pringing only

  elevator1.write(map(output, 0, 255, 50, 130));
}
