#include <Arduino.h>
#include <HX711.h>
#include <PID_v1.h>
#include <Servo.h>

Servo elevator1;
// Min 50, max 130, mid 90
int servoPin = 4;

long sensorValue = 0;
int sensorPin = 18;
long sensorAVG = 0;

int pwmPin = 5;
unsigned long pwmRead = 0;
int pwmValue = 0;
float control = 0;
int factor = 20; // %, sets the effects of the PWM input on the PID setpoint

double setpoint, input, output;
// target is a setpoint value, sets the nominal sinking of the vehicle
double target = 128; // Controls the sinking to the point where the water level
                     // is at the middle of the sensor
int kp = 2;
int ki = 5;
int kd = 1;
PID elevator1Pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

const int waterLvl1Dout = 2;
const int waterLvl1Sck = 3;
HX711 waterLvl1;

unsigned long prevTime = 0;
unsigned long currTime = 0;
int counter = 0;

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(pwmPin, INPUT);

  Serial.begin(115200);
  Serial.println("Serial started");

  elevator1.attach(servoPin);
  elevator1.write(90);

  elevator1Pid.SetMode(AUTOMATIC);

  waterLvl1.begin(waterLvl1Dout, waterLvl1Sck);
}

void loop() {
  sensorValue = waterLvl1.read();
  sensorAVG = (sensorAVG + sensorValue) / 2;

  // To modify setpoint value based on PWM input
  // pwmRead = pulseIn(pwmPin, HIGH, 40000);
  // if (pwmRead >= 990 && pwmRead <= 2010) {
  //   pwmValue = map(pwmRead, 990, 2010, 0, 255);
  //   control = (pwmValue - 128) / 100.0 * factor;
  // } else
  //   control = 0;

  input = map(sensorAVG, 0, 1023, 0, 255);
  setpoint = target + control;
  elevator1Pid.Compute();
  elevator1Pid.Compute();
  elevator1Pid.Compute();

  // Serial.println(pwmRead);
  // Serial.println(pwmValue - 128);
  // Serial.println(control);
  // Serial.println(setpoint);
  // Serial.println(input);
  // Serial.println(output);
  // Serial.println("*******");
  // delay(500); // For serial printing only

  // For measuring cycle time
  // currTime = millis();
  // Serial.println(currTime - prevTime);
  // prevTime = currTime;

  elevator1.write(map(output, 0, 255, 50, 130));
}
