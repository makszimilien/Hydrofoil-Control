#include "ADS1X15.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

// RC servo
Servo elevator1;
// Min 50, max 130, mid 90
int servoPin = 4;

// PID controller
double setpoint, input, output;
// Target is a setpoint value, sets the nominal sinking of the vehicle
double target = 128;
int kp = 2;
int ki = 5;
int kd = 1;
PID elevator1Pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// External ADC and measurement
ADS1115 waterLvl(0x48);

long sensorLongValue = 0;
long sensorShortValue = 0;
long sensorAVG = 0;

void waterLvlReady();
volatile bool ready = false;
int readyPin = 1;
bool selector = false;

// Cycle time
unsigned long prevTime = 0;
unsigned long currTime = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("Serial started");

  elevator1.attach(servoPin);
  elevator1.write(90);

  elevator1Pid.SetMode(AUTOMATIC);

  Wire.setClock(400000);

  // ADC setup
  waterLvl.begin();
  waterLvl.setGain(0);     // 6.144 volt
  waterLvl.setDataRate(7); // 0 = slow   4 = medium   7 = fast

  // Set alert for ready interrupt
  waterLvl.setComparatorThresholdHigh(0x8000);
  waterLvl.setComparatorThresholdLow(0x0000);
  waterLvl.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  pinMode(readyPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(readyPin), waterLvlReady, RISING);

  waterLvl.setMode(0);
  waterLvl.requestADC(0);
}

void loop() {
  if (ready) {
    if (!selector) {
      waterLvl.requestADC(0); // Request a measurement on A0
      sensorLongValue = waterLvl.getValue();
      selector = true;
      Serial.println("Selector: True");
      Serial.print("Long: ");
      Serial.println(sensorLongValue);
    }

    else {
      waterLvl.requestADC(1);
      sensorShortValue = waterLvl.getValue();
      selector = false;
      Serial.println("Selector: False");
      Serial.print("Short: ");
      Serial.println(sensorShortValue);
    }
  }
  delay(5);

  // sensorAVG = (sensorAVG + sensorValue) / 2;

  input = map(sensorAVG, 0, 1023, 0, 255);
  setpoint = target;
  elevator1Pid.Compute();

  // For debugging only
  // delay(500);
  // Serial.println("*******");
  // Serial.println(sensorAVG);

  // Measuring cycle time
  currTime = millis();
  // Serial.println(currTime - prevTime);
  prevTime = currTime;

  // elevator1.write(map(output, 0, 255, 50, 130));
}

//  interrupt service routine
void waterLvlReady() { ready = true; }