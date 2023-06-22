/*
Copyright (c) 2023 Keith E Henry

MIT License
*/

#include <Arduino.h>

// PWM should be trimmed for a center point of 1.5ms
#define PWMTrim 1500. // Nominal PWM center value
#define PWMRng 500.   // Nominal PWM range
#define PWMUno 256    // Arduino Uno's PWM range

// Controls and functions are mapped using a mental image of the
//  standard unit circle, with x horizonal, y vertical,
//  positive to the right and up respectively.

// Pin definitions:
const byte AilPin = 2; // Aileron
const byte ElePin = 3; // Elevator
                       // All DX4e servo reversing switches are OFF.
// Globals for ISRs:
unsigned long AilValTmp;
unsigned long EleValTmp;
volatile float AilVal;
volatile float EleVal;

// ISRs:
void AilValISR()
{
  if (digitalRead(AilPin))
    AilValTmp = micros();
  else
    AilVal = -(micros() - AilValTmp - PWMTrim) / PWMRng; // Reversed
}

void EleValISR()
{
  if (digitalRead(ElePin))
    EleValTmp = micros();
  else
    EleVal = (micros() - EleValTmp - PWMTrim) / PWMRng;
}

void setup()
{
  // Two inputs with ISRs:
  pinMode(AilPin, INPUT);
  pinMode(ElePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(AilPin), AilValISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ElePin), EleValISR, CHANGE);

  Serial.begin(9600);
}

void loop()
{
  // Snapshot volatile coordinates
  float EleVal0 = EleVal;
  float AilVal0 = AilVal;

  // Compute the amount by which to slow down the turning wheel
  float angle = atan2(EleVal0, AilVal0);
  float offset = abs(sin(angle));

  // If turning to the Right, slow down the Right wheel, and visa versa.
  float LMotor, RMotor;
  if (AilVal0 >= 0)
  {
    LMotor = EleVal0;
    RMotor = EleVal0 * offset;
  }
  else
  {
    LMotor = EleVal0 * offset;
    RMotor = EleVal0;
  }

  // Map for MX1919 dual motor driver module:
  // Left motor
  unsigned int LInA = abs(max(0, LMotor)) * PWMUno;
  unsigned int LInB = abs(min(0, LMotor)) * PWMUno;
  // Right motor
  unsigned int RInA = abs(max(0, RMotor)) * PWMUno;
  unsigned int RInB = abs(min(0, RMotor)) * PWMUno;

  // Use Teleplot, an extension for PlatformIO, to view:
  /*
  Serial.print(">EleVal0:");
  Serial.println(EleVal0);
  Serial.print(">AilVal0:");
  Serial.println(AilVal0);
  Serial.print(">angle():");
  Serial.println(angle);
  Serial.print(">offset:");
  Serial.println(offset);
  */

  Serial.print(">LMotor:");
  Serial.println(LMotor);
  Serial.print(">RMotor:");
  Serial.println(RMotor);

  Serial.print(">LInA:");
  Serial.println(LInA);
  Serial.print(">LInB:");
  Serial.println(LInB);

  Serial.print(">RInA:");
  Serial.println(RInA);
  Serial.print(">RInB:");
  Serial.println(RInB);

  delay(500);
}
