/*
Copyright (c) 2023 Keith E Henry

MIT License

The DSerial.print statements are for use with Teleplot, a PlatformIO extension.
*/

#define DEBUG false  // Set to true for debug output, false for no debug output
#define DSerial \
    if (DEBUG) Serial

#include <Arduino.h>

// For the RC Receiver:
// PWM should be trimmed for a center point of 1.5ms
const int PWMTrim = 1500;  // Nominal PWM center value in ms
const int PWMRng = 400;    // Nominal PWM range is 500, mine is less

const int PWMUno = 255;  // Arduino Uno's PWM range
const int PWMMin = 20;   // Dead zone; motor just vibrates

// Controls and functions are mapped using a mental image of the
//  standard unit circle, with x horizonal, y vertical,
//  positive to the right and up respectively.

// Pin definitions
// RC receiver inputs:
const byte AilPin = 2;  // Aileron - hardware interrupt
const byte ElePin = 3;  // Elevator - ditto
// Left motor:
const byte L1APin = 8;   // Logic - reverse
const byte L2APin = 9;   // Logic - forward
const byte LEnPin = 10;  // PWM - rate
// Right motor:
const byte REnPin = 11;  // PWM - rate
const byte R2APin = 12;  // Logic - reverse
const byte R1APin = 13;  // Logic - forward

// All DX4e servo reversing switches are OFF.
// Globals for ISRs:
unsigned long AilValTmp;
unsigned long EleValTmp;
volatile int AilVal;
volatile int EleVal;

// ISRs:
void AilValISR() {
    int microsTmp;
    if (digitalRead(AilPin))
        AilValTmp = micros();
    else {
        microsTmp = micros() - AilValTmp - PWMTrim;
        if (abs(microsTmp) <= PWMRng)  // If didn't miss an interrupt somehow
            AilVal = -microsTmp;       // Aileron is Reversed
    }
}

void EleValISR() {
    int microsTmp;
    if (digitalRead(ElePin))
        EleValTmp = micros();
    else {
        microsTmp = micros() - EleValTmp - PWMTrim;
        if (abs(microsTmp) <= PWMRng)  // If didn't miss an interrupt somehow
            EleVal = microsTmp;
    }
}

void setup() {
    // RC receiver
    // Two inputs with ISRs:
    pinMode(AilPin, INPUT);
    pinMode(ElePin, INPUT);
    attachInterrupt(digitalPinToInterrupt(AilPin), AilValISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ElePin), EleValISR, CHANGE);

    // L293D:
    pinMode(L2APin, OUTPUT);
    pinMode(L1APin, OUTPUT);
    pinMode(LEnPin, OUTPUT);
    pinMode(REnPin, OUTPUT);
    pinMode(R1APin, OUTPUT);
    pinMode(R2APin, OUTPUT);

    AilVal = 0;
    EleVal = 0;

    DSerial.begin(9600);
    DSerial.println("Setup Complete");
}

void loop() {
    // Capture volatile joystick positions
    int EleVal0 = EleVal;
    int AilVal0 = AilVal;

    // Compute the amount by which to slow down the turning wheel
    float angle = atan2(EleVal0, AilVal0);
    float offset = abs(sin(angle));

    /*
    DSerial.print(">EleVal0:");
    DSerial.println(EleVal0);
    DSerial.print(">AilVal0:");
    DSerial.println(AilVal0);
    DSerial.print(">angle():");
    DSerial.println(angle);
    DSerial.print(">offset:");
    DSerial.println(offset);
    */

    // If turning to the Right, slow down the Right wheel, and visa versa.
    float LMotor, RMotor;
    if (AilVal0 >= 0) {
        LMotor = (float)EleVal0 / PWMRng;
        RMotor = LMotor * offset;
    } else {
        RMotor = (float)EleVal0 / PWMRng;
        LMotor = RMotor * offset;
    }

    DSerial.print(">LMotor:");
    DSerial.println(LMotor);
    DSerial.print(">RMotor:");
    DSerial.println(RMotor);

    /*
    // Map for MX1919 dual motor driver module:
    // Left motor
    unsigned int LInA = abs(max(0, LMotor)) * PWMUno;
    unsigned int LInB = abs(min(0, LMotor)) * PWMUno;
    // Right motor
    unsigned int RInA = abs(max(0, RMotor)) * PWMUno;
    unsigned int RInB = abs(min(0, RMotor)) * PWMUno;

    DSerial.print(">LInA:");
    DSerial.println(LInA);
    DSerial.print(">LInB:");
    DSerial.println(LInB);

    DSerial.print(">RInA:");
    DSerial.println(RInA);
    DSerial.print(">RInB:");
    DSerial.println(RInB);
  */

    // Map for L293D dual motor driver chip:
    bool L1A = (LMotor > 0) ? HIGH : LOW;
    bool L2A = !L1A;
    unsigned int LEn = min(abs(LMotor * PWMUno), PWMUno);
    DSerial.print(">L1A:");
    DSerial.println(L1A);
    DSerial.print(">L2A:");
    DSerial.println(L2A);
    DSerial.print(">LEn:");
    DSerial.println(LEn);

    bool R1A = (RMotor > 0) ? HIGH : LOW;
    bool R2A = !R1A;
    unsigned int REn = min(abs(RMotor * PWMUno), PWMUno);
    DSerial.print(">R1A:");
    DSerial.println(R1A);
    DSerial.print(">R2A:");
    DSerial.println(R2A);
    DSerial.print(">REn:");
    DSerial.println(REn);

    if (DEBUG) delay(500);

    digitalWrite(L1APin, L1A);
    digitalWrite(L2APin, L2A);
    if (LEn > PWMMin)
        analogWrite(LEnPin, LEn);
    else
        digitalWrite(LEnPin, LOW);

    digitalWrite(R1APin, R1A);
    digitalWrite(R2APin, R2A);

    if (REn > PWMMin)
        analogWrite(REnPin, REn);
    else
        digitalWrite(REnPin, LOW);
}
