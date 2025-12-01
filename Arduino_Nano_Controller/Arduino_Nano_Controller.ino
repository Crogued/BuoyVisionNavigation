#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
 
RF24 radio(9, 10);                // CE, CSN
const byte Address[6] = "00001";

// pins
const int switchPin   = 3;        // toggle Manual <-> Automatic
const int ledManual   = 5;
const int ledAuto     = 6;
const int joy12_pinY  = A2;       // joystick M1/M2 - originally Y axis
const int joy12_pinX  = A1;       // joystick M1/M2 - originally X axis
const int joy3_pinY   = A0;       // joystick M3 - Y axis

// central dead-zone
const int DZ_MIN      = 510;
const int DZ_MAX      = 530;
 
// speed limits
const int MC_MIN      = 1000;
const int MC_MAX      = 2000;
 
int lastSwitchState   = HIGH;
 
void setup() {
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(ledManual,   OUTPUT);
  pinMode(ledAuto,     OUTPUT);
 
  Serial.begin(19200);
  // initial header
  Serial.println(F("===================================="));
  Serial.println(F(" Starting USV Controller RF24 V1 "));
  Serial.println(F("   Motors: 1000 (reverse), 1500 (stop), 2000 (forward)"));
  Serial.println(F("===================================="));
  Serial.println();
 
  radio.begin();
  radio.openWritingPipe(Address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
 
void loop() {
  int sw = digitalRead(switchPin);
 
  // only send mode code when switch changes
  if (sw != lastSwitchState) {
    uint16_t codeModo = (sw == LOW) ? 40100 : 40200;
    radio.write(&codeModo, sizeof(codeModo));
    Serial.println(F("------------------------------------"));
    Serial.print(F(">> Mode: "));
    Serial.print((sw == LOW) ? F("MANUAL") : F("AUTOMATIC"));
    Serial.print(F("    ["));
    Serial.print(codeModo);
    Serial.println(F("]"));
    Serial.println(F("------------------------------------"));
    lastSwitchState = sw;
    digitalWrite(ledManual, sw == LOW ? HIGH : LOW);
    digitalWrite(ledAuto,   sw == HIGH? HIGH : LOW);
    delay(100);  // debouncing
  }
 
  // in manual mode, read joysticks and send commands to 3 motors
  if (sw == LOW) {
    // - joystick M1/M2 rotated 90 deg CW -
    int oldY = analogRead(joy12_pinY);
    int oldX = analogRead(joy12_pinX);
    int rawX = oldY;  // now X axis read from "original Y"
    int rawY = oldX;  // now Y axis read from "original X"
 
    // apply dead-zone
    if (rawX > DZ_MIN && rawX < DZ_MAX) rawX = (DZ_MIN+DZ_MAX)/2;
    if (rawY > DZ_MIN && rawY < DZ_MAX) rawY = (DZ_MIN+DZ_MAX)/2;
 
    // base and rotation calculation
    int base  = map(rawY, 0, 1023, MC_MAX, MC_MIN);   // push for forward/backward
    int turn  = map(rawX, 0, 1023, +200, -200);       // rotate on own axis
 
    int m1val = constrain(base + turn, MC_MIN, MC_MAX);
    int m2val = constrain(base - turn, MC_MIN, MC_MAX);
 
    // motor 3 (single Y axis)
    int raw3 = analogRead(joy3_pinY);
    if (raw3 > DZ_MIN && raw3 < DZ_MAX) raw3 = (DZ_MIN+DZ_MAX)/2;
    int m3val = map(raw3, 0, 1023, MC_MAX, MC_MIN);
    m3val = constrain(m3val, MC_MIN, MC_MAX);
 
    // send via RF24
    int code1 = 10000 + m1val;
    int code2 = 20000 + m2val;
    int code3 = 30000 + m3val;
    radio.write(&code1, sizeof(code1));
    radio.write(&code2, sizeof(code2));
    radio.write(&code3, sizeof(code3));
 
    // aesthetic prints
    Serial.println(F("---- MANUAL CONTROL ----"));
    Serial.print(F(" M1 [1000-2000]: ")); Serial.print(m1val);
    Serial.print(F("   D: ")); Serial.println(m1val - 1500);
    Serial.print(F(" M2 [1000-2000]: ")); Serial.print(m2val);
    Serial.print(F("   D: ")); Serial.println(m2val - 1500);
    Serial.print(F(" M3 [1000-2000]: ")); Serial.print(m3val);
    Serial.print(F("   D: ")); Serial.println(m3val - 1500);
    Serial.println(F("-------------------------"));
  }
 
  //delay(500); // Delay for testing
}
 