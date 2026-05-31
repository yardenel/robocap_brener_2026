#include <Wire.h>
#include "teensy/Payload.hpp"
#include "FSM.h"

FSM fsm;

void setup() {
    // Serial.begin(Payload::SERIAL_NUM);
    pinMode(35, OUTPUT);
    Wire.begin();
    fsm.begin();
}

void loop() {
    digitalWrite(35, HIGH);
    // Serial.println("WORKS");
    // static Payload payload;
    // payload.tick();
    fsm.update();
}
