#include <Wire.h>

#include "teensy/Payload.hpp"

void setup() {
    Serial.begin(Payload::SERIAL_NUM);
    Wire.begin();
}

void loop() {
    Serial.println("WORKS");
    static Payload payload;
    payload.tick();
}
