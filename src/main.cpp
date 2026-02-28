#include <Wire.h>

#include <Payload.hpp>

void setup() {
    Serial.begin(Payload::SERIAL_NUM);
    Wire.begin();
}

void loop() {
    static Payload payload;
    payload.tick();
}
