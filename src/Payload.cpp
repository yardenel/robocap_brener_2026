#include "Payload.hpp"

#include <Arduino.h>

void Payload::initialize() {
    Serial.begin(SERIAL_NUM);
    pinMode(muxS0, OUTPUT);
    pinMode(muxS1, OUTPUT);
    pinMode(muxS2, OUTPUT);
    pinMode(muxS3, OUTPUT);
    pinMode(mux_SIG, INPUT);
}

void Payload::tick() {}
