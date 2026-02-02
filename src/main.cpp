#include <Arduino.h>

#include "Payload.hpp"

Payload& get_payload() {
    static Payload payload;
    return payload;
}

void setup() {}

void loop() { get_payload().tick(); }
