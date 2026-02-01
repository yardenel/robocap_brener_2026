#include <Arduino.h>

#include "Payload.hpp"

Payload& get_payload() {
    static Payload payload;
    return payload
}

void setup() { get_payload().initialize(); }

void loop() { get_payload().tick(); }
