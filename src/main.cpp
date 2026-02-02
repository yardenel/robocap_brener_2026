#include <Arduino.h>

#include <vector>

#include "Payload.hpp"

Payload& get_payload() {
    static Payload payload;
    return payload;
}

void setup() {
    get_payload().initialize();
    std::vector<int> a(4, 7);
}

void loop() { get_payload().tick(); }
