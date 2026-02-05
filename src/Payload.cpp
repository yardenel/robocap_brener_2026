#include "Payload.hpp"

#include <Arduino.h>

#include "Mux.hpp"

Payload::Payload()
    : m_mux(MUX_SIG, MUX_S0, MUX_S1, MUX_S2, MUX_S3) {
    Serial.begin(SERIAL_NUM);
}

void Payload::tick() {}
