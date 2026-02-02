#include "Payload.hpp"

#include <Arduino.h>

#include "Mux.hpp"

Payload::Payload()
    : m_mux(mux_SIG, mux_S0, mux_S1, mux_S2, mux_S3) {
    Serial.begin(SERIAL_NUM);
}

void Payload::tick() {}
