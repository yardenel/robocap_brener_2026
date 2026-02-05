#include "Payload.hpp"

#include <Arduino.h>

#include "Mux.hpp"

Payload::Payload()
    : m_mux(MUX_SIG, MUX_S0, MUX_S1, MUX_S2, MUX_S3)
    , lf(LF_FORWARD, LF_BACKWARD, LF_PWM, ENGINES_FREQUENCY)
    , rf(RF_FORWARD, RF_BACKWARD, RF_PWM, ENGINES_FREQUENCY)
    , lr(LR_FORWARD, LR_BACKWARD, LR_PWM, ENGINES_FREQUENCY)
    , rr(RR_FORWARD, RR_BACKWARD, RR_PWM, ENGINES_FREQUENCY) {
    Serial.begin(SERIAL_NUM);
}

void Payload::tick() {}
