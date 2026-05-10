#include "teensy/Payload.hpp"

#include <Arduino.h>

#include "teensy/I2CMuxTCA.hpp"
#include "teensy/Mux.hpp"
#include <iostream>
#include <thread>

Payload::Payload()
    : m_csensor(50, ColorSensor::Gain::X01, [&]() { m_tcamux.select(1); })
    , m_tcamux(TCA_ADDR)
    , m_mux(MUX_SIG, MUX_S0, MUX_S1, MUX_S2, MUX_S3)
    , lf(LF_FORWARD, LF_BACKWARD, LF_PWM, ENGINES_FREQUENCY)
    , rf(RF_FORWARD, RF_BACKWARD, RF_PWM, ENGINES_FREQUENCY)
    , lr(LR_FORWARD, LR_BACKWARD, LR_PWM, ENGINES_FREQUENCY)
    , rr(RR_FORWARD, RR_BACKWARD, RR_PWM, ENGINES_FREQUENCY) {
    Serial.begin(SERIAL_NUM);
}

double Payload::dt() const { return m_dt.count(); }

void Payload::finish_tick() {
    auto now = m_clk::now();
    m_dt     = now - m_lastnow;
    if (m_dt < TARGET_CYCLE) {
        delay((TARGET_CYCLE - m_dt).count() * 1000);
        now  = m_clk::now();
        m_dt = now - m_lastnow;
    }

    m_lastnow = now;
}

void Payload::tick() {
    auto cur_clr = m_csensor.current_color();

    std::cout << "[" << cur_clr.r << ", " << cur_clr.g << ", " << cur_clr.b << "]\n";
    std::cout << "elapsed: " << dt() << "\n";

    finish_tick();
}
