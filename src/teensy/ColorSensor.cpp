#include "teensy/ColorSensor.hpp"
#include <iostream>

ColorSensor::ColorSensor(float integration_time, Gain gain)
    : m_integration_time(integration_time)
    , m_gain(gain) {}

ColorSensor::ColorSensor(
    float integration_time,
    Gain gain,
    std::function<void()> before_update
)
    : m_integration_time(integration_time)
    , m_gain(gain)
    , mf_before_update(before_update) {}

void ColorSensor::mf_ensure_init() {
    if (m_ready) return;
    if (!attach()) {
        Serial.print("ColorSensor.cpp:20:0: unable to attach to wire.");
        return;
    }

    integrationTime(m_integration_time);
    this->gain(m_gain);
    m_ready = true;
}

ColorSensor::Color ColorSensor::current_color() {
    mf_before_update();
    mf_ensure_init();
    return color();
}
