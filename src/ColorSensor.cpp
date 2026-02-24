#include <ColorSensor.hpp>

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
        Serial.print("ColorSensor.cpp:4:0: unable to attach to wire.");
        return;
    }
    if (!available()) {
        Serial.print("ColorSensor.cpp:5:0: color sensor unavailable.");
        return;
    };

    integrationTime(m_integration_time);
    this->gain(m_gain);
}
