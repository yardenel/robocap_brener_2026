#include <ColorSensor.hpp>

ColorSensor::ColorSensor(float integration_time, Gain gain) {
    if (!attach()) {
        Serial.print("ColorSensor.cpp:4:0: unable to attach to wire.");
        return;
    }

    if (!available()) {
        Serial.print("ColorSensor.cpp:5:0: color sensor unavailable.");
        return;
    };

    integrationTime(integration_time);
    this->gain(gain);
}
