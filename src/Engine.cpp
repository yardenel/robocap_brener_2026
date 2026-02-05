#include "Engine.hpp"

#include <Arduino.h>

Engine::Engine(
    uint8_t io_forward_pin,
    uint8_t io_backward_pin,
    uint8_t pwm_pin,
    float pwm_freq
)
    : io_fwd(io_forward_pin)
    , io_bkwd(io_backward_pin)
    , pwm(pwm_pin)
    , m_pwm_freq(pwm_freq)
    , m_cur_speed(0) {
    pinMode(io_fwd, OUTPUT);
    pinMode(pwm, OUTPUT);
    analogWriteFrequency(pwm, m_pwm_freq);
}

void Engine::set_speed(int16_t speed) {
    if (speed == m_cur_speed) return;
    if (!speed) {
        digitalWrite(io_fwd, LOW);
        digitalWrite(io_fwd, LOW);
        analogWrite(pwm, 0);
    } else if (speed > 0) {
        if (m_cur_speed <= 0) {
            digitalWrite(io_fwd, HIGH);
            digitalWrite(io_bkwd, LOW);
            analogWrite(pwm, speed);
        }
    } else {
        if (m_cur_speed >= 0) {
            digitalWrite(io_fwd, LOW);
            digitalWrite(io_bkwd, HIGH);
            analogWrite(pwm, -1 * speed);
        }
    }

    m_cur_speed = speed;
}
