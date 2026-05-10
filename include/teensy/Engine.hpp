#pragma once
#include <Arduino.h>

class Engine {
   private:
    const uint8_t io_fwd, io_bkwd, pwm;
    const float m_pwm_freq;
    int16_t m_cur_speed = 0;

   public:
    Engine(
        uint8_t io_forward_pin,
        uint8_t io_backward_pin,
        uint8_t pwm_pin,
        float pwm_freq
    );

    void set_speed(int16_t speed);

    Engine()  = delete;
    ~Engine() = default;
};
