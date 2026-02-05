#pragma once
#include <Arduino.h>

class Engine {
   private:
    const uint8_t io_pin, pwm_pin;
    const size_t pwm_freq;

   public:
    Engine(uint8_t io_pin, uint8_t pwm_pin, size_t pwm_freq);
    void set_speed(int16_t speed) const;

    Engine()  = delete;
    ~Engine() = default;
};
