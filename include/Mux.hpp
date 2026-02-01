#pragma once
#include <Arduino.h>

class Mux {
   private:
    const uint8_t sig, s0, s1, s2, s3;

   public:
    Mux() = delete;
    Mux(const uint8_t sig_pin,
        const uint8_t s0_pin,
        const uint8_t s1_pin,
        const uint8_t s2_pin,
        const uint8_t s3_pin);

    void update();
};
