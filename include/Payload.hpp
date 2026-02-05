#pragma once

#include "Engine.hpp"
#include "Mux.hpp"
#include "Vec2f.hpp"

class Payload {
   private:
    static constexpr const int SERIAL_NUM = 9600;

    static constexpr const int mux_S0  = 23;
    static constexpr const int mux_S1  = 22;
    static constexpr const int mux_S2  = 21;
    static constexpr const int mux_S3  = 20;
    static constexpr const int mux_SIG = 24;

    /// TODO: update these values to be correct.
    static constexpr const int LF_FORWARD = 30, LF_BACKWARD = 29, LF_PWM = 25;
    static constexpr const int LR_FORWARD = 31, LR_BACKWARD = 35, LR_PWM = 26;
    static constexpr const int RF_FORWARD = 32, RF_BACKWARD = 44, RF_PWM = 27;
    static constexpr const int RR_FORWARD = 33, RR_BACKWARD = 45, RR_PWM = 28;

    Mux m_mux;
    Engine lf, rf, lr, rr;
    Vec2f m_pos;

   public:
    Payload();
    ~Payload() = default;

    void tick();
};
