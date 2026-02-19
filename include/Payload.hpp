#pragma once

#include <Engine.hpp>
#include <I2CMuxTCA.hpp>
#include <Mux.hpp>
#include <Vec2f.hpp>

class Payload {
   private:
    static constexpr const int SERIAL_NUM = 115200;
    static constexpr const int MUX_S0     = 31;
    static constexpr const int MUX_S1     = 30;
    static constexpr const int MUX_S2     = 29;
    static constexpr const int MUX_S3     = 28;
    static constexpr const int MUX_SIG    = 32;

    static constexpr const int ENGINES_FREQUENCY = 20000;

    /// NOTE: the color sensor's addr is 0x29

    /// TODO: update these values to be correct.
    static constexpr const int LF_FORWARD = 13, LF_BACKWARD = 41, LF_PWM = 23;
    static constexpr const int LR_FORWARD = 38, LR_BACKWARD = 35, LR_PWM = 37;
    static constexpr const int RF_FORWARD = 40, RF_BACKWARD = 39, RF_PWM = 22;
    static constexpr const int RR_FORWARD = 34, RR_BACKWARD = 33, RR_PWM = 36;

    static constexpr const int TCA_ADDR = 0x70;

    I2CMuxTCA m_tcamux;
    Mux m_mux;
    Engine lf, rf, lr, rr;
    Vec2f m_pos;

   public:
    Payload();
    ~Payload() = default;

    void tick();
};
