#pragma once

#include <Vec2f.hpp>
#include <chrono>
#include <teensy/ColorSensor.hpp>
#include <teensy/Engine.hpp>
#include <teensy/I2CMuxTCA.hpp>
#include <teensy/Mux.hpp>

class Payload {
   private:
    using m_clk = std::chrono::steady_clock;

    static constexpr int MUX_S0  = 31;
    static constexpr int MUX_S1  = 30;
    static constexpr int MUX_S2  = 29;
    static constexpr int MUX_S3  = 28;
    static constexpr int MUX_SIG = 32;

    static constexpr int ENGINES_FREQUENCY = 20000;

    /// NOTE: the color sensor's addr is 0x29

    /// TODO: update these values to be correct.
    static constexpr int LF_FORWARD = 13, LF_BACKWARD = 41, LF_PWM = 23;
    static constexpr int LR_FORWARD = 38, LR_BACKWARD = 35, LR_PWM = 37;
    static constexpr int RF_FORWARD = 40, RF_BACKWARD = 39, RF_PWM = 22;
    static constexpr int RR_FORWARD = 34, RR_BACKWARD = 33, RR_PWM = 36;

    static constexpr int TCA_ADDR = 0x70;

    static constexpr double TARGET_TPS{20.0};
    static constexpr std::chrono::duration<double> TARGET_CYCLE{1.0 / TARGET_TPS};

    ColorSensor m_csensor;
    I2CMuxTCA m_tcamux;
    Mux m_mux;
    Engine lf, rf, lr, rr;
    Vec2f m_pos;
    m_clk::time_point m_lastnow{m_clk::now()};
    std::chrono::duration<double> m_dt{TARGET_CYCLE};

   public:
    static constexpr const int SERIAL_NUM = 115200;

    Payload();
    ~Payload() = default;

    void tick();
    void finish_tick();

    double dt() const;
};
