#pragma once
#include <CsenseTCS34725.hpp>

class ColorSensor : public TCS34725 {
   private:
    const float m_integration_time;
    const Gain m_gain;
    bool m_ready                                 = false;
    const std::function<void()> mf_before_update = []() {};

    void mf_ensure_init();

   public:
    ColorSensor(float integration_time, Gain gain);
    ColorSensor(float integration_time, Gain gain, std::function<void()>);
    Color current_color();
};
