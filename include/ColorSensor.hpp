#pragma once
#include <CsenseTCS34725.hpp>

class ColorSensor : TCS34725 {
   public:
    ColorSensor(float integration_time, Gain gain);
};
