#pragma once

#include "Mux.hpp"
#include "Vec2f.hpp"

class Payload {
   private:
    static constexpr const int SERIAL_NUM = 9600;
    static constexpr const int mux_S0     = 23;
    static constexpr const int mux_S1     = 22;
    static constexpr const int mux_S2     = 21;
    static constexpr const int mux_S3     = 20;
    static constexpr const int mux_SIG    = 24;

    Mux m_mux;
    Vec2f m_pos;

   public:
    Payload();
    ~Payload() = default;

    void tick();
};
