#pragma once

#include "Mux.hpp"
#include "Vec2f.hpp"

class Payload {
   private:
    static constexpr const int SERIAL_NUM = 9600;
    static constexpr const int muxS0      = 23;
    static constexpr const int muxS1      = 22;
    static constexpr const int muxS2      = 21;
    static constexpr const int muxS3      = 20;
    static constexpr const int mux_SIG    = 24;

    Mux m_mux;
    Vec2f m_pos;

   public:
    ~Payload() = default;

    void initialize();
    void tick();
};
