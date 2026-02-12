#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

class Mux {
   private:
    static constexpr const size_t CHANNEL_COUNT = 16;
    const uint8_t sig, s0, s1, s2, s3;
    std::array<int, CHANNEL_COUNT> m_io_arr;

    void select_channel(uint8_t idx) const;
    void update_io_arr();

   public:
    Mux(uint8_t sig_pin,
        uint8_t s0_pin,
        uint8_t s1_pin,
        uint8_t s2_pin,
        uint8_t s3_pin);

    int read_at(uint8_t idx) const;
    const std::array<int, CHANNEL_COUNT>& get_io_arr();
    Mux()  = delete;
    ~Mux() = default;
};
