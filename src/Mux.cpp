#include "Mux.hpp"

Mux::Mux(
    const uint8_t sig_pin,
    const uint8_t s0_pin,
    const uint8_t s1_pin,
    const uint8_t s2_pin,
    const uint8_t s3_pin
)
    : sig(sig_pin)
    , s0(s0_pin)
    , s1(s1_pin)
    , s2(s2_pin)
    , s3(s3_pin) {}
