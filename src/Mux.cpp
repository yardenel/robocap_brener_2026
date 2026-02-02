#include "Mux.hpp"

#include <Arduino.h>

Mux::Mux(
    uint8_t sig_pin,
    uint8_t s0_pin,
    uint8_t s1_pin,
    uint8_t s2_pin,
    uint8_t s3_pin
)
    : sig(sig_pin)
    , s0(s0_pin)
    , s1(s1_pin)
    , s2(s2_pin)
    , s3(s3_pin) {
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);

    pinMode(sig, INPUT);
}

void Mux::select_channel(uint8_t idx) const {
    digitalWrite(s0, idx & 1);
    digitalWrite(s1, (idx >> 1) & 1);
    digitalWrite(s2, (idx >> 2) & 1);
    digitalWrite(s3, (idx >> 3) & 1);
}

void Mux::update_io_arr() {
    for (uint8_t idx(0); idx < CHANNEL_COUNT; ++idx) m_io_arr[idx] = read_at(idx);
}

int Mux::read_at(uint8_t idx) const {
    if (idx >= CHANNEL_COUNT) return 0;
    select_channel(idx);
    return analogRead(sig);
}

const int* Mux::get_io_arr() {
    update_io_arr();
    return m_io_arr;
}
