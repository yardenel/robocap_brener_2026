#pragma once
#include <cstddef>
#include <cstdint>

class I2CMuxTCA {
   private:
    static constexpr const size_t CHANNEL_AMOUNT = 8;
    static constexpr const int NO_CHNL           = 0;

    uint8_t m_cur_active = NO_CHNL;  // initially no channel active :/
    const uint8_t m_addr;

    void mf_write_byte(uint8_t state) const;

   public:
    I2CMuxTCA(uint8_t addr);

    void select(uint8_t chnl);
    void disable();
    uint8_t active() const;

    I2CMuxTCA()  = delete;
    ~I2CMuxTCA() = default;
};