#pragma once

#include <cstdint>
class I2CMux {
   private:
    const uint8_t m_addr;
    uint8_t m_cur_chnl = UINT8_MAX;

   public:
    I2CMux(uint8_t addr);

    I2CMux()  = delete;
    ~I2CMux() = default;
};