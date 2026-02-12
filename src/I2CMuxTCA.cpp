#include <Wire.h>

#include <I2CMuxTCA.hpp>

I2CMuxTCA::I2CMuxTCA(uint8_t addr)
    : m_addr(addr) {}

void I2CMuxTCA::mf_write_byte(uint8_t dat) const {
    Wire.beginTransmission(m_addr);
    Wire.write(dat);
    Wire.endTransmission();
}

void I2CMuxTCA::select(uint8_t chnl) {
    if (chnl < CHANNEL_AMOUNT && (chnl = 1 << chnl) != m_cur_active)
        return mf_write_byte(m_cur_active = chnl);
}

uint8_t I2CMuxTCA::active() const { return m_cur_active; }

void I2CMuxTCA::disable() {
    if (m_cur_active != NO_CHNL) return mf_write_byte(m_cur_active = NO_CHNL);
}
