#pragma once
#include <Wire.h>

template <typename WireType>
class TCS34725_ {
    static constexpr uint8_t I2C_ADDR{0x29};
    // TCS34721/25 ID = 0x44, TCS34723/27 ID = 0x4d
    static constexpr uint8_t VALID_PART_IDS[]{0x44, 0x4d};
    static constexpr uint8_t COMMAND_BIT{0x80};
    static constexpr uint8_t POWER_DELAY{3};
    // 2.4 ms must pass after PON is asserted before an
    // RGBC can be initiated

    static constexpr float INTEGRATION_CYCLES_MIN{1.f};
    static constexpr float INTEGRATION_CYCLES_MAX{256.f};
    static constexpr float INTEGRATION_TIME_MS_MIN{2.4f};
    static constexpr float INTEGRATION_TIME_MS_MAX{
        INTEGRATION_TIME_MS_MIN * INTEGRATION_CYCLES_MAX
    };

   public:
    enum class Reg : uint8_t {
        ENABLE  = 0x00,
        ATIME   = 0x01,
        WTIME   = 0x03,
        AILTL   = 0x04,
        AILTH   = 0x05,
        AIHTL   = 0x06,
        AIHTH   = 0x07,
        PERS    = 0x0C,
        CONFIG  = 0x0D,
        CONTROL = 0x0F,
        ID      = 0x12,
        STATUS  = 0x13,
        CDATAL  = 0x14,
        CDATAH  = 0x15,
        RDATAL  = 0x16,
        RDATAH  = 0x17,
        GDATAL  = 0x18,
        GDATAH  = 0x19,
        BDATAL  = 0x1A,
        BDATAH  = 0x1B,
    };

    enum class Mask : uint8_t {
        ENABLE_AIEN   = 0x10,
        ENABLE_WEN    = 0x08,
        ENABLE_AEN    = 0x02,
        ENABLE_PON    = 0x01,
        STATUS_AINT   = 0x10,
        STATUS_AVALID = 0x01
    };

    enum class Gain : uint8_t { X01, X04, X16, X60 };

    struct Color {
        float r, g, b;
    };
    union RawData {
        struct {
            uint16_t c;
            uint16_t r;
            uint16_t g;
            uint16_t b;
        };
        uint8_t raw[sizeof(uint16_t) * 4];
    };

    bool attach(WireType& w = Wire) {
        wire      = &w;
        uint8_t x = read8(Reg::ID);
        if (x != VALID_PART_IDS[0] && x != VALID_PART_IDS[1]) return false;

        power(true);
        interrupt(true);  // use to detect availability (available())
        persistence(0);   // every RGBC cycle generates an interrupt
        return true;
    }

    void power(bool b) {
        if (b) {
            write8(Reg::ENABLE, static_cast<uint8_t>(Mask::ENABLE_PON));
            delay(POWER_DELAY);
            write8(
                Reg::ENABLE,
                static_cast<uint8_t>(Mask::ENABLE_PON)
                    | static_cast<uint8_t>(Mask::ENABLE_AEN)
            );
            return;
        }

        uint8_t val(read8(Reg::ENABLE));
        write8(
            Reg::ENABLE,
            val & ~((uint8_t)Mask::ENABLE_PON | (uint8_t)Mask::ENABLE_AEN)
        );
    }

    void enableColorTempAndLuxCalculation(bool b) { b_ct_lux_calc = b; }

    void integrationTime(float ms)  // 2.4 - 614.4 ms
    {
        ms = min(max(ms, INTEGRATION_TIME_MS_MIN), INTEGRATION_TIME_MS_MAX);
        uint8_t data(
            static_cast<int>(std::round(256.f - ms / INTEGRATION_TIME_MS_MIN))
        );

        write8(Reg::ATIME, data);
        atime            = data;
        integration_time = ms;
    }

    void gain(Gain g) {
        write8(Reg::CONTROL, static_cast<uint8_t>(g));
        switch (g) {
            case Gain::X01:
                gain_value = 1.f;
                break;
            case Gain::X04:
                gain_value = 4.f;
                break;
            case Gain::X16:
                gain_value = 16.f;
                break;
            case Gain::X60:
                gain_value = 60.f;
                break;
            default:
                gain_value = 1.f;
                break;
        }
    }

    void scale(float s) { scaling = s; }
    // The Glass Attenuation (FA) factor used to compensate for lower light
    // levels at the device due to the possible presence of glass. The GA is
    // the inverse of the glass transmissivity (T), so GA = 1/T. A transmissivity
    // of 50% gives GA = 1 / 0.50 = 2. If no glass is present, use GA = 1.
    // See Application Note: DN40-Rev 1.0 â€“ Lux and CCT Calculations using
    // ams Color Sensors for more details.
    void glassAttenuation(float v) { glass_attenuation = max(v, 1); }
    void persistence(uint16_t data) { write8(Reg::PERS, data); }
    bool available() {
        bool b = read8(Reg::STATUS) & (uint8_t)Mask::STATUS_AINT;
        if (b) {
            update();
            if (b_ct_lux_calc) calcTemperatureAndLuxDN40();
            clearInterrupt();
        }
        return b;
    }

    Color color() const {
        Color clr;
        if (!raw_data.c) {
            clr.r = clr.g = clr.b = 0;
            return clr;
        }

        auto norm_clr = [&](const uint16_t& clrval) -> float {
            return min(pow(static_cast<float>(clrval)
                               / static_cast<float>(raw_data.c),
                           scaling),
                       1)
                   * 255;
        };

        clr.r = norm_clr(raw_data.r);
        clr.g = norm_clr(raw_data.g);
        clr.b = norm_clr(raw_data.b);
        return clr;
    }

    const RawData& raw() const { return raw_data; }
    float lux() const { return lx; }
    float colorTemperature() const { return color_temp; }

    void interrupt(bool b) {
        uint8_t r = read8(Reg::ENABLE);
        write8(
            Reg::ENABLE,
            b ? r | static_cast<uint8_t>(Mask::ENABLE_AIEN)
              : r & ~static_cast<uint8_t>(Mask::ENABLE_AIEN)
        );
    }

    void clearInterrupt() {
        wire->beginTransmission(I2C_ADDR);
        wire->write(COMMAND_BIT | 0x66);
        wire->endTransmission();
    }

    void write8(Reg reg, uint8_t value) {
        wire->beginTransmission(I2C_ADDR);
        wire->write(COMMAND_BIT | static_cast<uint8_t>(reg));
        wire->write(value);
        wire->endTransmission();
    }

    uint8_t read8(Reg reg) {
        wire->beginTransmission(I2C_ADDR);
        wire->write(COMMAND_BIT | static_cast<uint8_t>(reg));
        wire->endTransmission();
        wire->requestFrom(I2C_ADDR, 1);
        return wire->read();
    }

    uint16_t read16(Reg reg) {
        uint16_t x;
        uint16_t t;

        wire->beginTransmission(I2C_ADDR);
        wire->write(COMMAND_BIT | (uint8_t)reg);
        wire->endTransmission();

        wire->requestFrom(I2C_ADDR, (uint8_t)2);
        t = wire->read();
        x = wire->read();
        x <<= 8;
        x |= t;
        return x;
    }

   private:
    void update() {
        wire->beginTransmission(I2C_ADDR);
        wire->write(COMMAND_BIT | (uint8_t)Reg::CDATAL);
        wire->endTransmission();
        wire->requestFrom(I2C_ADDR, sizeof(RawData));
        for (uint8_t i(0); i < sizeof(RawData); raw_data.raw[i++] = wire->read());
    }

    // https://github.com/adafruit/Adafruit_CircuitPython_TCS34725/blob/master/adafruit_tcs34725.py
    void calcTemperatureAndLuxDN40() {
        // Device specific values (DN40 Table 1 in Appendix I)
        const float GA             = glass_attenuation;  // Glass Attenuation Factor
        static const float DF      = 310.f;              // Device Factor
        static const float R_Coef  = 0.136f;             //
        static const float G_Coef  = 1.f;                // used in lux computation
        static const float B_Coef  = -0.444f;            //
        static const float CT_Coef = 3810.f;    // Color Temperature Coefficient
        static const float CT_Offset = 1391.f;  // Color Temperatuer Offset

        // Analog/Digital saturation (DN40 3.5)
        float saturation = (256 - atime > 63) ? 65535 : 1024 * (256 - atime);

        // Ripple saturation (DN40 3.7)
        if (integration_time < 150) saturation -= saturation / 4;

        // Check for saturation and mark the sample as invalid if true
        if (raw_data.c >= saturation) return;

        // IR Rejection (DN40 3.1)
        float sum = raw_data.r + raw_data.g + raw_data.b;
        float c   = raw_data.c;
        float ir  = (sum > c) ? ((sum - c) / 2.f) : 0.f;
        float r2  = raw_data.r - ir;
        float g2  = raw_data.g - ir;
        float b2  = raw_data.b - ir;

        // Lux Calculation (DN40 3.2)
        float g1  = R_Coef * r2 + G_Coef * g2 + B_Coef * b2;
        float cpl = (integration_time * gain_value) / (GA * DF);
        lx        = g1 / cpl;

        // CT Calculations (DN40 3.4)
        color_temp = CT_Coef * b2 / r2 + CT_Offset;
    }

    WireType* wire;
    float scaling{2.5f};

    // for lux & temperature
    bool b_ct_lux_calc{true};
    float lx;
    float color_temp;
    RawData raw_data;
    float gain_value{1.f};
    uint8_t atime{0xFF};
    float integration_time{2.4f};  // [ms]
    float glass_attenuation{1.f};
};

using TCS34725 = TCS34725_<TwoWire>;
