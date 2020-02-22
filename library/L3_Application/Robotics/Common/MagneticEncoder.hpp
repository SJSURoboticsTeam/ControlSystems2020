#pragma once
#include <cstdint>
#include "utility/log.hpp"
#include "L1_Peripheral/i2c.hpp"
#include "L1_Peripheral/lpc40xx/i2c.hpp"
#include "utility/map.hpp"

namespace sjsu
{
namespace robotics
{
class MagneticEncoder
{
 public:
  static constexpr uint8_t kMagneticEncoderAddress   = 0x36;
  static constexpr uint8_t kMagneticEncoderMagnitude = 0x1B;
  static constexpr uint8_t kEncoderAngle             = 0x0E;
  static constexpr units::angle::degree_t kZeroAngle = 0_deg;

  explicit constexpr MagneticEncoder(sjsu::I2c & i2c)
      : current_angle_(kZeroAngle), i2c_(i2c)
  {
  }

  void Initialize()
  {
    i2c_.Initialize();
  }

  units::angle::degree_t GetAngle() const
  {
    uint8_t register_vals[2];
    uint16_t angle;
    uint16_t mapedAngle;

    i2c_.WriteThenRead(kMagneticEncoderAddress,
                       { kEncoderAngle },
                       register_vals,
                       sizeof(register_vals));

    angle      = register_vals[0] << 8 | register_vals[1];
    mapedAngle = sjsu::Map(angle, 0, 4090, 0, 360);

    LOG_INFO("\nAngle: %d\nMaped: %d\n", angle, mapedAngle);
    // return (units::angle::deg)mapedAngle; //How to convert to degrees??
  }

  void SetZero()
  {
    current_angle_ = kZeroAngle;
  }

 private:
  units::angle::degree_t current_angle_;
  const sjsu::I2c & i2c_;
};
}  // namespace robotics
}  // namespace sjsu