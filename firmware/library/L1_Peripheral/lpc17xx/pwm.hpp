#pragma once

#include "L1_Peripheral/lpc40xx/pwm.hpp"

namespace sjsu
{
namespace lpc17xx
{
// The LPC40xx driver is compatible with the lpc17xx peripheral
using ::sjsu::lpc40xx::Pwm;
}  // namespace lpc17xx
}  // namespace sjsu
