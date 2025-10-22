//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2014-2022 Authors of ros_canopen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#ifndef N5_HOMING_MODE_HPP
#define N5_HOMING_MODE_HPP

#include "canopen_402_driver/default_homing_mode.hpp"
#include <chrono>

namespace ros2_canopen
{

/**
 * @brief Custom homing mode for N5-2-2 motor controller
 *
 * This homing mode extends DefaultHomingMode to handle the N5-2-2 controller's
 * requirement for different limit switch behaviors during homing vs. normal operation:
 * - Before homing: Sets "behavior upon reaching limit switch" (0x3701) to -1 (ignore limits)
 * - After homing: Sets it to 6 (error on limit for safety during normal operation)
 */
class N5HomingMode : public DefaultHomingMode
{
private:
  const uint16_t limit_behavior_index_ = 0x3701;  // "Behavior upon reaching limit switch"
  const int16_t homing_limit_value_ = -1;         // Ignore limits during homing
  const int16_t operational_limit_value_ = 6;     // Error on limit during operation

public:
  N5HomingMode(
    std::shared_ptr<LelyDriverBridge> driver,
    int homing_timeout_seconds)
  : DefaultHomingMode(driver, homing_timeout_seconds)
  {
  }

  /**
   * @brief Execute homing with N5-2-2 specific limit switch configuration
   *
   * This method:
   * 1. Writes SDO 0x3701 = -1 (ignore limits during homing)
   * 2. Executes the standard homing sequence
   * 3. Writes SDO 0x3701 = 6 (error on limit for normal operation)
   *
   * @return true if homing succeeded, false otherwise
   */
  virtual bool executeHoming() override;
};

}  // namespace ros2_canopen

#endif  // N5_HOMING_MODE_HPP
