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

#include "canopen_402_driver/n5_homing_mode.hpp"
#include <iostream>

using namespace ros2_canopen;

bool N5HomingMode::executeHoming()
{
  // Step 0: Check if already homed by reading the statusword
  // (We're already in homing mode since switchMode was called before this)
  uint16_t statusword = 0;
  bool statusword_read_success = driver->sync_sdo_read_typed<uint16_t>(
    0x6041,  // Statusword
    0,
    statusword,
    std::chrono::milliseconds(100)
  );

  if (statusword_read_success)
  {
    // Check if homing attained (bit 12) and target reached (bit 10) are both set
    const uint16_t HOMING_ATTAINED = (1 << 12);
    const uint16_t TARGET_REACHED = (1 << 10);

    if ((statusword & HOMING_ATTAINED) && (statusword & TARGET_REACHED))
    {
      std::cout << "N5HomingMode: Motor is already homed (statusword: 0x" << std::hex
                << statusword << std::dec << "), skipping homing sequence" << std::endl;

      // Still need to ensure safe limit behavior is set
      bool restore_success = driver->sync_sdo_write_typed<int16_t>(
        limit_behavior_index_,
        0,
        operational_limit_value_,
        std::chrono::milliseconds(100)
      );

      if (!restore_success)
      {
        std::cout << "N5HomingMode: Warning - Failed to set safe limit behavior" << std::endl;
      }

      return true;
    }
  }

  std::cout << "N5HomingMode: Motor not homed, proceeding with homing sequence" << std::endl;

  // Step 1: Configure limit switch behavior for homing (ignore limits)
  std::cout << "N5HomingMode: Setting limit behavior to " << std::dec << homing_limit_value_
            << " for homing" << std::endl;

  bool write_success = driver->sync_sdo_write_typed<int16_t>(
    limit_behavior_index_,
    0,
    homing_limit_value_,
    std::chrono::milliseconds(100)
  );

  if (!write_success)
  {
    std::cout << "N5HomingMode: Failed to set limit behavior before homing" << std::endl;
    return false;
  }

  // Step 2: Execute the standard homing sequence
  std::cout << "N5HomingMode: Executing homing sequence" << std::endl;
  bool homing_success = DefaultHomingMode::executeHoming();

  // Step 3: ALWAYS restore safe limit switch behavior for normal operation (error on limit)
  // This must happen regardless of homing success for safety
  std::cout << "N5HomingMode: Setting limit behavior to " << std::dec << operational_limit_value_
            << " for normal operation" << std::endl;

  bool restore_success = driver->sync_sdo_write_typed<int16_t>(
    limit_behavior_index_,
    0,
    operational_limit_value_,
    std::chrono::milliseconds(100)
  );

  if (!restore_success)
  {
    std::cout << "N5HomingMode: ERROR - Failed to restore safe limit behavior after homing!"
              << std::endl;
    // This is critical - we cannot leave the motor ignoring limits
    return false;
  }

  if (!homing_success)
  {
    std::cout << "N5HomingMode: Homing sequence failed (but safe limits were restored)" << std::endl;
    return false;
  }

  std::cout << "N5HomingMode: Homing completed successfully" << std::endl;
  return true;
}
