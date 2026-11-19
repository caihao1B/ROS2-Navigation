#pragma once
#include <algorithm>
namespace nav_control {
// Confirmation controls only additional NAV_CONTROL long-term memory. Every raw
// lethal candidate still reaches the unchanged TERRAIN protective stop and live
// costmap.
inline int confirmLethal(int previous, bool lethal, double observation_gap) {
  if (!lethal)
    return 0;
  return std::min(3, (observation_gap > .3 ? 0 : previous) + 1);
}
} // namespace nav_control
