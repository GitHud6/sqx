#include "machine_profile.hpp"

namespace laserctl::machine {
const AxisHardware& machine_axis(Axis axis) {
    return generated::kAxes[static_cast<std::size_t>(axis)];
}

const LaserHardware& machine_laser() {
    return generated::kLaser;
}

}  // namespace laserctl::machine
