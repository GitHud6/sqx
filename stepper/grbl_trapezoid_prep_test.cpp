#include "grbl_trapezoid_prep.hpp"

#include <algorithm>
#include <cmath>

namespace ray2::stepper {
namespace {

bool run_case(const MotionProfileInput& input, const std::uint32_t total_steps,
              GrblTrapezoidPrepTestCase& result) {
    result = {.expected_steps = total_steps};
    GrblTrapezoidPrep prep = {};
    if (!grbl_trapezoid_prep_init(prep, input, total_steps)) return false;

    GrblTrapezoidPrepSlice slice = {};
    while (grbl_trapezoid_prep_next(prep, slice)) {
        if (slice.timer_ticks == 0 || !std::isfinite(slice.duration_us) ||
            !std::isfinite(slice.dt_remainder_us)) {
            return false;
        }
        result.emitted_steps += slice.major_steps;
        ++result.slice_count;
        if (slice.major_steps == 0) {
            ++result.zero_step_slice_count;
        }
        result.total_duration_us += slice.duration_us;
        result.maximum_remainder_us = std::max(result.maximum_remainder_us,
                                                std::abs(slice.dt_remainder_us));
    }
    result.passed = prep.complete && result.emitted_steps == total_steps &&
                    prep.emitted_major_steps == total_steps &&
                    prep.dt_remainder_minutes == 0.0;
    return result.passed;
}

}  // namespace

bool grbl_trapezoid_prep_self_test(GrblTrapezoidPrepSelfTestReport& report) {
    constexpr double acceleration = 500.0 * 60.0 * 60.0;
    report = {};
    const bool first = run_case({.length_mm = 100.0,
                                 .entry_speed_mm_per_min = 0.0,
                                 .nominal_speed_mm_per_min = 3000.0,
                                 .exit_speed_mm_per_min = 0.0,
                                 .acceleration_mm_per_min2 = acceleration},
                                8000, report.cases[0]);
    const bool second = run_case({.length_mm = 1.0,
                                  .entry_speed_mm_per_min = 1500.0,
                                  .nominal_speed_mm_per_min = 3000.0,
                                  .exit_speed_mm_per_min = 1500.0,
                                  .acceleration_mm_per_min2 = acceleration},
                                 80, report.cases[1]);
    const bool third = run_case({.length_mm = 0.0125,
                                 .entry_speed_mm_per_min = 0.0,
                                 .nominal_speed_mm_per_min = 3000.0,
                                 .exit_speed_mm_per_min = 0.0,
                                 .acceleration_mm_per_min2 = acceleration},
                                1, report.cases[2]);
    report.passed = first && second && third;
    return report.passed;
}

}  // namespace ray2::stepper
