#include "trapezoid_execution_plan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ray2::stepper {
namespace {

bool run_case(const MotionProfileInput& input, const std::uint32_t total_steps,
              TrapezoidGeneratorTestCase& result) {
    result = {};
    result.expected_steps = total_steps;
    TrapezoidExecutionPlan plan = {};
    if (!trapezoid_execution_plan_build({
                                            .execution_id = 1,
                                            .total_major_steps = total_steps,
                                            .profile_input = input,
                                            .segment_duration_us = 1000,
                                            .timer_ticks_per_us = 20,
                                        }, plan)) {
        return false;
    }

    std::uint32_t emitted_steps = 0;
    double previous_time = 0.0;
    double previous_distance = 0.0;
    double maximum_remainder = 0.0;
    double emitted_pulse_time_us = 0.0;
    TrapezoidTimeSlice slice = {};
    while (trapezoid_execution_plan_next(plan, slice)) {
        if (slice.major_steps == 0 || slice.step_interval_ticks == 0 ||
            slice.end_time_us <= previous_time || slice.end_distance_mm < previous_distance ||
            !std::isfinite(plan.generator.dt_remainder_us)) {
            return false;
        }
        emitted_steps += slice.major_steps;
        ++result.slice_count;
        emitted_pulse_time_us += static_cast<double>(slice.step_interval_ticks) *
                                 slice.major_steps / 20.0;
        maximum_remainder = std::max(maximum_remainder, plan.generator.dt_remainder_us);
        previous_time = slice.end_time_us;
        previous_distance = slice.end_distance_mm;
    }
    result.emitted_steps = emitted_steps;
    result.final_time_us = previous_time;
    result.expected_time_us = plan.profile.total_time_us;
    result.emitted_pulse_time_us = emitted_pulse_time_us;
    result.pulse_time_error_us = emitted_pulse_time_us - plan.profile.total_time_us;
    result.maximum_remainder_us = maximum_remainder;
    const double tick_rounding_bound_us =
        static_cast<double>(result.slice_count) / 20.0 + 0.001;
    result.passed = trapezoid_execution_plan_complete(plan) && emitted_steps == total_steps &&
                    std::abs(previous_time - plan.profile.total_time_us) < 0.001 &&
                    std::abs(previous_distance - plan.profile.length_mm) < 0.000001 &&
                    plan.generator.dt_remainder_us == 0.0 &&
                    result.pulse_time_error_us >= 0.0 &&
                    result.pulse_time_error_us <= tick_rounding_bound_us;
    return result.passed;
}

}  // namespace

// This is intentionally independent of Unity/ESP-IDF test infrastructure so
// it can be called by a host harness or temporarily from an embedded test.
bool trapezoid_segment_generator_self_test_report(TrapezoidGeneratorSelfTestReport& report) {
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

bool trapezoid_segment_generator_self_test() {
    TrapezoidGeneratorSelfTestReport report = {};
    return trapezoid_segment_generator_self_test_report(report);
}

bool trapezoid_segment_generator_format_self_test(char* buffer, const std::size_t buffer_size) {
    if (buffer == nullptr || buffer_size == 0) return false;
    TrapezoidGeneratorSelfTestReport report = {};
    trapezoid_segment_generator_self_test_report(report);
    int used = std::snprintf(buffer, buffer_size, "[PROFILE-SELFTEST:result=%s",
                             report.passed ? "ok" : "fail");
    if (used <= 0 || static_cast<std::size_t>(used) >= buffer_size) return false;
    for (const TrapezoidGeneratorTestCase& test_case : report.cases) {
        const int written = std::snprintf(
            buffer + used, buffer_size - static_cast<std::size_t>(used),
            "|case=(expected=%u,emitted=%u,slices=%u,time=%.1f/%.1f,pulse=%.1f/%.1f,error=%.3f,remainder_max=%.3f,ok=%d)",
            static_cast<unsigned>(test_case.expected_steps),
            static_cast<unsigned>(test_case.emitted_steps),
            static_cast<unsigned>(test_case.slice_count), test_case.final_time_us,
            test_case.expected_time_us, test_case.emitted_pulse_time_us,
            test_case.expected_time_us, test_case.pulse_time_error_us,
            test_case.maximum_remainder_us,
            test_case.passed ? 1 : 0);
        if (written <= 0 || static_cast<std::size_t>(written) >=
                                buffer_size - static_cast<std::size_t>(used)) {
            return false;
        }
        used += written;
    }
    const int suffix = std::snprintf(buffer + used, buffer_size - static_cast<std::size_t>(used), "]\r\n");
    return suffix > 0 && static_cast<std::size_t>(suffix) < buffer_size - static_cast<std::size_t>(used);
}

}  // namespace ray2::stepper
