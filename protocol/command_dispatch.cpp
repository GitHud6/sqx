#include "command_dispatch.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "command_executor.hpp"
#include "homing_service.hpp"
#include "limit_switch_driver.hpp"
#include "sd_command_handler.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
#include "rmt_step_pulse_driver.hpp"
#include "planner.hpp"
#include "protocol_tx.hpp"
#include "realtime_command.hpp"
#include "stepper_driver.hpp"
#include "segment_ring.hpp"
#include "stepper_prep.hpp"
#include "timer_stepper_executor.hpp"
#include "trapezoid_segment_generator.hpp"
#include "uart0_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace ray2::protocol {

namespace {

bool write_all(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) return false;
    // The TX pool deliberately uses small fixed packets. Split verbose
    // engineering responses here instead of growing every queued packet.
    while (length != 0) {
        const std::size_t chunk_length = std::min(length, kProtocolTxMessageCapacity);
        if (!protocol_tx_enqueue_normal(data, chunk_length)) return false;
        data += chunk_length;
        length -= chunk_length;
    }
    return true;
}

bool write_text(const char* text) {
    return write_all(reinterpret_cast<const std::uint8_t*>(text), std::strlen(text));
}

bool report_status(command::Error error) {
    if (error == command::Error::Ok || error == command::Error::ProgramEnd) {
        return write_text("ok\r\n");
    }

    char response[80] = {};
    const int length = std::snprintf(response, sizeof(response), "error:%s\r\n", command::error_text(error));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

command::ParserState s_parser_state = {};
SemaphoreHandle_t s_parser_mutex = nullptr;

enum class PositionCommandType {
    Status,
    Set,
};

struct PositionCommand {
    PositionCommandType type = PositionCommandType::Status;
    double machine_position_mm[machine::kAxisCount] = {};
};

bool equals_ignore_case(const char* text, std::size_t length, const char* expected) {
    const std::size_t expected_length = std::strlen(expected);
    if (length != expected_length) {
        return false;
    }
    for (std::size_t index = 0; index < length; ++index) {
        if (std::toupper(static_cast<unsigned char>(text[index])) !=
            std::toupper(static_cast<unsigned char>(expected[index]))) {
            return false;
        }
    }
    return true;
}

bool snapshot_parser_state(command::ParserState& state) {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) return false;
    }
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    state = s_parser_state;
    xSemaphoreGive(s_parser_mutex);
    return true;
}

const char* realtime_state_text(const stepper::MachinePosition& position) {
    if (homing::homing_is_active()) return "Home";
    if (position.state != stepper::PositionState::Known) return "Alarm";
    return planner::planner_is_idle() ? "Idle" : "Run";
}

const char* motion_mode_text(const command::MotionMode mode) {
    switch (mode) {
        case command::MotionMode::Rapid: return "G0";
        case command::MotionMode::Linear: return "G1";
        case command::MotionMode::ArcClockwise: return "G2";
        case command::MotionMode::ArcCounterClockwise: return "G3";
    }
    return "G1";
}

const char* distance_mode_text(const command::DistanceMode mode) {
    return mode == command::DistanceMode::Absolute ? "G90" : "G91";
}

const char* units_text(const command::Units units) {
    return units == command::Units::Millimeters ? "G21" : "G20";
}

const char* spindle_mode_text(const command::SpindleMode mode) {
    switch (mode) {
        case command::SpindleMode::Clockwise: return "M3";
        case command::SpindleMode::CounterClockwise: return "M4";
        case command::SpindleMode::Off: return "M5";
    }
    return "M5";
}

const char* motion_profile_mode_text(const stepper::MotionProfileMode mode) {
    switch (mode) {
        case stepper::MotionProfileMode::Trapezoid: return "trapezoid";
        case stepper::MotionProfileMode::SCurve: return "s-curve";
    }
    return "unknown";
}

bool report_build_info() {
    static constexpr char kBuildInfo[] =
        "[VER:1.1f.20260826:RayCube-IDF]\r\n"
        "[OPT:V,32,256]\r\n"
        "ok\r\n";
    return write_text(kBuildInfo);
}

bool report_gcode_modes() {
    command::ParserState state = {};
    if (!snapshot_parser_state(state)) return write_text("error:protocol-no-mem\r\n");

    const double feed_rate = state.units == command::Units::Inches
                                 ? state.feed_rate_mm_per_min / 25.4
                                 : state.feed_rate_mm_per_min;
    static char response[192] = {};
    const int length = state.units == command::Units::Inches
                           ? std::snprintf(response, sizeof(response),
                                           "[GC:%s G54 G17 %s %s G94 %s M9 T0 F%.1f S%.0f]\r\nok\r\n",
                                           motion_mode_text(state.motion_mode), units_text(state.units),
                                           distance_mode_text(state.distance_mode),
                                           spindle_mode_text(state.spindle_mode), feed_rate,
                                           state.spindle_speed)
                           : std::snprintf(response, sizeof(response),
                                           "[GC:%s G54 G17 %s %s G94 %s M9 T0 F%.0f S%.0f]\r\nok\r\n",
                                           motion_mode_text(state.motion_mode), units_text(state.units),
                                           distance_mode_text(state.distance_mode),
                                           spindle_mode_text(state.spindle_mode), feed_rate,
                                           state.spindle_speed);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool report_coordinate_parameters() {
    command::ParserState state = {};
    if (!snapshot_parser_state(state)) return write_text("error:protocol-no-mem\r\n");

    static char response[192] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[G54:%.3f,%.3f,%.3f]\r\n[G92:0.000,0.000,0.000]\r\n[TLO:0.000]\r\nok\r\n",
        state.work_offset_mm[0], state.work_offset_mm[1], state.work_offset_mm[2]);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool is_profile_self_test_command(const char* line, const std::size_t length) {
    return equals_ignore_case(line, length, "$PROFILE/SELFTEST");
}

bool report_profile_self_test() {
    if (homing::homing_is_active() || sd::sd_job_is_active() || !planner::planner_is_idle()) {
        return write_text("error:profile-selftest-busy\r\n");
    }
    char response[768] = {};
    if (!stepper::trapezoid_segment_generator_format_self_test(response, sizeof(response))) {
        return write_text("error:profile-selftest-format\r\n");
    }
    const std::size_t length = std::strlen(response);
    if (!write_all(reinterpret_cast<const std::uint8_t*>(response), length)) return false;
    return write_text(std::strstr(response, "result=ok") != nullptr &&
                          std::strstr(response, "grbl_ref=(result=ok") != nullptr
                          ? "ok\r\n"
                          : "error:profile-selftest-failed\r\n");
}

bool report_grbl_settings() {
    const auto& x_axis = machine::raycube_axis(machine::Axis::X);
    const auto& y_axis = machine::raycube_axis(machine::Axis::Y);
    const auto& laser = machine::raycube_laser();
    static char response[512] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "$0=%u\r\n$1=250\r\n$10=1\r\n$11=%.3f\r\n$20=1\r\n$22=1\r\n$23=0\r\n"
        "$30=%lu\r\n$31=0\r\n$32=1\r\n$100=%.3f\r\n$101=%.3f\r\n"
        "$110=%.3f\r\n$111=%.3f\r\n$120=%.3f\r\n$121=%.3f\r\n"
        "$130=%.3f\r\n$131=%.3f\r\nok\r\n",
        machine::kStepPulseMicroseconds, planner::kJunctionDeviationMm,
        static_cast<unsigned long>(laser.max_power), static_cast<double>(x_axis.steps_per_mm),
        static_cast<double>(y_axis.steps_per_mm), static_cast<double>(x_axis.max_rate_mm_per_min),
        static_cast<double>(y_axis.max_rate_mm_per_min),
        static_cast<double>(x_axis.acceleration_mm_per_sec2),
        static_cast<double>(y_axis.acceleration_mm_per_sec2), static_cast<double>(x_axis.max_travel_mm),
        static_cast<double>(y_axis.max_travel_mm));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool parse_jog_command(const char* line,
                       const std::size_t length,
                       const char*& payload,
                       std::size_t& payload_length) {
    if (line == nullptr || length <= 3 || line[0] != '$' ||
        std::toupper(static_cast<unsigned char>(line[1])) != 'J' || line[2] != '=') {
        return false;
    }
    payload = line + 3;
    payload_length = length - 3;
    return true;
}

bool is_jog_command(const char* line, const std::size_t length) {
    return line != nullptr && length >= 2 && line[0] == '$' &&
           std::toupper(static_cast<unsigned char>(line[1])) == 'J';
}

bool execute_jog_command(const char* payload, const std::size_t payload_length) {
    if (homing::homing_is_active() || sd::sd_job_is_active() || !planner::planner_is_idle()) {
        return report_status(command::Error::MotionBusy);
    }
    if (stepper::machine_position_state() != stepper::PositionState::Known) {
        return report_status(command::Error::PositionUnknown);
    }

    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::execute_jog_line(payload, payload_length, s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    return report_status(result);
}

bool is_position_command(const char* line, std::size_t length) {
    return line != nullptr && length >= 5 && line[0] == '$' &&
           std::toupper(static_cast<unsigned char>(line[1])) == 'P' &&
           std::toupper(static_cast<unsigned char>(line[2])) == 'O' &&
           std::toupper(static_cast<unsigned char>(line[3])) == 'S' && line[4] == '/';
}

bool parse_position_command(const char* line, std::size_t length, PositionCommand& command) {
    if (!is_position_command(line, length)) {
        return false;
    }

    const char* const payload = line + 5;
    const std::size_t payload_length = length - 5;
    if (equals_ignore_case(payload, payload_length, "STATUS")) {
        command = {};
        command.type = PositionCommandType::Status;
        return true;
    }

    constexpr char kSetPrefix[] = "SET=";
    if (payload_length <= sizeof(kSetPrefix) - 1 ||
        !equals_ignore_case(payload, sizeof(kSetPrefix) - 1, kSetPrefix)) {
        return false;
    }

    const char* const coordinates = payload + sizeof(kSetPrefix) - 1;
    const std::size_t coordinates_length = payload_length - (sizeof(kSetPrefix) - 1);
    if (coordinates_length == 0 || coordinates_length >= 64) {
        return false;
    }

    char buffer[64] = {};
    std::memcpy(buffer, coordinates, coordinates_length);
    char* value_end = nullptr;
    const double x = std::strtod(buffer, &value_end);
    if (value_end == buffer || *value_end != ',') {
        return false;
    }
    const double y = std::strtod(value_end + 1, &value_end);
    if (*value_end != '\0' || !std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }

    command = {};
    command.type = PositionCommandType::Set;
    command.machine_position_mm[0] = x;
    command.machine_position_mm[1] = y;
    return true;
}

const char* position_state_text(stepper::PositionState state) {
    switch (state) {
        case stepper::PositionState::Unknown: return "unknown";
        case stepper::PositionState::Known: return "known";
        case stepper::PositionState::Invalidated: return "invalidated";
    }
    return "invalid";
}

const char* position_invalidation_reason_text(stepper::PositionInvalidationReason reason) {
    switch (reason) {
        case stepper::PositionInvalidationReason::None: return "none";
        case stepper::PositionInvalidationReason::Startup: return "startup";
        case stepper::PositionInvalidationReason::Reset: return "reset";
        case stepper::PositionInvalidationReason::RmtFailure: return "rmt-failure";
        case stepper::PositionInvalidationReason::ExecutionFailure: return "execution-failure";
    }
    return "unknown";
}

bool is_homing_command(const char* line, const std::size_t length) {
    return equals_ignore_case(line, length, "$H");
}

bool is_homing_status_command(const char* line, const std::size_t length) {
    return equals_ignore_case(line, length, "$H/STATUS");
}

bool parse_homing_seek_test_command(const char* line, const std::size_t length,
                                    homing::TestAxis& axis) {
    if (equals_ignore_case(line, length, "$H/TEST=X")) {
        axis = homing::TestAxis::X;
        return true;
    }
    if (equals_ignore_case(line, length, "$H/TEST=Y")) {
        axis = homing::TestAxis::Y;
        return true;
    }
    return false;
}

bool report_homing_status() {
    const homing::HomingState state = homing::homing_state();
    const driver::LimitSwitchState limits = driver::limit_switch_driver_state();
    char response[128] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[HOMING:state=%s|limit=(x=%d,y=%d)|debounce=%lu]\r\nok\r\n",
        homing::homing_state_text(state), limits.triggered[0] ? 1 : 0,
        limits.triggered[1] ? 1 : 0, static_cast<unsigned long>(limits.debounce_events));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool start_homing() {
    switch (homing::homing_start()) {
        case homing::StartResult::Accepted:
            return write_text("ok\r\n");
        case homing::StartResult::Busy:
            return write_text("error:homing-busy\r\n");
        case homing::StartResult::Unavailable:
            return write_text("error:homing-unavailable\r\n");
    }
    return write_text("error:homing-unavailable\r\n");
}

bool start_homing_seek_test(const homing::TestAxis axis) {
    switch (homing::homing_start_seek_test(axis)) {
        case homing::StartResult::Accepted:
            return write_text("ok\r\n");
        case homing::StartResult::Busy:
            return write_text("error:homing-busy\r\n");
        case homing::StartResult::Unavailable:
            return write_text("error:homing-unavailable\r\n");
    }
    return write_text("error:homing-unavailable\r\n");
}

const char* planner_block_state_text(planner::PlannerBlockState state) {
    switch (state) {
        case planner::PlannerBlockState::Executing: return "executing";
        case planner::PlannerBlockState::Frozen: return "frozen";
        case planner::PlannerBlockState::Lookahead: return "lookahead";
    }
    return "unknown";
}

bool report_position_status() {
    const stepper::MachinePosition position = stepper::machine_position();
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const double planned_x = s_parser_state.planned_machine_position_mm[0];
    const double planned_y = s_parser_state.planned_machine_position_mm[1];
    xSemaphoreGive(s_parser_mutex);
    const planner::PlannerDiagnostics planner_diagnostics = planner::planner_diagnostics();
    const stepper::SegmentExecutionDiagnostics segment = stepper::stepper_segment_diagnostics();
    const stepper::TimerStepperDiagnostics timer = stepper::timer_stepper_executor_diagnostics();
    const stepper::StepperPrepDiagnostics prep = stepper::stepper_prep_diagnostics();
    const command::ArcDiagnostics arc = command::arc_diagnostics();
    const driver::RmtStepPulseDiagnostics rmt = driver::rmt_step_pulse_driver_diagnostics();
    const RealtimeCommandDiagnostics realtime = realtime_command_diagnostics();
    const driver::Uart0RxDiagnostics uart = driver::uart0_rx_diagnostics();
    const ProtocolTxDiagnostics tx = protocol_tx_diagnostics();

    // Static storage keeps verbose diagnostics out of the protocol task stack.
    // write_all() fragments this response into the fixed-size TX pool packets.
    static char response[8192] = {};
    int length = std::snprintf(
        response, sizeof(response),
        "[POS:state=%s|reason=%s|steps=(%lld,%lld)|machine=(%.4f,%.4f)|planned=(%.4f,%.4f)|ahead=(%.4f,%.4f)|planner=(core=%d,recalc=(calls=%lu,max_cycles=%lu,lock=(snapshot=%lu/%lu,commit=%lu/%lu),stale=%lu),queued=%zu,frozen=%zu,completed=%llu,coalesced=%llu,collapse=(samples=%lu,severe=%lu,bins=%lu,%lu,%lu,%lu,%lu,id=%llu->%llu,len=%.4f/%.4f,speed=%.1f->%.1f,next=(entry=%.1f,nominal=%.1f,max_entry=%.1f,junction=%.1f),turn=%.2f))|prep=(active=%d,generated=%d,profile=%s,id=%llu,major=%ld/%ld,axis=(%ld,%ld),speed=%.1f,ctx=(used=%zu/%zu,g=%llu,e=%llu,c=%llu))|arc=(accepted=%llu,error=%.3f,max_chord=%.3f,last=(radius=%.3f,sweep=%.1f,chord=%.3f,segments=%zu))|lookahead=(b0=%s#%llu:e=%.1f,x=%.1f,j=%.1f,n=%.1f;b1=%s#%llu:e=%.1f,x=%.1f,j=%.1f,n=%.1f;b2=%s#%llu:e=%.1f,x=%.1f,j=%.1f,n=%.1f)|segment=(queued=%zu,capacity=%zu,high_water=%zu,underflows=%llu,active=%d,id=%llu,interval=%lu->%lu,amass=%u,speed=(%.1f,%.1f),completed_pending=%zu)|timer=(core=(init=%d,isr=%d),interval=%lu->%lu,delta=%lu,boundary_delta=%lu,boundaries=%lu,events=%lu,jitter=(samples=%lu,max=%.2fus,boundary=%lu/%.2fus,over2/5/10=%lu/%lu/%lu,mask=(%lu,%lu,%lu,%lu)/(%lu,%lu,%lu,%lu),body=(samples=%lu,max=%.2fus,boundary=%lu/%.2fus,over10=%lu,wrap=%lu),hot=(mhz=%lu,samples=%lu,front=%.2fus,normal=%lu/%.2fus/%lu,boundary=%lu/%.2fus/%lu,peak=(interval=%lu->%lu,amass=%u->%u,complete=%d))),amass=(changes=%lu,matrix=%lu,%lu,%lu,%lu/%lu,%lu,%lu,%lu/%lu,%lu,%lu,%lu/%lu,%lu,%lu,%lu),boundary=(id=%llu->%llu,base=%lu->%lu,effective=%lu->%lu,amass=%u->%u,major=%ld->%ld,slice=(duration=%lu->%lu,speed=%.1f->%.1f/%.1f->%.1f,phase=%u->%u)))|rmt=(trigger=(%llu,%llu),reset=(%llu,%llu),start=(%llu,%llu),timer=(%llu,%llu),invalid=%llu)|realtime=(status=%llu)|uart=(rx=%llu,rt=%llu,normal=%llu,high_water=%zu)|tx=(pending=%zu,normal=(queued=%llu,sent=%llu,failed=%llu),status=(published=%llu,sent=%llu,failed=%llu,coalesced=%llu))]\r\nok\r\n",
        position_state_text(position.state),
        position_invalidation_reason_text(position.invalidation_reason),
        static_cast<long long>(position.steps[0]),
        static_cast<long long>(position.steps[1]),
        position.millimeters[0], position.millimeters[1], planned_x, planned_y,
        planned_x - position.millimeters[0], planned_y - position.millimeters[1],
        planner_diagnostics.planner_task_core,
        static_cast<unsigned long>(planner_diagnostics.recalculate_calls),
        static_cast<unsigned long>(planner_diagnostics.recalculate_max_cycles),
        static_cast<unsigned long>(planner_diagnostics.snapshot_lock_max_cycles),
        static_cast<unsigned long>(planner_diagnostics.snapshot_lock_samples == 0 ? 0 :
            planner_diagnostics.snapshot_lock_total_cycles / planner_diagnostics.snapshot_lock_samples),
        static_cast<unsigned long>(planner_diagnostics.commit_lock_max_cycles),
        static_cast<unsigned long>(planner_diagnostics.commit_lock_samples == 0 ? 0 :
            planner_diagnostics.commit_lock_total_cycles / planner_diagnostics.commit_lock_samples),
        static_cast<unsigned long>(planner_diagnostics.stale_snapshot_count),
        planner_diagnostics.queued_blocks, planner_diagnostics.frozen_blocks,
        static_cast<unsigned long long>(planner::planner_completed_block_count()),
        static_cast<unsigned long long>(planner_diagnostics.coalesced_short_blocks),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.samples),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.severe_samples),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.severe_turn_bins[0]),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.severe_turn_bins[1]),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.severe_turn_bins[2]),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.severe_turn_bins[3]),
        static_cast<unsigned long>(planner_diagnostics.velocity_collapse.severe_turn_bins[4]),
        static_cast<unsigned long long>(planner_diagnostics.velocity_collapse.execution_id),
        static_cast<unsigned long long>(planner_diagnostics.velocity_collapse.next_execution_id),
        planner_diagnostics.velocity_collapse.length_mm,
        planner_diagnostics.velocity_collapse.next_length_mm,
        planner_diagnostics.velocity_collapse.entry_speed_mm_per_min,
        planner_diagnostics.velocity_collapse.exit_speed_mm_per_min,
        planner_diagnostics.velocity_collapse.next_entry_speed_mm_per_min,
        planner_diagnostics.velocity_collapse.next_nominal_speed_mm_per_min,
        planner_diagnostics.velocity_collapse.next_max_entry_speed_mm_per_min,
        planner_diagnostics.velocity_collapse.next_max_junction_speed_mm_per_min,
        planner_diagnostics.velocity_collapse.turn_degrees,
        prep.active ? 1 : 0, prep.generation_complete ? 1 : 0,
        motion_profile_mode_text(prep.profile_mode),
        static_cast<unsigned long long>(prep.execution_id),
        static_cast<long>(prep.executed_major_steps), static_cast<long>(prep.total_major_steps),
        static_cast<long>(prep.executed_axis_steps[0]), static_cast<long>(prep.executed_axis_steps[1]),
        static_cast<double>(prep.current_speed_tenths_mm_per_min) / 10.0,
        prep.context_used, prep.context_capacity,
        static_cast<unsigned long long>(prep.generation_context_execution_id),
        static_cast<unsigned long long>(prep.execution_context_execution_id),
        static_cast<unsigned long long>(prep.completion_context_execution_id),
        static_cast<unsigned long long>(arc.accepted_count), arc.chord_error_mm,
        arc.maximum_chord_length_mm, arc.last_radius_mm, arc.last_sweep_degrees,
        arc.last_chord_length_mm, arc.last_segment_count,
        planner_block_state_text(planner_diagnostics.blocks[0].state),
        static_cast<unsigned long long>(planner_diagnostics.blocks[0].execution_id),
        planner_diagnostics.blocks[0].entry_speed_mm_per_min,
        planner_diagnostics.blocks[0].exit_speed_mm_per_min,
        planner_diagnostics.blocks[0].max_junction_speed_mm_per_min,
        planner_diagnostics.blocks[0].programmed_rate_mm_per_min,
        planner_block_state_text(planner_diagnostics.blocks[1].state),
        static_cast<unsigned long long>(planner_diagnostics.blocks[1].execution_id),
        planner_diagnostics.blocks[1].entry_speed_mm_per_min,
        planner_diagnostics.blocks[1].exit_speed_mm_per_min,
        planner_diagnostics.blocks[1].max_junction_speed_mm_per_min,
        planner_diagnostics.blocks[1].programmed_rate_mm_per_min,
        planner_block_state_text(planner_diagnostics.blocks[2].state),
        static_cast<unsigned long long>(planner_diagnostics.blocks[2].execution_id),
        planner_diagnostics.blocks[2].entry_speed_mm_per_min,
        planner_diagnostics.blocks[2].exit_speed_mm_per_min,
        planner_diagnostics.blocks[2].max_junction_speed_mm_per_min,
        planner_diagnostics.blocks[2].programmed_rate_mm_per_min,
        segment.queued_segments, segment.segment_capacity, segment.high_water_segments,
        static_cast<unsigned long long>(segment.underflow_count), segment.active ? 1 : 0,
        static_cast<unsigned long long>(segment.active_execution_id),
        static_cast<unsigned long>(segment.active_step_interval_us),
        static_cast<unsigned long>(segment.active_amass_timer_interval_us),
        static_cast<unsigned>(segment.active_amass_level),
        static_cast<double>(segment.active_start_speed_tenths_mm_per_min) / 10.0,
        static_cast<double>(segment.active_end_speed_tenths_mm_per_min) / 10.0,
        segment.pending_completions,
        timer.timer_init_core, timer.timer_isr_core,
        static_cast<unsigned long>(timer.interval_min_ticks == UINT32_MAX ? 0 : timer.interval_min_ticks),
        static_cast<unsigned long>(timer.interval_max_ticks),
        static_cast<unsigned long>(timer.interval_max_delta_ticks),
        static_cast<unsigned long>(timer.boundary_max_delta_ticks),
        static_cast<unsigned long>(timer.boundary_samples),
        static_cast<unsigned long>(timer.scheduled_timer_events),
        static_cast<unsigned long>(timer.timer_lateness_samples),
        static_cast<double>(timer.timer_lateness_max_ticks) /
            stepper::kStepTimerTicksPerMicrosecond,
        static_cast<unsigned long>(timer.timer_lateness_boundary_samples),
        static_cast<double>(timer.timer_lateness_boundary_max_ticks) /
            stepper::kStepTimerTicksPerMicrosecond,
        static_cast<unsigned long>(timer.timer_lateness_over_2us),
        static_cast<unsigned long>(timer.timer_lateness_over_5us),
        static_cast<unsigned long>(timer.timer_lateness_over_10us),
        static_cast<unsigned long>(timer.timer_lateness_mask_samples[0]),
        static_cast<unsigned long>(timer.timer_lateness_mask_samples[1]),
        static_cast<unsigned long>(timer.timer_lateness_mask_samples[2]),
        static_cast<unsigned long>(timer.timer_lateness_mask_samples[3]),
        static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[0]),
        static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[1]),
        static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[2]),
        static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[3]),
        static_cast<unsigned long>(timer.timer_isr_body_samples),
        static_cast<double>(timer.timer_isr_body_max_ticks) /
            stepper::kStepTimerTicksPerMicrosecond,
        static_cast<unsigned long>(timer.timer_isr_body_boundary_samples),
        static_cast<double>(timer.timer_isr_body_boundary_max_ticks) /
            stepper::kStepTimerTicksPerMicrosecond,
        static_cast<unsigned long>(timer.timer_isr_body_over_10us),
        static_cast<unsigned long>(timer.timer_isr_body_counter_wrapped),
        static_cast<unsigned long>(timer.timer_isr_cycles_per_microsecond),
        static_cast<unsigned long>(timer.timer_isr_hot_path_samples),
        static_cast<double>(timer.timer_isr_pulse_front_max_cycles) /
            timer.timer_isr_cycles_per_microsecond,
        static_cast<unsigned long>(timer.timer_isr_normal_samples),
        static_cast<double>(timer.timer_isr_normal_max_cycles) /
            timer.timer_isr_cycles_per_microsecond,
        static_cast<unsigned long>(timer.timer_isr_normal_over_10us),
        static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_samples),
        static_cast<double>(timer.timer_isr_boundary_hot_path_max_cycles) /
            timer.timer_isr_cycles_per_microsecond,
        static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_over_10us),
        static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_peak_previous_interval_ticks),
        static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_peak_next_interval_ticks),
        static_cast<unsigned>(timer.timer_isr_boundary_hot_path_peak_previous_amass_level),
        static_cast<unsigned>(timer.timer_isr_boundary_hot_path_peak_next_amass_level),
        timer.timer_isr_boundary_hot_path_peak_completes_block ? 1 : 0,
        static_cast<unsigned long>(timer.amass_level_changes),
        static_cast<unsigned long>(timer.amass_transitions[0][0]),
        static_cast<unsigned long>(timer.amass_transitions[0][1]),
        static_cast<unsigned long>(timer.amass_transitions[0][2]),
        static_cast<unsigned long>(timer.amass_transitions[0][3]),
        static_cast<unsigned long>(timer.amass_transitions[1][0]),
        static_cast<unsigned long>(timer.amass_transitions[1][1]),
        static_cast<unsigned long>(timer.amass_transitions[1][2]),
        static_cast<unsigned long>(timer.amass_transitions[1][3]),
        static_cast<unsigned long>(timer.amass_transitions[2][0]),
        static_cast<unsigned long>(timer.amass_transitions[2][1]),
        static_cast<unsigned long>(timer.amass_transitions[2][2]),
        static_cast<unsigned long>(timer.amass_transitions[2][3]),
        static_cast<unsigned long>(timer.amass_transitions[3][0]),
        static_cast<unsigned long>(timer.amass_transitions[3][1]),
        static_cast<unsigned long>(timer.amass_transitions[3][2]),
        static_cast<unsigned long>(timer.amass_transitions[3][3]),
        static_cast<unsigned long long>(timer.boundary_prev_execution_id),
        static_cast<unsigned long long>(timer.boundary_next_execution_id),
        static_cast<unsigned long>(timer.boundary_prev_base_interval_ticks),
        static_cast<unsigned long>(timer.boundary_next_base_interval_ticks),
        static_cast<unsigned long>(timer.boundary_prev_interval_ticks),
        static_cast<unsigned long>(timer.boundary_next_interval_ticks),
        static_cast<unsigned>(timer.boundary_prev_amass_level),
        static_cast<unsigned>(timer.boundary_next_amass_level),
        static_cast<long>(timer.boundary_prev_major_steps),
        static_cast<long>(timer.boundary_next_major_steps),
        static_cast<unsigned long>(timer.boundary_prev_duration_us),
        static_cast<unsigned long>(timer.boundary_next_duration_us),
        static_cast<double>(timer.boundary_prev_start_speed_tenths_mm_per_min) / 10.0,
        static_cast<double>(timer.boundary_prev_end_speed_tenths_mm_per_min) / 10.0,
        static_cast<double>(timer.boundary_next_start_speed_tenths_mm_per_min) / 10.0,
        static_cast<double>(timer.boundary_next_end_speed_tenths_mm_per_min) / 10.0,
        static_cast<unsigned>(timer.boundary_prev_phase),
        static_cast<unsigned>(timer.boundary_next_phase),
        static_cast<unsigned long long>(rmt.trigger_count[0]),
        static_cast<unsigned long long>(rmt.trigger_count[1]),
        static_cast<unsigned long long>(rmt.reset_count[0]),
        static_cast<unsigned long long>(rmt.reset_count[1]),
        static_cast<unsigned long long>(rmt.start_count[0]),
        static_cast<unsigned long long>(rmt.start_count[1]),
        static_cast<unsigned long long>(timer.rmt_triggers[0]),
        static_cast<unsigned long long>(timer.rmt_triggers[1]),
        static_cast<unsigned long long>(rmt.invalid_trigger_count),
        static_cast<unsigned long long>(realtime.status_report_count),
        static_cast<unsigned long long>(uart.received_bytes),
        static_cast<unsigned long long>(uart.realtime_bytes),
        static_cast<unsigned long long>(uart.normal_bytes), uart.queue_high_water_chunks,
        tx.normal_pending, static_cast<unsigned long long>(tx.normal_enqueued),
        static_cast<unsigned long long>(tx.normal_sent),
        static_cast<unsigned long long>(tx.normal_write_failed),
        static_cast<unsigned long long>(tx.status_published),
        static_cast<unsigned long long>(tx.status_sent),
        static_cast<unsigned long long>(tx.status_write_failed),
        static_cast<unsigned long long>(tx.status_coalesced));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(response)) {
        // Long-lived counters can make the full engineering snapshot exceed
        // its fixed DRAM buffer. Preserve the active timing diagnosis instead
        // of silently dropping the command response.
        length = std::snprintf(
            response, sizeof(response),
            "[POS:truncated=1|state=%s|reason=%s|steps=(%lld,%lld)|timer=(events=%lu,jitter=(samples=%lu,max=%.2fus,boundary=%lu/%.2fus,over2/5/10=%lu/%lu/%lu,mask=(%lu,%lu,%lu,%lu)/(%lu,%lu,%lu,%lu),body=(samples=%lu,max=%.2fus,boundary=%lu/%.2fus,over10=%lu,wrap=%lu),hot=(mhz=%lu,samples=%lu,front=%.2fus,normal=%lu/%.2fus/%lu,boundary=%lu/%.2fus/%lu,peak=(interval=%lu->%lu,amass=%u->%u,complete=%d))))]\r\nok\r\n",
            position_state_text(position.state),
            position_invalidation_reason_text(position.invalidation_reason),
            static_cast<long long>(position.steps[0]),
            static_cast<long long>(position.steps[1]),
            static_cast<unsigned long>(timer.scheduled_timer_events),
            static_cast<unsigned long>(timer.timer_lateness_samples),
            static_cast<double>(timer.timer_lateness_max_ticks) /
                stepper::kStepTimerTicksPerMicrosecond,
            static_cast<unsigned long>(timer.timer_lateness_boundary_samples),
            static_cast<double>(timer.timer_lateness_boundary_max_ticks) /
                stepper::kStepTimerTicksPerMicrosecond,
            static_cast<unsigned long>(timer.timer_lateness_over_2us),
            static_cast<unsigned long>(timer.timer_lateness_over_5us),
            static_cast<unsigned long>(timer.timer_lateness_over_10us),
            static_cast<unsigned long>(timer.timer_lateness_mask_samples[0]),
            static_cast<unsigned long>(timer.timer_lateness_mask_samples[1]),
            static_cast<unsigned long>(timer.timer_lateness_mask_samples[2]),
            static_cast<unsigned long>(timer.timer_lateness_mask_samples[3]),
            static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[0]),
            static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[1]),
            static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[2]),
            static_cast<unsigned long>(timer.timer_lateness_over_10us_masks[3]),
            static_cast<unsigned long>(timer.timer_isr_body_samples),
            static_cast<double>(timer.timer_isr_body_max_ticks) /
                stepper::kStepTimerTicksPerMicrosecond,
            static_cast<unsigned long>(timer.timer_isr_body_boundary_samples),
            static_cast<double>(timer.timer_isr_body_boundary_max_ticks) /
                stepper::kStepTimerTicksPerMicrosecond,
            static_cast<unsigned long>(timer.timer_isr_body_over_10us),
            static_cast<unsigned long>(timer.timer_isr_body_counter_wrapped),
            static_cast<unsigned long>(timer.timer_isr_cycles_per_microsecond),
            static_cast<unsigned long>(timer.timer_isr_hot_path_samples),
            static_cast<double>(timer.timer_isr_pulse_front_max_cycles) /
                timer.timer_isr_cycles_per_microsecond,
            static_cast<unsigned long>(timer.timer_isr_normal_samples),
            static_cast<double>(timer.timer_isr_normal_max_cycles) /
                timer.timer_isr_cycles_per_microsecond,
            static_cast<unsigned long>(timer.timer_isr_normal_over_10us),
            static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_samples),
            static_cast<double>(timer.timer_isr_boundary_hot_path_max_cycles) /
                timer.timer_isr_cycles_per_microsecond,
            static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_over_10us),
            static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_peak_previous_interval_ticks),
            static_cast<unsigned long>(timer.timer_isr_boundary_hot_path_peak_next_interval_ticks),
            static_cast<unsigned>(timer.timer_isr_boundary_hot_path_peak_previous_amass_level),
            static_cast<unsigned>(timer.timer_isr_boundary_hot_path_peak_next_amass_level),
            timer.timer_isr_boundary_hot_path_peak_completes_block ? 1 : 0);
    }
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool set_known_position(const PositionCommand& command) {
    if (!planner::planner_is_idle() || sd::sd_job_is_active()) {
        return write_text("error:position-busy\r\n");
    }
    if (stepper::stepper_set_machine_position_mm(command.machine_position_mm) != ESP_OK) {
        return write_text("error:position-invalid\r\n");
    }

    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error sync_result = command::sync_parser_position_from_machine(s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    if (sync_result != command::Error::Ok) {
        return report_status(sync_result);
    }
    return report_position_status();
}

struct SdListResponseContext {
    bool write_failed = false;
};

bool write_sd_list_entry(const char* path, std::uintmax_t size, bool is_directory, void* user_data) {
    auto* context = static_cast<SdListResponseContext*>(user_data);
    char response[192] = {};
    const int length = std::snprintf(response, sizeof(response),
                                     "[FILE:%s|SIZE:%ju|TYPE:%s]\r\n",
                                     path, size, is_directory ? "DIR" : "FILE");
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(response) ||
        !write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length))) {
        context->write_failed = true;
        return false;
    }
    return true;
}

}  // namespace

esp_err_t command_dispatch_start() {
    if (s_parser_mutex != nullptr) {
        return ESP_OK;
    }

    s_parser_mutex = xSemaphoreCreateMutex();
    return s_parser_mutex == nullptr ? ESP_ERR_NO_MEM : ESP_OK;
}

bool command_report_realtime_status() {
    command::ParserState parser_state = {};
    if (!snapshot_parser_state(parser_state)) return write_text("error:protocol-no-mem\r\n");

    const stepper::MachinePosition position = stepper::machine_position();
    const stepper::SegmentExecutionDiagnostics segment = stepper::stepper_segment_diagnostics();
    const double feed_rate = segment.active
                                 ? static_cast<double>(segment.active_end_speed_tenths_mm_per_min) / 10.0
                                 : 0.0;
    static char response[160] = {};
    const int length = std::snprintf(
        response, sizeof(response), "<%s|MPos:%.3f,%.3f|FS:%.0f,%.0f>\r\n",
        realtime_state_text(position), position.millimeters[0], position.millimeters[1], feed_rate,
        parser_state.spindle_speed);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           protocol_tx_publish_realtime_status(
               reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool command_dispatch(const char* line, std::size_t length, bool& capacity_blocked) {
    capacity_blocked = false;
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) {
            return write_text("error:protocol-no-mem\r\n");
        }
    }

    if (equals_ignore_case(line, length, "$I")) {
        return report_build_info();
    }
    if (equals_ignore_case(line, length, "$G")) {
        return report_gcode_modes();
    }
    if (equals_ignore_case(line, length, "$#")) {
        return report_coordinate_parameters();
    }
    if (equals_ignore_case(line, length, "$$")) {
        return report_grbl_settings();
    }
    if (is_profile_self_test_command(line, length)) {
        return report_profile_self_test();
    }
    if (equals_ignore_case(line, length, "$X")) {
        // ray2-old returns ok outside Alarm as a no-op. RayCube currently has
        // no separate Alarm lock, so this must not promote an unknown position.
        return homing::homing_is_active() ? write_text("error:homing-busy\r\n") : write_text("ok\r\n");
    }
    const char* jog_payload = nullptr;
    std::size_t jog_payload_length = 0;
    if (parse_jog_command(line, length, jog_payload, jog_payload_length)) {
        return execute_jog_command(jog_payload, jog_payload_length);
    }
    if (is_jog_command(line, length)) {
        return report_status(command::Error::InvalidJogCommand);
    }

    if (is_homing_status_command(line, length)) {
        return report_homing_status();
    }
    homing::TestAxis homing_test_axis = {};
    if (parse_homing_seek_test_command(line, length, homing_test_axis)) {
        return start_homing_seek_test(homing_test_axis);
    }
    if (is_homing_command(line, length)) {
        return start_homing();
    }
    if (homing::homing_is_active()) {
        return write_text("error:homing-busy\r\n");
    }

    PositionCommand position_command = {};
    if (parse_position_command(line, length, position_command)) {
        return position_command.type == PositionCommandType::Status
                   ? report_position_status()
                   : set_known_position(position_command);
    }
    if (is_position_command(line, length)) {
        return write_text("error:position-invalid-command\r\n");
    }

    ray2::sd::SystemCommand sd_command = {};
    if (ray2::sd::parse_system_command(line, length, sd_command)) {
        switch (sd_command.type) {
            case ray2::sd::SystemCommandType::Status:
                if (!ray2::driver::sdcard_is_mounted()) {
                    return write_text("error:sd-not-mounted\r\n");
                }
                {
                    char response[192] = {};
                    const int response_length = std::snprintf(
                        response, sizeof(response),
                        "[SD:mounted path=/sd|job=%s|line=%u|fault=%s|error=%s]\r\nok\r\n",
                        ray2::sd::sd_job_state_text(ray2::sd::sd_job_state()),
                        static_cast<unsigned>(ray2::sd::sd_job_current_line()),
                        ray2::sd::sd_job_fault_text(ray2::sd::sd_job_fault()),
                        command::error_text(ray2::sd::sd_job_command_error()));
                    return response_length > 0 && static_cast<std::size_t>(response_length) < sizeof(response) &&
                           write_all(reinterpret_cast<const std::uint8_t*>(response),
                                     static_cast<std::size_t>(response_length));
                }
            case ray2::sd::SystemCommandType::List:
                if (!ray2::driver::sdcard_is_mounted()) {
                    return write_text("error:sd-not-mounted\r\n");
                }
                {
                    SdListResponseContext context = {};
                    if (!ray2::sd::list_root(write_sd_list_entry, &context) || context.write_failed) {
                        return write_text("error:sd-list-failed\r\n");
                    }
                    return write_text("ok\r\n");
                }
            case ray2::sd::SystemCommandType::Run: {
                if (ray2::stepper::machine_position_state() != ray2::stepper::PositionState::Known) {
                    return write_text("error:position-unknown\r\n");
                }
                const esp_err_t result = ray2::sd::sd_job_start(sd_command.path);
                if (result == ESP_ERR_INVALID_STATE) {
                    if (!ray2::driver::sdcard_is_mounted()) {
                        return write_text("error:sd-not-mounted\r\n");
                    }
                    return write_text("error:sd-job-active\r\n");
                }
                if (result != ESP_OK) {
                    return write_text("error:sd-run-failed\r\n");
                }
                char response[192] = {};
                const int response_length = std::snprintf(
                    response, sizeof(response), "[SD:run accepted path=%s]\r\nok\r\n", sd_command.path);
                return response_length > 0 && static_cast<std::size_t>(response_length) < sizeof(response) &&
                       write_all(reinterpret_cast<const std::uint8_t*>(response),
                                 static_cast<std::size_t>(response_length));
            }
        }
    }
    if (ray2::sd::is_system_command(line, length)) {
        return write_text("error:sd-invalid-command\r\n");
    }
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::execute_line(line, length, s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    if (result == command::Error::MotionBusy) {
        capacity_blocked = true;
        return true;
    }
    return report_status(result);
}

command::Error command_dispatch_external(const char* line, std::size_t length) {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) {
            return command::Error::MotionRejected;
        }
    }

    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::execute_line(line, length, s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    return result;
}

command::Error command_sync_parser_position_from_machine() {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) return command::Error::MotionRejected;
    }
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::sync_parser_position_from_machine(s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    return result;
}

}  // namespace ray2::protocol
