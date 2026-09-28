#!/usr/bin/env python3
"""Validate a machine profile and emit compile-time C++ constants."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any


AXIS_COUNT = 2
GPIO_MAX = 48


def fail(message: str) -> None:
    raise ValueError(f"machine profile: {message}")


def mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        fail(f"{path} must be an object")
    return value


def array(value: Any, path: str) -> list[Any]:
    if not isinstance(value, list):
        fail(f"{path} must be an array")
    return value


def integer(value: Any, path: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        fail(f"{path} must be an integer")
    if value < minimum or value > maximum:
        fail(f"{path} must be in [{minimum}, {maximum}]")
    return value


def positive_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        fail(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        fail(f"{path} must be finite and greater than zero")
    return result


def boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        fail(f"{path} must be a boolean")
    return value


def required(data: dict[str, Any], key: str, path: str) -> Any:
    if key not in data:
        fail(f"{path}.{key} is required")
    return data[key]


def cpp_bool(value: bool) -> str:
    return "true" if value else "false"


def cpp_float(value: float) -> str:
    rendered = format(value, ".9g")
    if "." not in rendered and "e" not in rendered.lower():
        rendered += ".0"
    return f"{rendered}F"


def validate_profile(document: Any) -> tuple[int, list[dict[str, Any]], dict[str, Any]]:
    root = mapping(document, "root")
    schema_version = integer(required(root, "schema_version", "root"), "schema_version", 1, 1)
    if schema_version != 1:
        fail("schema_version must be 1")
    pulse_width = integer(
        required(root, "step_pulse_microseconds", "root"),
        "step_pulse_microseconds",
        1,
        1000,
    )
    axes_input = array(required(root, "axes", "root"), "axes")
    if len(axes_input) != AXIS_COUNT:
        fail(f"axes must contain exactly {AXIS_COUNT} entries")

    used_pins: set[int] = set()
    axes: list[dict[str, Any]] = []
    for index, raw_axis in enumerate(axes_input):
        path = f"axes[{index}]"
        source = mapping(raw_axis, path)
        axis: dict[str, Any] = {}
        for field in ("step_pin", "direction_pin", "disable_pin", "limit_pin"):
            pin = integer(required(source, field, path), f"{path}.{field}", 0, GPIO_MAX)
            if pin in used_pins:
                fail(f"{path}.{field} duplicates GPIO{pin}")
            used_pins.add(pin)
            axis[field] = pin
        for field in ("direction_inverted", "homing_seek_positive_direction", "disable_active_high", "limit_active_low"):
            axis[field] = boolean(required(source, field, path), f"{path}.{field}")
        for field in ("steps_per_mm", "max_rate_mm_per_min", "acceleration_mm_per_sec2", "max_travel_mm"):
            axis[field] = positive_number(required(source, field, path), f"{path}.{field}")
        axes.append(axis)

    laser_source = mapping(required(root, "laser", "root"), "laser")
    laser: dict[str, Any] = {}
    for field in ("pwm_pin", "enable_pin"):
        pin = integer(required(laser_source, field, "laser"), f"laser.{field}", 0, GPIO_MAX)
        if pin in used_pins:
            fail(f"laser.{field} duplicates GPIO{pin}")
        used_pins.add(pin)
        laser[field] = pin
    for field in ("pwm_inverted", "enable_active_high"):
        laser[field] = boolean(required(laser_source, field, "laser"), f"laser.{field}")
    laser["pwm_frequency_hz"] = integer(
        required(laser_source, "pwm_frequency_hz", "laser"), "laser.pwm_frequency_hz", 1, 1000000
    )
    laser["max_power"] = integer(
        required(laser_source, "max_power", "laser"), "laser.max_power", 1, 4294967295
    )
    return pulse_width, axes, laser


def render(pulse_width: int, axes: list[dict[str, Any]], laser: dict[str, Any]) -> str:
    axis_rows = []
    for axis in axes:
        axis_rows.append(
            "    {\n"
            f"        .step_pin = {axis['step_pin']},\n"
            f"        .direction_pin = {axis['direction_pin']},\n"
            f"        .disable_pin = {axis['disable_pin']},\n"
            f"        .direction_inverted = {cpp_bool(axis['direction_inverted'])},\n"
            f"        .homing_seek_positive_direction = {cpp_bool(axis['homing_seek_positive_direction'])},\n"
            f"        .disable_active_high = {cpp_bool(axis['disable_active_high'])},\n"
            f"        .steps_per_mm = {cpp_float(axis['steps_per_mm'])},\n"
            f"        .max_rate_mm_per_min = {cpp_float(axis['max_rate_mm_per_min'])},\n"
            f"        .acceleration_mm_per_sec2 = {cpp_float(axis['acceleration_mm_per_sec2'])},\n"
            f"        .max_travel_mm = {cpp_float(axis['max_travel_mm'])},\n"
            f"        .limit_pin = {axis['limit_pin']},\n"
            f"        .limit_active_low = {cpp_bool(axis['limit_active_low'])},\n"
            "    },"
        )
    return "\n".join(
        [
            "// Generated from config/machine_profile.json. Do not edit.",
            "#pragma once",
            "",
            "namespace laserctl::machine::generated {",
            "",
            f"inline constexpr unsigned int kStepPulseMicroseconds = {pulse_width};",
            "inline constexpr AxisHardware kAxes[kAxisCount] = {",
            *axis_rows,
            "};",
            "",
            "inline constexpr LaserHardware kLaser = {",
            f"    .pwm_pin = {laser['pwm_pin']},",
            f"    .enable_pin = {laser['enable_pin']},",
            f"    .pwm_inverted = {cpp_bool(laser['pwm_inverted'])},",
            f"    .enable_active_high = {cpp_bool(laser['enable_active_high'])},",
            f"    .pwm_frequency_hz = {laser['pwm_frequency_hz']}U,",
            f"    .max_power = {laser['max_power']}U,",
            "};",
            "",
            "}  // namespace laserctl::machine::generated",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()

    try:
        source = arguments.input.read_text(encoding="utf-8")
        pulse_width, axes, laser = validate_profile(json.loads(source))
        output = render(pulse_width, axes, laser)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", newline="\n", dir=arguments.output.parent, delete=False
    ) as temporary:
        temporary.write(output)
        temporary_path = Path(temporary.name)
    os.replace(temporary_path, arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
