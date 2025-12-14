#!/usr/bin/env python3
"""Generate comparison plots between actual and ANSWER JSON trajectories for OCP scenarios.

Usage:
  python3 tools/plot_ocp_compare.py
  python3 tools/plot_ocp_compare.py --scenario 1
  python3 tools/plot_ocp_compare.py --base_dir /path/to/reference_IO

This script parses jsonl trajectories (supporting both wrapped and bare payloads,
variable timestamps, etc.), computes key metrics, and saves matplotlib PNG graphs per scenario.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt

DEFAULT_BASE_DIR = Path("/home/sujin/Desktop/obstacle_cruise_planner/reference_IO")
SCENARIO_MAP = {
    1: "scenario1",
    2: "scenario2",
    3: "scenario3",
}
TRAJECTORY_FILENAME = "planning__scenario_planning__lane_driving__trajectory.jsonl"
ANSWER_FILENAME = "ANSWER_planning__scenario_planning__lane_driving__trajectory.jsonl"
OBJECTS_FILENAME = "perception__object_recognition__objects.jsonl"
OUTPUT_ROOT = Path("/home/sujin/Desktop/obstacle_cruise_planner/tools/plots")
STOP_VELOCITY_EPS = 1e-3


def normalize_timestamp(value: Any) -> float:
    if isinstance(value, dict):
        sec = float(value.get("sec", value.get("seconds", 0)))
        nsec = float(value.get("nanosec", value.get("nanos", 0)))
        return sec + nsec * 1e-9
    if isinstance(value, (int, float)):
        as_float = float(value)
        if as_float > 1e12:
            return as_float / 1e9
        return as_float
    return 0.0


def extract_payload(entry: Dict[str, Any]) -> Dict[str, Any]:
    for key in ("message", "msg", "payload"):
        if key in entry and isinstance(entry[key], dict):
            return entry[key]
    return entry


def extract_frame_stamp(entry: Dict[str, Any], payload: Dict[str, Any]) -> float:
    if "timestamp" in entry:
        timestamp = entry["timestamp"]
        return normalize_timestamp(timestamp)
    for container in (payload, payload.get("header", {})):
        if not isinstance(container, dict):
            continue
        if "stamp" in container:
            return normalize_timestamp(container["stamp"])
        if "time" in container:
            return normalize_timestamp(container["time"])
    return 0.0


def load_jsonl_frames(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(f"{path} missing")
    frames: List[Dict[str, Any]] = []
    with path.open() as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            payload = extract_payload(entry)
            stamp = extract_frame_stamp(entry, payload)
            points = payload.get("points")
            if not isinstance(points, list):
                points = []
            frames.append({"stamp": stamp, "points": points})
    return frames


def quaternion_to_yaw(quat: Dict[str, Any]) -> float:
    x = float(quat.get("x", 0.0))
    y = float(quat.get("y", 0.0))
    z = float(quat.get("z", 0.0))
    w = float(quat.get("w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compute_metrics(points: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not points:
        return {
            "final_velocity": 0.0,
            "stop_index": -1,
            "stop_distance": 0.0,
            "last_x": 0.0,
            "last_y": 0.0,
            "last_yaw": 0.0,
        }
    velocities = [float(pt.get("longitudinal_velocity_mps", 0.0)) for pt in points]
    final_point = points[-1]
    final_velocity = velocities[-1]
    stop_index = next((idx for idx, vel in enumerate(velocities) if vel <= STOP_VELOCITY_EPS), -1)
    stop_distance = 0.0
    if stop_index > 0:
        for idx in range(1, stop_index + 1):
            prev = points[idx - 1]["pose"]["position"]
            current = points[idx]["pose"]["position"]
            dx = float(current.get("x", 0.0)) - float(prev.get("x", 0.0))
            dy = float(current.get("y", 0.0)) - float(prev.get("y", 0.0))
            stop_distance += math.hypot(dx, dy)
    else:
        stop_distance = 0.0
    last_pose = final_point.get("pose", {}).get("position", {})
    last_x = float(last_pose.get("x", 0.0))
    last_y = float(last_pose.get("y", 0.0))
    orientation = final_point.get("pose", {}).get("orientation", {})
    last_yaw = quaternion_to_yaw(orientation if isinstance(orientation, dict) else {})
    return {
        "final_velocity": final_velocity,
        "stop_index": stop_index,
        "stop_distance": stop_distance,
        "last_x": last_x,
        "last_y": last_y,
        "last_yaw": last_yaw,
    }


def compute_per_frame_metrics(frames: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [compute_metrics(frame["points"]) for frame in frames]


def plot_overlay(
    actual: List[float],
    answer: List[float],
    title: str,
    ylabel: str,
    output_path: Path,
    first_frame_idx: int = 0,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(range(first_frame_idx, first_frame_idx + len(actual)), actual, label="actual", linewidth=1.5)
    ax.plot(range(first_frame_idx, first_frame_idx + len(answer)), answer, label="answer", linewidth=1.2, linestyle="--")
    ax.set_title(title)
    ax.set_xlabel("frame index")
    ax.set_ylabel(ylabel)
    ax.legend(loc="upper right", fontsize="small")
    ax.grid(True, linestyle="--", linewidth=0.5)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_error(
    errors: List[float],
    title: str,
    ylabel: str,
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(range(len(errors)), errors, label="error", linewidth=1.5, color="tab:red")
    ax.set_title(title)
    ax.set_xlabel("frame index")
    ax.set_ylabel(ylabel)
    ax.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
    ax.grid(True, linestyle="--", linewidth=0.5)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_xy_error(
    dx: List[float],
    dy: List[float],
    title: str,
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(range(len(dx)), dx, label="Δx_last", linewidth=1.5)
    ax.plot(range(len(dy)), dy, label="Δy_last", linewidth=1.5)
    ax.set_title(title)
    ax.set_xlabel("frame index")
    ax.set_ylabel("meters")
    ax.legend(loc="upper right", fontsize="small")
    ax.grid(True, linestyle="--", linewidth=0.5)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_obstacle_flag(
    flags: List[int],
    title: str,
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 2))
    ax.fill_between(range(len(flags)), 0, flags, step="post", alpha=0.3, color="tab:purple")
    ax.step(range(len(flags)), flags, where="post", label="obstacle present")
    ax.set_ylim(-0.1, 1.1)
    ax.set_yticks([0, 1])
    ax.set_title(title)
    ax.set_xlabel("frame index")
    ax.set_ylabel("flag")
    ax.grid(True, linestyle="--", linewidth=0.5)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def load_obstacle_events(path: Path) -> List[Tuple[float, int]]:
    events: List[Tuple[float, int]] = []
    if not path.exists():
        return events
    with path.open() as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            payload = extract_payload(entry)
            stamp = extract_frame_stamp(entry, payload)
            objects = payload.get("objects")
            if isinstance(objects, list):
                count = len(objects)
            else:
                count = 0
            events.append((stamp, count))
    return events


def map_frames_to_obstacle_flags(
    frame_stamps: Iterable[float], object_events: List[Tuple[float, int]]
) -> Tuple[List[int], int]:
    flags: List[int] = []
    idx = 0
    current_flag = 0
    total = 0
    for stamp in frame_stamps:
        while idx < len(object_events) and object_events[idx][0] <= stamp:
            current_flag = 1 if object_events[idx][1] > 0 else 0
            idx += 1
        flags.append(current_flag)
        total += current_flag
    return flags, total


def ensure_output_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def process_scenario(index: int, base_dir: Path) -> None:
    scenario_name = SCENARIO_MAP[index]
    scenario_root = base_dir / scenario_name
    output_dir = OUTPUT_ROOT / scenario_name
    ensure_output_dir(output_dir)

    actual_path = scenario_root / TRAJECTORY_FILENAME
    answer_path = scenario_root / ANSWER_FILENAME
    objects_path = scenario_root / OBJECTS_FILENAME
    try:
        actual_frames = load_jsonl_frames(actual_path)
        answer_frames = load_jsonl_frames(answer_path)
    except FileNotFoundError as exc:
        print(f"Scenario {index}: {exc}")
        return

    frames_compared = min(len(actual_frames), len(answer_frames))
    if len(actual_frames) != len(answer_frames):
        print(
            f"Scenario {index}: frame count mismatch (actual={len(actual_frames)}, answer={len(answer_frames)})"
        )

    actual_metrics = compute_per_frame_metrics(actual_frames)
    answer_metrics = compute_per_frame_metrics(answer_frames)

    point_mismatches = sum(
        1
        for idx in range(frames_compared)
        if len(actual_frames[idx]["points"]) != len(answer_frames[idx]["points"])
    )

    final_vel_actual = [m["final_velocity"] for m in actual_metrics]
    final_vel_answer = [m["final_velocity"] for m in answer_metrics]
    stop_dist_actual = [m["stop_distance"] for m in actual_metrics]
    stop_dist_answer = [m["stop_distance"] for m in answer_metrics]

    dx_last = []
    dy_last = []
    final_vel_errors = []
    stop_dist_errors = []
    stop_index_errors = []
    yaw_actual = [m["last_yaw"] for m in actual_metrics]
    yaw_answer = [m["last_yaw"] for m in answer_metrics]

    for idx in range(frames_compared):
        actual = actual_metrics[idx]
        answer = answer_metrics[idx]
        final_vel_errors.append(actual["final_velocity"] - answer["final_velocity"])
        stop_dist_errors.append(actual["stop_distance"] - answer["stop_distance"])
        stop_index_errors.append(actual["stop_index"] - answer["stop_index"])
        dx_last.append(actual["last_x"] - answer["last_x"])
        dy_last.append(actual["last_y"] - answer["last_y"])

    plot_overlay(
        final_vel_actual,
        final_vel_answer,
        "Final velocity (actual vs answer)",
        "velocity [m/s]",
        output_dir / "final_velocity_overlay.png",
    )
    plot_error(
        [abs(err) for err in final_vel_errors[:frames_compared]],
        "Δfinal_velocity",
        "m/s",
        output_dir / "final_velocity_error.png",
    )
    plot_overlay(
        stop_dist_actual,
        stop_dist_answer,
        "Stop distance (actual vs answer)",
        "distance [m]",
        output_dir / "stop_distance_overlay.png",
    )
    plot_xy_error(
        dx_last[:frames_compared],
        dy_last[:frames_compared],
        "Δlast position (x/y)",
        output_dir / "xy_last_error.png",
    )
    plot_overlay(
        yaw_actual,
        yaw_answer,
        "Yaw (actual vs answer)",
        "radians",
        output_dir / "yaw_overlay.png",
    )

    object_flag_count = 0
    if index == 3 and objects_path.exists():
        object_events = load_obstacle_events(objects_path)
        frame_stamps = [frame["stamp"] for frame in actual_frames[:frames_compared]]
        flags, object_flag_count = map_frames_to_obstacle_flags(frame_stamps, object_events)
        plot_obstacle_flag(
            flags,
            "Obstacle presence (Scenario3)",
            output_dir / "obstacle_flag.png",
        )

    max_final_vel_err = max((abs(val) for val in final_vel_errors[:frames_compared]), default=0.0)
    max_stop_dist_err = max((abs(val) for val in stop_dist_errors[:frames_compared]), default=0.0)
    max_dx = max((abs(val) for val in dx_last[:frames_compared]), default=0.0)
    max_dy = max((abs(val) for val in dy_last[:frames_compared]), default=0.0)

    print(f"Scenario {index} summary:")
    print(f"  frames compared: {frames_compared}")
    print(f"  points per-frame mismatches: {point_mismatches}")
    print(f"  max |Δfinal_velocity| = {max_final_vel_err:.6f} m/s")
    print(f"  max |Δstop_distance| = {max_stop_dist_err:.6f} m")
    print(f"  max |Δx_last| = {max_dx:.6f} m, max |Δy_last| = {max_dy:.6f} m")
    if index == 3 and objects_path.exists():
        print(f"  frames where objects present: {object_flag_count}")


def parse_scenarios(value: Optional[str]) -> List[int]:
    if value is None:
        return [1, 2, 3]
    selected = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        num = int(token)
        if num not in SCENARIO_MAP:
            raise ValueError(f"unknown scenario {token}")
        selected.append(num)
    return selected


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot OCP scenario comparisons.")
    parser.add_argument(
        "--scenario",
        type=str,
        help="comma-separated scenario numbers (1/2/3) to process; default=1,2,3",
    )
    parser.add_argument(
        "--base_dir",
        type=Path,
        default=DEFAULT_BASE_DIR,
        help="reference IO base directory",
    )
    args = parser.parse_args()
    try:
        scenarios = parse_scenarios(args.scenario)
    except ValueError as exc:
        parser.error(str(exc))
    for scenario_index in scenarios:
        process_scenario(scenario_index, args.base_dir)


if __name__ == "__main__":
    main()
