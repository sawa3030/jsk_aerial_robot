#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag


AXES = ("x", "y", "z", "roll", "pitch", "yaw")
POSITION_AXES = ("x", "y", "z")
ATTITUDE_AXES = ("roll", "pitch", "yaw")


def normalize_name(name):
    return str(name).strip().lstrip("/")


def extract_stamp(msg, fallback_time):
    header = getattr(msg, "header", None)
    if header is not None and hasattr(header, "stamp"):
        stamp = header.stamp.to_sec()
        if stamp > 0.0:
            return stamp
    return fallback_time


def find_trigger_window(bag_path, trigger_topic, ignore_seconds, plot_duration):
    with rosbag.Bag(bag_path, "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        search_start = bag_start + ignore_seconds

        trigger_time = None
        for _, msg, bag_time in bag.read_messages(topics=[trigger_topic]):
            sample_time = extract_stamp(msg, bag_time.to_sec())
            if sample_time >= search_start:
                trigger_time = sample_time
                break

    if trigger_time is None:
        raise RuntimeError(
            "No message on '{}' was found after {:.1f} seconds from bag start.".format(
                trigger_topic, ignore_seconds
            )
        )

    window_end = trigger_time + plot_duration
    if bag_end < window_end:
        raise RuntimeError(
            "Bag ends at {:.3f}, but {:.1f} seconds are required after trigger time {:.3f}.".format(
                bag_end, plot_duration, trigger_time
            )
        )

    return trigger_time, window_end


def find_fixed_window(bag_path, window_start_seconds, window_end_seconds):
    if window_end_seconds <= window_start_seconds:
        raise ValueError(
            "window_end_seconds ({:.3f}) must be greater than window_start_seconds ({:.3f}).".format(
                window_end_seconds, window_start_seconds
            )
        )

    with rosbag.Bag(bag_path, "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()

    window_start = bag_start + window_start_seconds
    window_end = bag_start + window_end_seconds
    if window_end > bag_end:
        raise RuntimeError(
            "Bag ends at {:.3f}, but fixed window end is {:.3f}.".format(bag_end, window_end)
        )

    return window_start, window_end


def init_series():
    series = {"time": []}
    for axis in AXES:
        series[axis] = []
    return series


def collect_pid_error_series(bag_path, pid_topic, window_start, window_end):
    series = init_series()

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[pid_topic]):
            sample_time = extract_stamp(msg, bag_time.to_sec())
            if sample_time < window_start:
                continue
            if sample_time > window_end:
                break

            series["time"].append(sample_time)
            for axis in AXES:
                series[axis].append(getattr(getattr(msg, axis), "err_p"))

    return series


def summarize_series(series):
    summary = []
    for axis in AXES:
        values = np.asarray(series[axis], dtype=float)
        if len(values) == 0:
            continue
        summary.append(
            (
                axis,
                len(values),
                float(np.mean(values)),
                math.sqrt(float(np.mean(values ** 2))),
                float(np.max(np.abs(values))),
            )
        )
    return summary


def print_summary(series):
    summary = summarize_series(series)
    if not summary:
        print("No PID error samples were found.")
        return

    print(
        "{:<8s} {:>7s} {:>12s} {:>12s} {:>12s}".format(
            "axis", "samples", "mean_err_p", "rms_err_p", "max_abs_err"
        )
    )
    for row in summary:
        print("{:<8s} {:>7d} {:>12.5f} {:>12.5f} {:>12.5f}".format(*row))


def write_csv(csv_path, series):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "x_err_p", "y_err_p", "z_err_p", "roll_err_p", "pitch_err_p", "yaw_err_p"])
        sample_count = len(series["time"])
        for i in range(sample_count):
            writer.writerow(
                [
                    series["time"][i],
                    series["x"][i],
                    series["y"][i],
                    series["z"][i],
                    series["roll"][i],
                    series["pitch"][i],
                    series["yaw"][i],
                ]
            )


def plot_series(series, output_path, plot_duration):
    if not series["time"]:
        raise RuntimeError("Nothing to plot.")

    t = np.asarray(series["time"], dtype=float)
    t = t - t[0]

    fig, axes = plt.subplots(2, 1, figsize=(9, 6.5), squeeze=False)
    fig.suptitle("COG pose error from debug/pose/pid", fontsize=14)

    ax_pos = axes[0][0]
    ax_att = axes[1][0]

    ax_pos.plot(t, series["x"], label="x", color="#1f77b4", linewidth=1.2)
    ax_pos.plot(t, series["y"], label="y", color="#ff7f0e", linewidth=1.2)
    ax_pos.plot(t, series["z"], label="z", color="#2ca02c", linewidth=1.2)
    ax_pos.set_title("Position error")
    ax_pos.set_xlim(0, plot_duration)
    ax_pos.set_ylabel("[m]")
    ax_pos.grid(True, linestyle=":", linewidth=0.8)
    ax_pos.legend(loc="upper right", ncol=3, fontsize=9)

    ax_att.plot(t, series["roll"], label="roll", color="#9467bd", linewidth=1.2)
    ax_att.plot(t, series["pitch"], label="pitch", color="#8c564b", linewidth=1.2)
    ax_att.plot(t, series["yaw"], label="yaw", color="#e377c2", linewidth=1.2)
    ax_att.set_title("Attitude error")
    ax_att.set_xlim(0, plot_duration)
    ax_att.set_xlabel("[s]")
    ax_att.set_ylabel("[rad]")
    ax_att.grid(True, linestyle=":", linewidth=0.8)
    ax_att.legend(loc="upper right", ncol=3, fontsize=9)

    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.97])
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Plot COG position / attitude error directly from /<namespace>/debug/pose/pid "
            "using each axis's err_p field."
        )
    )
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--namespace", default="hydrus", help="Robot namespace, e.g. hydrus")
    parser.add_argument(
        "--window-start-seconds",
        type=float,
        help="Optional fixed plot window start [s] from bag start",
    )
    parser.add_argument(
        "--window-end-seconds",
        type=float,
        help="Optional fixed plot window end [s] from bag start",
    )
    parser.add_argument(
        "--ignore-seconds",
        type=float,
        default=30.0,
        help="Ignore the first N seconds from bag start before searching the trigger topic",
    )
    parser.add_argument(
        "--plot-duration",
        type=float,
        default=25.0,
        help="Plot this many seconds from the first trigger message after --ignore-seconds",
    )
    parser.add_argument(
        "--trigger-topic",
        help="Trigger topic. Default: /<namespace>/soft_joint_reference_interp",
    )
    parser.add_argument(
        "--pid-topic",
        help="PID debug topic. Default: /<namespace>/debug/pose/pid",
    )
    parser.add_argument(
        "--output",
        help="Output PNG path. Default: <bag_basename>_rotor_mocap_estimation_error.png",
    )
    parser.add_argument("--csv", help="Optional CSV path for per-sample error export")
    return parser.parse_args()


def main():
    args = parse_args()

    bag_path = os.path.abspath(args.bag)
    bag_stem = os.path.splitext(os.path.basename(bag_path))[0]
    namespace = normalize_name(args.namespace)
    trigger_topic = args.trigger_topic or "/{}/soft_joint_reference_interp".format(namespace)
    pid_topic = args.pid_topic or "/{}/debug/pose/pid".format(namespace)

    use_fixed_window = (
        args.window_start_seconds is not None or args.window_end_seconds is not None
    )
    if use_fixed_window:
        if args.window_start_seconds is None or args.window_end_seconds is None:
            raise ValueError("Both --window-start-seconds and --window-end-seconds must be specified together.")
        window_start, window_end = find_fixed_window(
            bag_path=bag_path,
            window_start_seconds=args.window_start_seconds,
            window_end_seconds=args.window_end_seconds,
        )
        plot_duration = args.window_end_seconds - args.window_start_seconds
    else:
        window_start, window_end = find_trigger_window(
            bag_path=bag_path,
            trigger_topic=trigger_topic,
            ignore_seconds=args.ignore_seconds,
            plot_duration=args.plot_duration,
        )
        plot_duration = args.plot_duration

    output_path = args.output or os.path.join(
        os.path.dirname(bag_path), "{}_rotor_mocap_estimation_error.png".format(bag_stem)
    )

    series = collect_pid_error_series(
        bag_path=bag_path,
        pid_topic=pid_topic,
        window_start=window_start,
        window_end=window_end,
    )

    if not series["time"]:
        if use_fixed_window:
            raise RuntimeError(
                "No PID error samples were found on '{}' in the requested fixed window [{:.3f}, {:.3f}] seconds.".format(
                    pid_topic, args.window_start_seconds, args.window_end_seconds
                )
            )
        raise RuntimeError(
            "No PID error samples were found on '{}' in the requested {:.1f}-second window after trigger.".format(
                pid_topic, args.plot_duration
            )
        )

    plot_series(series, output_path, plot_duration)
    print_summary(series)
    if use_fixed_window:
        print(
            "Plotting fixed bag window [{:.3f}, {:.3f}] seconds (absolute stamps [{:.3f}, {:.3f}]).".format(
                args.window_start_seconds, args.window_end_seconds, window_start, window_end
            )
        )
    else:
        print(
            "Trigger topic '{}' at {:.3f}s, plotting [{:.3f}, {:.3f}]".format(
                trigger_topic, window_start, window_start, window_end
            )
        )
    print("PID topic: {}".format(pid_topic))
    print("Saved plot to {}".format(output_path))

    if args.csv:
        csv_path = os.path.abspath(args.csv)
        write_csv(csv_path, series)
        print("Saved CSV to {}".format(csv_path))


if __name__ == "__main__":
    main()
