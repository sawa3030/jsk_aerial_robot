#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os
import re
from collections import defaultdict
from dataclasses import dataclass

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag


MOCAP_TOPIC_RE = re.compile(r"^/(?P<ns>[^/]+)/(?P<rotor>thrust\d+)/mocap/pose$")


@dataclass
class TransformState:
    parent: str
    translation: np.ndarray
    rotation: np.ndarray  # [x, y, z, w]
    stamp: float
    is_static: bool = False


def normalize_name(name):
    return str(name).strip().lstrip("/")


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_quaternion(quat):
    q = np.asarray(quat, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def quaternion_conjugate(quat):
    x, y, z, w = quat
    return np.array([-x, -y, -z, w], dtype=float)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def rotate_vector(quat, vec):
    q = normalize_quaternion(quat)
    vq = np.array([vec[0], vec[1], vec[2], 0.0], dtype=float)
    rotated = quaternion_multiply(quaternion_multiply(q, vq), quaternion_conjugate(q))
    return rotated[:3]


def compose_transform(t1, q1, t2, q2):
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)
    t = np.asarray(t1, dtype=float) + rotate_vector(q1, np.asarray(t2, dtype=float))
    q = normalize_quaternion(quaternion_multiply(q1, q2))
    return t, q


def quaternion_to_euler(quat):
    x, y, z, w = normalize_quaternion(quat)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_angle_deg(quat):
    q = normalize_quaternion(quat)
    w = max(-1.0, min(1.0, abs(q[3])))
    return math.degrees(2.0 * math.acos(w))


def extract_stamp(msg, fallback_time):
    header = getattr(msg, "header", None)
    if header is not None and hasattr(header, "stamp"):
        stamp = header.stamp.to_sec()
        if stamp > 0.0:
            return stamp
    return fallback_time


def extract_pose(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        pose = msg.pose.pose
    elif hasattr(msg, "pose"):
        pose = msg.pose
    elif hasattr(msg, "transform"):
        pose = msg.transform
    else:
        pose = msg

    if hasattr(pose, "position") and hasattr(pose, "orientation"):
        position = pose.position
        orientation = pose.orientation
    elif hasattr(pose, "translation") and hasattr(pose, "rotation"):
        position = pose.translation
        orientation = pose.rotation
    else:
        raise TypeError("Unsupported pose-like message type: {}".format(type(msg).__name__))

    translation = np.array([position.x, position.y, position.z], dtype=float)
    rotation = normalize_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return translation, rotation


def discover_mocap_topics(bag, namespace):
    topics = sorted(bag.get_type_and_topic_info().topics.keys())
    ns = normalize_name(namespace)
    discovered = []
    for topic in topics:
        match = MOCAP_TOPIC_RE.match(topic)
        if not match:
            continue
        if normalize_name(match.group("ns")) != ns:
            continue
        discovered.append(topic)
    return discovered


def topic_to_rotor_frame(topic):
    match = MOCAP_TOPIC_RE.match(topic)
    if not match:
        raise ValueError("Topic '{}' is not a recognized mocap rotor topic".format(topic))
    ns = normalize_name(match.group("ns"))
    rotor = normalize_name(match.group("rotor"))
    return rotor, "{}/{}".format(ns, rotor)


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
    print(trigger_time -bag_start, window_end-bag_start)

    return trigger_time, window_end


def get_world_to_child(current_transforms, world_frame, child_frame, sample_time, max_age):
    world = normalize_name(world_frame)
    child = normalize_name(child_frame)

    translations = []
    rotations = []
    ages = []
    visited = set()
    current = child

    while current != world:
        if current in visited:
            return None
        visited.add(current)

        state = current_transforms.get(current)
        if state is None:
            return None
        if max_age is not None and not state.is_static and sample_time - state.stamp > max_age:
            return None

        translations.append(state.translation)
        rotations.append(state.rotation)
        if not state.is_static:
            ages.append(sample_time - state.stamp)
        current = state.parent

    translation = np.zeros(3, dtype=float)
    rotation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    for t, q in reversed(list(zip(translations, rotations))):
        translation, rotation = compose_transform(translation, rotation, t, q)

    return {
        "translation": translation,
        "rotation": rotation,
        "max_age": max(ages) if ages else 0.0,
    }


def collect_error_series(
    bag_path,
    namespace,
    world_frame,
    max_tf_age,
    window_start,
    window_end,
    mocap_topics=None,
    tf_topic="/tf",
    tf_static_topic="/tf_static",
):
    current_transforms = {}
    series = defaultdict(lambda: defaultdict(list))
    skipped_missing_tf = defaultdict(int)
    skipped_bad_pose = defaultdict(int)

    with rosbag.Bag(bag_path, "r") as bag:
        if mocap_topics is None:
            mocap_topics = discover_mocap_topics(bag, namespace)
        if not mocap_topics:
            raise RuntimeError(
                "No mocap topics found under namespace '{}'. Expected topics like /{}/thrust1/mocap".format(
                    namespace, normalize_name(namespace)
                )
            )

        topics_to_read = list(mocap_topics) + [tf_topic, tf_static_topic]
        frame_map = {topic: topic_to_rotor_frame(topic) for topic in mocap_topics}

        for topic, msg, bag_time in bag.read_messages(topics=topics_to_read):
            sample_time = bag_time.to_sec()
            if sample_time > window_end:
                break

            if topic == tf_topic or topic == tf_static_topic:
                for transform in msg.transforms:
                    try:
                        translation, rotation = extract_pose(transform)
                    except TypeError:
                        continue
                    child = normalize_name(transform.child_frame_id)
                    parent = normalize_name(transform.header.frame_id)
                    stamp = extract_stamp(transform, sample_time)
                    current_transforms[child] = TransformState(
                        parent=parent,
                        translation=translation,
                        rotation=rotation,
                        stamp=stamp,
                        is_static=(topic == tf_static_topic),
                    )
                continue

            rotor_name, rotor_frame = frame_map[topic]
            try:
                mocap_translation, mocap_rotation = extract_pose(msg)
            except TypeError:
                skipped_bad_pose[rotor_name] += 1
                continue

            mocap_time = extract_stamp(msg, sample_time)
            if mocap_time < window_start:
                continue
            if mocap_time > window_end:
                continue

            estimated = get_world_to_child(
                current_transforms=current_transforms,
                world_frame=world_frame,
                child_frame=rotor_frame,
                sample_time=mocap_time,
                max_age=max_tf_age,
            )
            if estimated is None:
                skipped_missing_tf[rotor_name] += 1
                continue

            pos_error_world = estimated["translation"] - mocap_translation
            pos_error = rotate_vector(quaternion_conjugate(mocap_rotation), pos_error_world)
            delta_q = quaternion_multiply(quaternion_conjugate(mocap_rotation), estimated["rotation"])
            roll_err, pitch_err, yaw_err = quaternion_to_euler(delta_q)

            series[rotor_name]["time"].append(mocap_time)
            series[rotor_name]["dx"].append(pos_error[0])
            series[rotor_name]["dy"].append(pos_error[1])
            series[rotor_name]["dxy"].append(float(np.linalg.norm(pos_error[:2])))
            series[rotor_name]["dz"].append(pos_error[2])
            series[rotor_name]["roll_deg"].append(math.degrees(wrap_to_pi(roll_err)))
            series[rotor_name]["pitch_deg"].append(math.degrees(wrap_to_pi(pitch_err)) - 10 if rotor_name == "thrust1" else math.degrees(wrap_to_pi(pitch_err)) + 10)
            # series[rotor_name]["pitch_deg"].append(math.degrees(wrap_to_pi(pitch_err)))
            series[rotor_name]["yaw_deg"].append(math.degrees(wrap_to_pi(yaw_err)))
            series[rotor_name]["angle_deg"].append(quaternion_angle_deg(delta_q))
            series[rotor_name]["tf_age"].append(estimated["max_age"])
            series[rotor_name]["mocap_x"].append(mocap_translation[0])
            series[rotor_name]["mocap_y"].append(mocap_translation[1])
            series[rotor_name]["est_x"].append(estimated["translation"][0])
            series[rotor_name]["est_y"].append(estimated["translation"][1])

    return series, skipped_missing_tf, skipped_bad_pose


def rotor_sort_key(name):
    match = re.search(r"(\d+)$", name)
    if match:
        return int(match.group(1))
    return name


def summarize_series(series):
    lines = []
    for rotor_name in sorted(series.keys(), key=rotor_sort_key):
        values = series[rotor_name]
        dx = np.asarray(values["dx"], dtype=float)
        dy = np.asarray(values["dy"], dtype=float)
        dxy = np.asarray(values["dxy"], dtype=float)
        angle = np.asarray(values["angle_deg"], dtype=float)
        tf_age = np.asarray(values["tf_age"], dtype=float)
        if len(dx) == 0:
            continue
        lines.append(
            (
                rotor_name,
                len(dx),
                math.sqrt(float(np.mean(dx ** 2))),
                math.sqrt(float(np.mean(dy ** 2))),
                math.sqrt(float(np.mean(dxy ** 2))),
                float(np.mean(angle)),
                math.sqrt(float(np.mean(angle ** 2))),
                float(np.max(angle)),
                float(np.max(tf_age)),
            )
        )
    return lines


def print_summary(series, skipped_missing_tf, skipped_bad_pose):
    summary = summarize_series(series)
    if not summary:
        print("No synchronized mocap/TF samples were found.")
        return

    print(
        "{:<8s} {:>7s} {:>12s} {:>12s} {:>12s} {:>14s} {:>14s} {:>12s} {:>12s}".format(
            "rotor",
            "samples",
            "rmse_x_mocap[m]",
            "rmse_y_mocap[m]",
            "rmse_xy_mocap[m]",
            "mean_att[deg]",
            "rms_att[deg]",
            "max_att",
            "max_tf_age",
        )
    )
    for row in summary:
        print(
            "{:<8s} {:>7d} {:>12.5f} {:>12.5f} {:>12.5f} {:>14.3f} {:>14.3f} {:>12.3f} {:>12.4f}".format(
                *row
            )
        )

    for rotor_name in sorted(set(list(skipped_missing_tf.keys()) + list(skipped_bad_pose.keys())), key=rotor_sort_key):
        missing_tf = skipped_missing_tf.get(rotor_name, 0)
        bad_pose = skipped_bad_pose.get(rotor_name, 0)
        if missing_tf or bad_pose:
            print(
                "{}: skipped {} samples without valid TF chain, {} samples with unsupported pose type".format(
                    rotor_name, missing_tf, bad_pose
                )
            )


def write_csv(csv_path, series):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "rotor",
                "time",
                "dx_mocap",
                "dy_mocap",
                "dxy_mocap",
                "dz_mocap",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "angle_deg",
                "tf_age",
                "mocap_x",
                "mocap_y",
                "est_x",
                "est_y",
            ]
        )
        for rotor_name in sorted(series.keys(), key=rotor_sort_key):
            values = series[rotor_name]
            sample_count = len(values["time"])
            for i in range(sample_count):
                writer.writerow(
                    [
                        rotor_name,
                        values["time"][i],
                        values["dx"][i],
                        values["dy"][i],
                        values["dxy"][i],
                        values["dz"][i],
                        values["roll_deg"][i],
                        values["pitch_deg"][i],
                        values["yaw_deg"][i],
                        values["angle_deg"][i],
                        values["tf_age"][i],
                        values["mocap_x"][i],
                        values["mocap_y"][i],
                        values["est_x"][i],
                        values["est_y"][i],
                    ]
                )


def plot_series(series, output_path):
    rotor_names = sorted(series.keys(), key=rotor_sort_key)
    if not rotor_names:
        raise RuntimeError("Nothing to plot.")

    fig, axes = plt.subplots(2, len(rotor_names), figsize=(max(4 * len(rotor_names), 4), 5.6), squeeze=False)
    fig.suptitle("Rotor mocap vs estimated TF error", fontsize=14)

    for col, rotor_name in enumerate(rotor_names):
        values = series[rotor_name]
        t = np.asarray(values["time"], dtype=float)
        if len(t) == 0:
            continue
        t = t - t[0]

        ax_xy = axes[0][col]
        ax_att = axes[1][col]

        ax_xy.plot(t, values["dx"], label="x", color="#0072B2", linewidth=1.2)
        ax_xy.plot(t, values["dy"], label="y", color="#D55E00", linewidth=1.2)
        # ax_xy.plot(t, values["dxy"], label="xy norm in mocap frame", color="#2ca02c", linewidth=1.4, linestyle="--")
        ax_xy.set_title("{} position error".format(rotor_name))
        ax_xy.set_xlim(0, 25)
        ax_xy.set_ylim(-0.15, 0.15)
        ax_xy.set_xticks(np.arange(0, 25, 5))
        ax_xy.set_yticks([-0.1, 0.0, 0.1])
        ax_xy.text(1.0, -0.05, "[s]", transform=ax_xy.transAxes, ha="right", va="top", fontsize=10)
        ax_xy.text(-0.08, 1.0, "[m]", transform=ax_xy.transAxes, ha="left", va="bottom", fontsize=10)
        ax_xy.grid(True, linestyle=":", linewidth=0.8)
        ax_xy.legend(loc="upper center", ncol=2, fontsize=9)

        ax_att.plot(t, values["roll_deg"], label="roll", color="#009E73", linewidth=1.1)
        ax_att.plot(t, values["pitch_deg"], label="pitch", color="#E69F00", linewidth=1.1)
        ax_att.plot(t, values["yaw_deg"], label="yaw", color="#CC79A7", linewidth=1.1)
        # ax_att.plot(t, values["angle_deg"], label="angle norm", color="#d62728", linewidth=1.4, linestyle="--")
        ax_att.set_title("{} attitude error".format(rotor_name))
        ax_att.set_xlim(0, 25)
        ax_att.set_ylim(-15.0, 15.0)
        ax_att.set_xticks(np.arange(0, 25, 5))
        ax_att.set_yticks([-10, 0, 10])
        ax_att.text(1.0, -0.05, "[s]", transform=ax_att.transAxes, ha="right", va="top", fontsize=10)
        ax_att.text(-0.12, 1.0, "[deg]", transform=ax_att.transAxes, ha="left", va="bottom", fontsize=10)
        ax_att.grid(True, linestyle=":", linewidth=0.8)
        ax_att.legend(loc="upper center", ncol=3, fontsize=9)

    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.97])
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Compare rotor mocap truth topics like /hydrus/thrust1/mocap with the estimated rotor pose "
            "reconstructed from /tf and /tf_static, then plot xy and attitude errors."
        )
    )
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--namespace", default="hydrus", help="Robot namespace, e.g. hydrus")
    parser.add_argument("--world-frame", default="world", help="World frame name used in TF")
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
        "--max-tf-age",
        type=float,
        default=0.2,
        help="Maximum allowed age [s] of dynamic TF segments when matching mocap samples",
    )
    parser.add_argument("--tf-topic", default="/tf", help="Dynamic TF topic")
    parser.add_argument("--tf-static-topic", default="/tf_static", help="Static TF topic")
    parser.add_argument(
        "--output",
        help="Output PNG path. Default: <bag_basename>_rotor_mocap_estimation_error.png",
    )
    parser.add_argument(
        "--csv",
        help="Optional CSV path for per-sample error export",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    bag_path = os.path.abspath(args.bag)
    bag_stem = os.path.splitext(os.path.basename(bag_path))[0]
    trigger_topic = args.trigger_topic or "/{}/soft_joint_reference_interp".format(normalize_name(args.namespace))
    window_start, window_end = find_trigger_window(
        bag_path=bag_path,
        trigger_topic=trigger_topic,
        ignore_seconds=args.ignore_seconds,
        plot_duration=args.plot_duration,
    )
    output_path = args.output or os.path.join(
        os.path.dirname(bag_path), "{}_rotor_mocap_estimation_error.png".format(bag_stem)
    )

    series, skipped_missing_tf, skipped_bad_pose = collect_error_series(
        bag_path=bag_path,
        namespace=args.namespace,
        world_frame=args.world_frame,
        max_tf_age=args.max_tf_age,
        window_start=window_start,
        window_end=window_end,
        tf_topic=args.tf_topic,
        tf_static_topic=args.tf_static_topic,
    )

    if not any(len(values["time"]) > 0 for values in series.values()):
        raise RuntimeError(
            "No valid mocap/TF pairs were found in the requested {:.1f}-second window after trigger.".format(
                args.plot_duration
            )
        )

    plot_series(series, output_path)
    print_summary(series, skipped_missing_tf, skipped_bad_pose)
    print(
        "Trigger topic '{}' at {:.3f}s, plotting [{:.3f}, {:.3f}]".format(
            trigger_topic, window_start, window_start, window_end
        )
    )
    print("Saved plot to {}".format(output_path))

    if args.csv:
        csv_path = os.path.abspath(args.csv)
        write_csv(csv_path, series)
        print("Saved CSV to {}".format(csv_path))


if __name__ == "__main__":
    main()
