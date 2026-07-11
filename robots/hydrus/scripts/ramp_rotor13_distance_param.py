#!/usr/bin/env python

import argparse
import math

import rospy


def main():
    parser = argparse.ArgumentParser(
        description="Ramp /pub_soft_link_joint_states/rotor13_distance over time."
    )
    parser.add_argument("--param", default="pub_soft_link_joint_states/rotor13_distance")
    parser.add_argument("--start", type=float, default=None)
    parser.add_argument("--end", type=float, default=0.3)
    parser.add_argument("--step", type=float, default=0.005, help="Step size per update")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--confirm-interval",
        type=float,
        default=0.1,
        help="Ask confirmation every this much absolute progress; <=0 disables",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    duration = max(args.duration, 0.0)
    step = max(abs(args.step), 1.0e-6)

    # Keep SIGINT handling in Python so Ctrl+C interrupts blocking input()/sleep.
    rospy.init_node("ramp_rotor13_distance_param", anonymous=True, disable_signals=True)

    if args.start is None:
        start_value = float(rospy.get_param(args.param, 0.8))
    else:
        start_value = args.start

    diff = args.end - start_value
    steps = max(int(math.ceil(abs(diff) / step)), 1)
    interval = duration / float(steps)
    confirm_interval = max(args.confirm_interval, 0.0)
    next_confirm_at = confirm_interval if confirm_interval > 0.0 else None

    print(
        "Start ramp: param={0}, current_start={1:.4f}, end={2:.4f}, step={3:.4f}, duration={4:.2f}s, steps={5}".format(
            args.param, start_value, args.end, step, duration, steps
        )
    )

    for i in range(steps + 1):
        if i == 0:
            value = start_value
        elif i == steps:
            value = args.end
        else:
            moved = min(i * step, abs(diff))
            value = start_value + math.copysign(moved, diff)
        rospy.set_param(args.param, value)
        # if i == 0 or i == steps or i % max(steps // 10, 1) == 0:
        print("step {0:4d}/{1:4d}: {2:.4f}".format(i, steps, value))
        if i < steps and next_confirm_at is not None:
            moved_abs = abs(value - start_value)
            while moved_abs + 1.0e-9 >= next_confirm_at:
                try:
                    answer = input(
                        "Reached +{0:.1f} from start. Continue ramp? [Y/n]: ".format(
                            next_confirm_at
                        )
                    ).strip().lower()
                except KeyboardInterrupt:
                    print("\nStopped by Ctrl+C at step {0}/{1}, value={2:.4f}".format(i, steps, value))
                    return
                if answer not in ("y", "yes"):
                    print("Stopped by user at step {0}/{1}, value={2:.4f}".format(i, steps, value))
                    return
                next_confirm_at += confirm_interval
        if i < steps:
            try:
                rospy.sleep(interval)
            except KeyboardInterrupt:
                print("\nStopped by Ctrl+C at step {0}/{1}, value={2:.4f}".format(i, steps, value))
                return

    print("Done ramp: {0:.4f} -> {1:.4f}".format(start_value, args.end))
    rospy.loginfo(
        "Done: %s changed from %.4f to %.4f in %.2f sec",
        args.param,
        start_value,
        args.end,
        duration,
    )


if __name__ == "__main__":
    main()
