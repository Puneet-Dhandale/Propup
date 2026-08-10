#!/usr/bin/env python3
"""
Reads walk_test_bag and reports peak velocity and peak acceleration
per joint, for the presentation.
"""

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

BAG_PATH = "walk_test_bag"

def main():
    storage_options = StorageOptions(uri=BAG_PATH, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # dict: joint_name -> list of (time_sec, velocity) tuples, in time order
    joint_velocity_history = {}

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/joint_states":
            continue

        msg = deserialize_message(data, JointState)
        time_sec = t * 1e-9  # bag timestamps are in nanoseconds

        for name, vel in zip(msg.name, msg.velocity):
            if name not in joint_velocity_history:
                joint_velocity_history[name] = []
            joint_velocity_history[name].append((time_sec, vel))

    print(f"{'Joint':<24}{'Peak |velocity| (rad/s)':<28}{'Peak |accel| (rad/s^2)'}")
    print("-" * 80)

    for joint_name, samples in joint_velocity_history.items():
        # sort by time just in case messages weren't strictly ordered
        samples.sort(key=lambda pair: pair[0])
        times = np.array([s[0] for s in samples])
        vels = np.array([s[1] for s in samples])

        peak_vel = np.abs(vels).max()

        # acceleration = d(velocity)/dt, same finite-difference idea as
        # velocity = d(position)/dt -- just one level up.
        # np.diff gives consecutive differences; dt varies since /joint_states
        # isn't published at a perfectly fixed rate, so divide element-wise.
        dv = np.diff(vels)
        dt = np.diff(times)
        # guard against any duplicate/zero-dt timestamps
        dt[dt == 0] = np.nan
        accel = dv / dt
        peak_accel = np.nanmax(np.abs(accel))

        print(f"{joint_name:<24}{peak_vel:<28.4f}{peak_accel:.4f}")


if __name__ == "__main__":
    main()
