#!/usr/bin/env python3
"""
Reads walk_test_bag and reports peak velocity and peak acceleration
per joint. Velocity is resampled onto a fixed 50 Hz time grid before
differentiating to acceleration, so that occasional near-duplicate
/joint_states timestamps (dt near zero) don't produce nonsense spikes.
"""

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

BAG_PATH = "walk_test_bag"   # <-- update to match whichever bag you're analyzing
RESAMPLE_RATE_HZ = 50.0        # matches the 0.02s control loop period

def main():
    storage_options = StorageOptions(uri=BAG_PATH, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    joint_velocity_history = {}  # joint_name -> list of (time_sec, velocity)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/joint_states":
            continue

        msg = deserialize_message(data, JointState)
        time_sec = t * 1e-9

        for name, vel in zip(msg.name, msg.velocity):
            joint_velocity_history.setdefault(name, []).append((time_sec, vel))

    print(f"{'Joint':<24}{'Peak |velocity| (rad/s)':<28}{'Peak |accel| (rad/s^2)'}")
    print("-" * 80)

    dt_grid = 1.0 / RESAMPLE_RATE_HZ

    for joint_name, samples in joint_velocity_history.items():
        samples.sort(key=lambda pair: pair[0])
        times = np.array([s[0] for s in samples])
        vels = np.array([s[1] for s in samples])

        # Drop exact-duplicate timestamps (would break np.interp)
        times, unique_idx = np.unique(times, return_index=True)
        vels = vels[unique_idx]

        peak_vel = np.abs(vels).max()

        # Resample onto an evenly-spaced grid at RESAMPLE_RATE_HZ
        grid_times = np.arange(times[0], times[-1], dt_grid)
        grid_vels = np.interp(grid_times, times, vels)

        # Now differentiate on the *uniform* grid -- dt is constant (dt_grid),
        # so no more division-by-tiny-dt spikes.
        accel = np.diff(grid_vels) / dt_grid
        peak_accel = np.abs(accel).max()

        print(f"{joint_name:<24}{peak_vel:<28.4f}{peak_accel:.4f}")


if __name__ == "__main__":
    main()
