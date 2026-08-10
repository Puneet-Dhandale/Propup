#!/usr/bin/env python3
"""Diagnostic: checks how evenly /joint_states messages are spaced in time."""

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

BAG_PATH = "walk_test_bag"

storage_options = StorageOptions(uri=BAG_PATH, storage_id="sqlite3")
converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
reader = SequentialReader()
reader.open(storage_options, converter_options)

times = []
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/joint_states":
        times.append(t * 1e-9)

times = np.array(sorted(times))
dt = np.diff(times)

print(f"Total messages: {len(times)}")
print(f"dt min:    {dt.min():.6f} s")
print(f"dt max:    {dt.max():.6f} s")
print(f"dt mean:   {dt.mean():.6f} s")
print(f"dt median: {np.median(dt):.6f} s")
print(f"dt std:    {dt.std():.6f} s")
print(f"# of dt values under 0.0005s (2000Hz+): {(dt < 0.0005).sum()}")
print(f"# of dt values that are exactly 0:      {(dt == 0).sum()}")
