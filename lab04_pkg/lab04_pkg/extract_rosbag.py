import argparse
import csv
import math
from pathlib import Path
from contextlib import ExitStack

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

TOPICS = [
    '/odom',
    '/ekf',
    '/ekf_ext',
    '/ground_truth'
]

FIELDS = [
    'time', 'time_ns', 'bag_time_ns', 'bag_elapsed',
    'frame_id', 'child_frame_id',
    'x', 'y', 'z',
    'qx', 'qy', 'qz', 'qw', 'yaw',
    'vx', 'vy', 'vz',
    'wx', 'wy', 'wz'
]

FIELDS += [
    f'{kind}_cov_{i}{j}'
    for kind in ('pose', 'twist')
    for i in range(6)
    for j in range(6)
]


def main():

    parser = argparse.ArgumentParser(
        description='Export ROS 2 odometry to CSV'
    )

    parser.add_argument(
        'bag',
        type=Path,
        help='Directory containing metadata.yaml'
    )

    parser.add_argument(
        '--output',
        type=Path,
        default=Path('results')
    )

    args = parser.parse_args()
    bag = args.bag.resolve()

    if not (bag / 'metadata.yaml').is_file():
        parser.error(f'metadata.yaml not found in {bag}')

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag),
            storage_id='sqlite3'
        ),
        rosbag2_py.ConverterOptions('', '')
    )

    available = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    topics = [
        t for t in TOPICS
        if available.get(t) == 'nav_msgs/msg/Odometry'
    ]

    for t in TOPICS:
        if t not in topics:
            print(f'Skipping {t}: missing or not Odometry')

    if not topics:
        raise RuntimeError('No requested Odometry topics found')

    output = args.output / bag.name
    output.mkdir(parents=True, exist_ok=True)

    counts = dict.fromkeys(topics, 0)
    start = None

    with ExitStack() as stack:

        writers = {}

        for topic in topics:

            file = stack.enter_context(
                (output / (topic[1:] + '.csv')).open(
                    'w', newline=''
                )
            )

            writers[topic] = csv.writer(file)
            writers[topic].writerow(FIELDS)

        while reader.has_next():

            topic, data, timestamp = reader.read_next()

            if start is None:
                start = timestamp

            if topic not in writers:
                continue

            msg = deserialize_message(data, Odometry)

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            v = msg.twist.twist.linear
            w = msg.twist.twist.angular

            stamp = msg.header.stamp
            ns = stamp.sec * 10**9 + stamp.nanosec

            yaw = math.atan2(
                2 * (q.w*q.z + q.x*q.y),
                1 - 2 * (q.y*q.y + q.z*q.z)
            )

            writers[topic].writerow([
                ns / 1e9,
                ns,
                timestamp,
                (timestamp - start) / 1e9,

                msg.header.frame_id,
                msg.child_frame_id,

                p.x, p.y, p.z,
                q.x, q.y, q.z, q.w,
                yaw,

                v.x, v.y, v.z,
                w.x, w.y, w.z,

                *msg.pose.covariance,
                *msg.twist.covariance
            ])

            counts[topic] += 1

    for topic, count in counts.items():
        print(f'{topic}: {count} messages')

    print(f'CSV files saved in {output.resolve()}')


if __name__ == '__main__':
    main()