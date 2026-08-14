#!/usr/bin/env python3
"""Collect stationary IMU samples and print odometry calibration values."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu


STANDARD_GRAVITY_M_S2 = 9.80665


def format_vector(values):
    return '[' + ', '.join(f'{value:.8f}' for value in values) + ']'


class RunningStats:
    """Numerically stable running mean and population standard deviation."""

    def __init__(self, width):
        self.count = 0
        self.mean = [0.0] * width
        self.m2 = [0.0] * width

    def add(self, values):
        self.count += 1
        for index, value in enumerate(values):
            delta = value - self.mean[index]
            self.mean[index] += delta / self.count
            self.m2[index] += delta * (value - self.mean[index])

    def standard_deviation(self):
        if self.count == 0:
            return [math.nan] * len(self.mean)
        return [math.sqrt(value / self.count) for value in self.m2]


class ImuCalibrationGetter(Node):
    def __init__(self, topic, sample_count):
        super().__init__('imu_calibration_getter')
        self.topic = topic
        self.sample_count = sample_count
        self.stats = RunningStats(6)
        self.invalid_samples = 0
        self.finished = False
        self.first_sample_time = None
        self.last_sample_time = None
        self.subscription = self.create_subscription(
            Imu, topic, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f'Keep the robot stationary: collecting {sample_count} samples from {topic}')

    def on_imu(self, message):
        if self.finished:
            return

        values = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.invalid_samples += 1
            return

        now = time.monotonic()
        if self.first_sample_time is None:
            self.first_sample_time = now
        self.last_sample_time = now
        self.stats.add(values)

        if self.stats.count % 100 == 0 and self.stats.count < self.sample_count:
            self.get_logger().info(
                f'Collected {self.stats.count}/{self.sample_count} valid samples')

        if self.stats.count >= self.sample_count:
            self.finished = True

    def result_text(self):
        means = self.stats.mean
        standard_deviations = self.stats.standard_deviation()
        acceleration_mean = means[:3]
        acceleration_stddev = standard_deviations[:3]
        gyro_mean = means[3:]
        gyro_stddev = standard_deviations[3:]
        gravity_magnitude = math.sqrt(sum(value * value for value in acceleration_mean))
        acceleration_scale = (
            STANDARD_GRAVITY_M_S2 / gravity_magnitude
            if gravity_magnitude > 1.0e-9 else math.nan
        )
        elapsed = max(0.0, (self.last_sample_time or 0.0) - (self.first_sample_time or 0.0))
        sample_rate = (self.stats.count - 1) / elapsed if elapsed > 0.0 else math.nan

        return '\n'.join((
            '',
            'IMU CALIBRATION RESULT',
            f'samples: {self.stats.count}',
            f'invalid_samples_ignored: {self.invalid_samples}',
            f'measured_rate_hz: {sample_rate:.3f}',
            f'acceleration_mean_m_s2: {format_vector(acceleration_mean)}',
            f'acceleration_stddev_m_s2: {format_vector(acceleration_stddev)}',
            f'measured_gravity_magnitude_m_s2: {gravity_magnitude:.8f}',
            f'odom_acceleration_scale_factor: {acceleration_scale:.8f}',
            f'scaled_acceleration_mean_m_s2: '
            f'{format_vector([value * acceleration_scale for value in acceleration_mean])}',
            f'gyro_bias_rad_s: {format_vector(gyro_mean)}',
            f'gyro_stddev_rad_s: {format_vector(gyro_stddev)}',
        ))


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description='Collect stationary raw IMU samples and print calibration statistics.')
    parser.add_argument(
        '--topic', default='/imu/data_raw', help='sensor_msgs/msg/Imu topic to sample')
    parser.add_argument(
        '--samples', type=int, default=1000, help='number of valid samples (default: 1000)')
    parser.add_argument(
        '--timeout', type=float, default=30.0,
        help='seconds to wait for all samples (default: 30)')
    arguments = parser.parse_args(remove_ros_args(args=argv)[1:])
    if arguments.samples <= 0:
        parser.error('--samples must be greater than zero')
    if arguments.timeout <= 0.0:
        parser.error('--timeout must be greater than zero')
    return arguments


def main(argv=None):
    argv = sys.argv if argv is None else argv
    arguments = parse_arguments(argv)
    rclpy.init(args=argv)
    node = ImuCalibrationGetter(arguments.topic, arguments.samples)
    deadline = time.monotonic() + arguments.timeout
    exit_code = 0
    try:
        while rclpy.ok() and not node.finished and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.finished:
            print(node.result_text(), flush=True)
        else:
            publisher_count = node.count_publishers(arguments.topic)
            if node.invalid_samples:
                reason = f'all {node.invalid_samples} received messages contained non-finite data'
            elif publisher_count == 0:
                reason = (
                    'no publisher was discovered; start mpu6050_node and verify that this shell '
                    'uses the same ROS_DOMAIN_ID as the launch process'
                )
            else:
                reason = (
                    f'{publisher_count} publisher(s) were discovered but no compatible Imu '
                    'messages arrived; inspect the topic type and QoS'
                )
            node.get_logger().error(
                f'Timed out after {arguments.timeout:.1f}s with '
                f'{node.stats.count}/{arguments.samples} valid samples from {arguments.topic}: '
                f'{reason}')
            exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warning(
            f'Cancelled with {node.stats.count}/{arguments.samples} samples collected')
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
