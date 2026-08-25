"""Wait for a requested amount of timestamped audio in ROS simulation time."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from task_generator_msgs.msg import AudioFrame, EpisodeRecord


class CaptureWaiter(Node):
    def __init__(self, duration: float, wall_timeout: float, namespace: str, robot_name: str):
        super().__init__("arena_acoustics_capture_waiter")
        self.duration_ns = round(duration * 1_000_000_000)
        self.wall_deadline = time.monotonic() + wall_timeout
        self.clock_ns: int | None = None
        self.start_ns: int | None = None
        self.raw_end_ns: int | None = None
        self.rendered_end_ns: int | None = None
        self.raw_chunks = 0
        self.rendered_chunks = 0
        self.done = False
        self.error: str | None = None
        qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self._clock, qos)
        prefix = "/" + namespace.strip("/") if namespace.strip("/") else ""
        robot = robot_name.strip("/")
        if not robot:
            raise ValueError("robot_name must not be empty")
        self.raw_topic = f"{prefix}/{robot}/audio/raw_array"
        self.rendered_topic = f"{prefix}/{robot}/audio/headphones/stereo"
        self.episode_topic = f"{prefix}/state/episode"
        self.create_subscription(AudioFrame, self.raw_topic, self._raw, qos)
        self.create_subscription(AudioFrame, self.rendered_topic, self._rendered, qos)
        episode_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(EpisodeRecord, self.episode_topic, self._episode, episode_qos)
        self.create_timer(0.25, self._watchdog)

    @staticmethod
    def _stamp(msg: AudioFrame) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _clock(self, msg: Clock) -> None:
        self.clock_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)

    def _raw(self, msg: AudioFrame) -> None:
        self.raw_chunks += 1
        chunk_end_ns = self._chunk_end(msg)
        self.raw_end_ns = chunk_end_ns if self.raw_end_ns is None else max(self.raw_end_ns, chunk_end_ns)
        self._maybe_finish()

    def _rendered(self, msg: AudioFrame) -> None:
        self.rendered_chunks += 1
        chunk_end_ns = self._chunk_end(msg)
        self.rendered_end_ns = chunk_end_ns if self.rendered_end_ns is None else max(self.rendered_end_ns, chunk_end_ns)
        self._maybe_finish()

    def _episode(self, msg: EpisodeRecord) -> None:
        if int(msg.outcome_state) != int(EpisodeRecord.RUNNING):
            return
        self.start_ns = int(msg.start_time.sec) * 1_000_000_000 + int(msg.start_time.nanosec)
        self._maybe_finish()

    def _chunk_end(self, msg: AudioFrame) -> int:
        channels = int(msg.channel_count)
        frames = int(msg.frame_count)
        if (
            str(msg.encoding) != "32FC1"
            or not bool(msg.interleaved)
            or channels <= 0
            or frames <= 0
            or int(msg.sample_rate) <= 0
        ):
            self.error = "audio stream contains an invalid encoding, channel count, or sample rate"
            self.done = True
            return self._stamp(msg)
        if len(msg.data) != frames * channels:
            self.error = "AudioFrame.data length does not match frame_count * channel_count"
            self.done = True
            return self._stamp(msg)
        return self._stamp(msg) + round(frames * 1_000_000_000 / int(msg.sample_rate))

    def _maybe_finish(self) -> None:
        if self.coverage_complete:
            self.done = True

    @property
    def coverage_complete(self) -> bool:
        if self.start_ns is None or self.raw_end_ns is None or self.rendered_end_ns is None:
            return False
        requested_end_ns = self.start_ns + self.duration_ns
        return self.raw_end_ns >= requested_end_ns and self.rendered_end_ns >= requested_end_ns

    def _watchdog(self) -> None:
        if time.monotonic() >= self.wall_deadline:
            self.error = "wall-time watchdog expired before the requested simulation-time audio duration"
            self.done = True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--wall-timeout", type=float, required=True)
    parser.add_argument("--namespace", required=True, help="environment namespace, for example /arena/env_0")
    parser.add_argument("--robot-name", default="jackal")
    args = parser.parse_args(argv)
    rclpy.init()
    node = CaptureWaiter(args.duration, args.wall_timeout, args.namespace, args.robot_name)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
        result = {
            "valid": node.error is None and node.coverage_complete,
            "coverage_complete": node.coverage_complete,
            "start_timestamp_ns": node.start_ns,
            "end_clock_ns": node.clock_ns,
            "raw_end_timestamp_ns": node.raw_end_ns,
            "rendered_end_timestamp_ns": node.rendered_end_ns,
            "raw_chunks_seen": node.raw_chunks,
            "rendered_chunks_seen": node.rendered_chunks,
            "raw_topic": node.raw_topic,
            "rendered_topic": node.rendered_topic,
            "episode_topic": node.episode_topic,
            "error": node.error,
        }
        print(json.dumps(result))
        return 0 if result["valid"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
