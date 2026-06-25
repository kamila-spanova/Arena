from __future__ import annotations

import hashlib
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from task_generator.auditory.qos_profiles import acoustic_metadata_qos, transient_event_qos
from task_generator_msgs.msg import RobotFleet, SoundEvent
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class RobotSoundSource:
    name: str
    namespace: str
    position: Point | None = None


class RobotSoundNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("robot_sound_node", **kwargs)

        self.declare_parameter("robot_fleet_topic", "state/robots")
        self.declare_parameter("sound_events_topic", "human_sound_events")
        self.declare_parameter("sound_type", "motor")
        self.declare_parameter("odom_topic_template", "{namespace}/{name}_velocity_controller/odom")
        self.declare_parameter("asset_id", "motor")
        self.declare_parameter("source_volume_db", 55.0)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("only_when_moving", False)
        self.declare_parameter("min_speed_mps", 0.02)

        self._robots: dict[str, RobotSoundSource] = {}
        self._odom_qos = QoSProfile( history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT )
        self._odom_subs = []
        self._last_speed: dict[str, float] = {}
        self._event_counter = 0

        self._sound_pub = self.create_publisher(
            SoundEvent,
            str(self.get_parameter("sound_events_topic").value),
            transient_event_qos(),
        )

        self.create_subscription(
            RobotFleet,
            str(self.get_parameter("robot_fleet_topic").value),
            self._cb_robot_fleet,
            acoustic_metadata_qos(),
        )

        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_robot_sounds,
        )

    def _cb_robot_fleet(self, msg: RobotFleet) -> None:
        for robot in msg.robots:
            name = str(robot.name)
            if name in self._robots:
                continue

            namespace = str(robot.ns).rstrip("/")
            self._robots[name] = RobotSoundSource(name=name, namespace=namespace)

            odom_topic = str(self.get_parameter("odom_topic_template").value).format(namespace=namespace, name=name)
            self.get_logger().warning(f"subscribing to robot odom: {odom_topic}")
            sub = self.create_subscription(
                Odometry,
                odom_topic,
                lambda odom, robot_name=name: self._cb_odom(robot_name, odom),
                self._odom_qos,
            )
            self._odom_subs.append(sub)

            self.get_logger().warning(f"subscribing to robot odom: {odom_topic}")

    def _cb_odom(self, robot_name: str, msg: Odometry) -> None:
        source = self._robots.get(robot_name)
        if source is None:
            return

        source.position = msg.pose.pose.position
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self._last_speed[robot_name] = (vx * vx + vy * vy) ** 0.5

    def _publish_robot_sounds(self) -> None:
        only_when_moving = bool(self.get_parameter("only_when_moving").value)
        min_speed = float(self.get_parameter("min_speed_mps").value)

        for robot_name, source in self._robots.items():
            if source.position is None:
                continue

            if only_when_moving and self._last_speed.get(robot_name, 0.0) < min_speed:
                continue

            self._sound_pub.publish(self._make_sound_event(robot_name, source.position))

    def _make_sound_event(self, robot_name: str, position: Point) -> SoundEvent:
        stamp = self.get_clock().now().to_msg()
        sound_type = str(self.get_parameter("sound_type").value)
        asset_id = str(self.get_parameter("asset_id").value)
        period = float(self.get_parameter("publish_period_sec").value)

        msg = SoundEvent()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.event_id = f"robot:{robot_name}:{stamp.sec}:{stamp.nanosec}:{self._event_counter}"
        self._event_counter += 1

        msg.source_agent_id = self._robot_numeric_id(robot_name)
        msg.source_agent_name = robot_name
        msg.sound_type = sound_type
        msg.label = sound_type
        msg.asset_id = asset_id
        msg.source_position = position
        msg.source_yaw = 0.0
        msg.source_volume_db = float(self.get_parameter("source_volume_db").value)
        msg.semantic_tags = ["robot", "motor", "mechanical"]
        msg.duration.sec = int(period)
        msg.duration.nanosec = int((period % 1.0) * 1_000_000_000)
        msg.loop = False
        return msg

    @staticmethod
    def _robot_numeric_id(robot_name: str) -> int:
        digest = hashlib.blake2b(robot_name.encode(), digest_size=4).digest()
        return -int.from_bytes(digest, "big")


def main() -> None:
    rclpy.init()
    node = RobotSoundNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()