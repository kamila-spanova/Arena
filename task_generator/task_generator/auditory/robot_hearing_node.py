from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from task_generator.auditory.qos_profiles import transient_event_qos
from task_generator_msgs.msg import HeardSoundEvent
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


@dataclass
class PendingHeardSound:
    release_time: Time
    msg: HeardSoundEvent


class RobotHearingNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("robot_hearing_node", **kwargs)

        self.declare_parameter("robot_name", "")
        self.declare_parameter("heard_sound_events_topic", "heard_sound_events")
        self.declare_parameter("output_topic", "heard_sound")
        self.declare_parameter("ignore_self", True)
        self.declare_parameter("min_snr_db", 3.0)
        self.declare_parameter("honor_propagation_delay", True)
        self.declare_parameter("marker_topic", "heard_sound_marker")
        self.declare_parameter("marker_lifetime_sec", 1.5)
        self.declare_parameter("marker_z_offset", 1.2)

        self._robot_name = str(self.get_parameter("robot_name").value).strip()
        if not self._robot_name:
            raise ValueError("robot_hearing_node requires parameter 'robot_name'")

        self._listener_id = f"robot:{self._robot_name}"
        self._pending: list[PendingHeardSound] = []

        self._pub = self.create_publisher(
            HeardSoundEvent,
            str(self.get_parameter("output_topic").value),
            transient_event_qos(),
        )

        self._marker_pub = self.create_publisher(
            Marker,
            str(self.get_parameter("marker_topic").value),
            10,
        )

        self.create_subscription(
            HeardSoundEvent,
            str(self.get_parameter("heard_sound_events_topic").value),
            self._cb_heard_sound,
            transient_event_qos(),
        )

        self.create_timer(0.01, self._publish_due_events)

        self.get_logger().info(
            f"listening for {self._listener_id!r} on "
            f"{self.get_parameter('heard_sound_events_topic').value!r}"
        )

    def _cb_heard_sound(self, msg: HeardSoundEvent) -> None:
        if msg.listener_id != self._listener_id:
            return

        if bool(self.get_parameter("ignore_self").value):
            if msg.source_agent_name == self._robot_name:
                return

        if not msg.audible:
            return

        snr_db = float(msg.received_volume_db - msg.hearing_threshold_db)
        if snr_db < float(self.get_parameter("min_snr_db").value):
            return

        if bool(self.get_parameter("honor_propagation_delay").value):
            release_time = Time.from_msg(msg.header.stamp) + rclpy.duration.Duration(
                seconds=float(msg.direct_delay_sec)
            )
        else:
            release_time = self.get_clock().now()

        self._pending.append(PendingHeardSound(release_time=release_time, msg=msg))

    def _publish_due_events(self) -> None:
        if not self._pending:
            return

        now = self.get_clock().now()
        due = [item for item in self._pending if item.release_time <= now]
        self._pending = [item for item in self._pending if item.release_time > now]

        for item in due:
            self._pub.publish(item.msg)
            self._publish_heard_marker(item.msg)
    
    def _publish_heard_marker(self, msg: HeardSoundEvent) -> None:
        if msg.sound_type != "greeting":
            return

        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = msg.header.frame_id or "map"
        marker.ns = f"{self._robot_name}_heard_sound"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = msg.listener_position
        marker.pose.position.z += float(self.get_parameter("marker_z_offset").value)
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.35
        marker.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)

        marker.text = "Heard: voice"

        lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)
        marker.lifetime.sec = int(lifetime_sec)
        marker.lifetime.nanosec = int((lifetime_sec % 1.0) * 1_000_000_000)

        self._marker_pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = RobotHearingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
