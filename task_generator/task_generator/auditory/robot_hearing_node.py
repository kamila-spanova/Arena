from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from task_generator.auditory.qos_profiles import acoustic_metadata_qos, transient_event_qos
from task_generator_msgs.msg import HeardSoundEvent, RobotFleet
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


@dataclass
class PendingHeardSound:
    release_time: Time
    robot_name: str
    msg: HeardSoundEvent


class RobotHearingNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("robot_hearing_node", **kwargs)

        # self.declare_parameter("robot_name", "")
        self.declare_parameter("heard_sound_events_topic", "heard_sound_events")
        # self.declare_parameter("output_topic", "heard_sound")
        self.declare_parameter("robot_fleet_topic", "state/robots")
        self.declare_parameter("heard_sound_topic_suffix", "heard_sound")
        self.declare_parameter("marker_topic_suffix", "heard_sound_marker")
        self.declare_parameter("ignore_self", True)
        self.declare_parameter("min_snr_db", -15.0)
        self.declare_parameter("honor_propagation_delay", True)
        # self.declare_parameter("marker_topic", "heard_sound_marker")
        self.declare_parameter("marker_lifetime_sec", 1.5)
        self.declare_parameter("marker_z_offset", 1.2)

        # self._robot_name = str(self.get_parameter("robot_name").value).strip()
        self._robot_names: set[str] = set()
        self._robot_frames: dict[str, str] = {}
        self._heard_pubs = {}
        self._marker_pubs = {}
        self._pending: list[PendingHeardSound] = []
        self.create_subscription(
            RobotFleet,
            str(self.get_parameter("robot_fleet_topic").value),
            self._cb_robot_fleet,
            acoustic_metadata_qos(),
        )

        self.create_subscription(
            HeardSoundEvent,
            str(self.get_parameter("heard_sound_events_topic").value),
            self._cb_heard_sound,
            transient_event_qos(),
        )

        self.create_timer(0.01, self._publish_due_events)

        self.get_logger().info(
            "robot hearing node listening on "
            f"{self.get_parameter('heard_sound_events_topic').value!r}; "
            "waiting for robot fleet"
        )

    def _cb_heard_sound(self, msg: HeardSoundEvent) -> None:
        if not msg.listener_id.startswith("robot:"):
            return

        robot_name = msg.listener_id.removeprefix("robot:")
        if robot_name not in self._robot_names:
            return

        if bool(self.get_parameter("ignore_self").value):
            if msg.source_agent_name == robot_name:
                return

        # if not msg.audible:
        #     return

        # snr_db = float(msg.received_volume_db - msg.hearing_threshold_db)
        # if snr_db < float(self.get_parameter("min_snr_db").value):
        #     return

        if bool(self.get_parameter("honor_propagation_delay").value):
            release_time = Time.from_msg(msg.header.stamp) + rclpy.duration.Duration(
                seconds=float(msg.direct_delay_sec)
            )
        else:
            release_time = self.get_clock().now()

        self._pending.append(
            PendingHeardSound(
                release_time=release_time,
                robot_name=robot_name,
                msg=msg,
            )
        )

    def _cb_robot_fleet(self, msg: RobotFleet) -> None:
        heard_suffix = str(self.get_parameter("heard_sound_topic_suffix").value).strip("/")
        marker_suffix = str(self.get_parameter("marker_topic_suffix").value).strip("/")

        for robot in msg.robots:
            robot_name = str(robot.name).strip()
            if not robot_name:
                continue

            self._robot_names.add(robot_name)
            self._robot_frames[robot_name] = str(robot.frame).strip()

            if robot_name not in self._heard_pubs:
                self._heard_pubs[robot_name] = self.create_publisher(
                    HeardSoundEvent,
                    f"{robot_name}/{heard_suffix}",
                    transient_event_qos(),
                )

            if robot_name not in self._marker_pubs:
                self._marker_pubs[robot_name] = self.create_publisher(
                    Marker,
                    f"{robot_name}/{marker_suffix}",
                    10,
                )

            self.get_logger().info(f"registered robot hearing outputs for {robot_name!r}")

    def _publish_due_events(self) -> None:
        if not self._pending:
            return

        now = self.get_clock().now()
        due = [item for item in self._pending if item.release_time <= now]
        self._pending = [item for item in self._pending if item.release_time > now]

        # for item in due:
        #     self._pub.publish(item.msg)
        #     self._publish_heard_marker(item.msg)
        for item in due:
            pub = self._heard_pubs.get(item.robot_name)
            if pub is not None:
                pub.publish(item.msg)

            self._publish_heard_marker(item.robot_name, item.msg)

    def _publish_heard_marker(self, robot_name: str, msg: HeardSoundEvent) -> None:
        if msg.sound_type != "greeting":
            return

        pub = self._marker_pubs.get(robot_name)
        if pub is None:
            self.get_logger().warning(f"no marker publisher for robot {robot_name!r}")
            return

        robot_frame = self._robot_frames.get(robot_name, "")
        if not robot_frame:
            self.get_logger().warning(f"no robot frame for {robot_name!r}")
            return

        marker = Marker()
        marker.header.frame_id = robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{robot_name}_heard_sound"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Robot-local position: directly above the robot base frame.
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 3.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 1.5
        marker.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=1.0)
        marker.text = "HEARD GREETING"

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.frame_locked = True

        pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = RobotHearingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
