from __future__ import annotations

import json
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory



class HumanSoundPlaybackNode(Node):
    def __init__(self) -> None:
        super().__init__("human_sound_playback")

        self.declare_parameter(
            "sound_events_topic",
            "human_sound_events",
        )
        default_sound_dir = Path(get_package_share_directory("task_generator")) / "sounds"
        self.declare_parameter("sound_dir", str(default_sound_dir))
        self.declare_parameter(
            "player_command",
            "aplay",
        )
        self.declare_parameter(
            "sound_map",
            '{"footstep": "footstep.wav", "greeting": "greeting.wav"}',
        )

        self._sound_dir = Path(str(self.get_parameter("sound_dir").value))
        self._player_command = str(self.get_parameter("player_command").value)
        self._sound_map = json.loads(str(self.get_parameter("sound_map").value))

        topic = str(self.get_parameter("sound_events_topic").value)
        self.create_subscription(String, topic, self._cb_sound_event, 10)

        self.get_logger().info(f"playing human sound events from {topic}")

    def _cb_sound_event(self, msg: String) -> None:
        event = msg.data.strip()
        filename = self._sound_map.get(event)
        if filename is None:
            self.get_logger().warn(f"no wav mapping for sound event {event!r}")
            return

        wav_path = self._sound_dir / filename
        if not wav_path.is_file():
            self.get_logger().warn(f"missing wav file: {wav_path}")
            return

        subprocess.Popen(
            [self._player_command, str(wav_path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )


def main() -> None:
    rclpy.init()
    node = HumanSoundPlaybackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()