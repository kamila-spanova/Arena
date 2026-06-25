from __future__ import annotations
from pathlib import Path
from task_generator.auditory.asset_lib import AcousticAssetCatalog
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from task_generator.auditory.audio_mixer import AudioMixer
from task_generator_msgs.msg import EpisodeRecord, SoundEvent
from task_generator.auditory.qos_profiles import transient_event_qos

# FOOTSTEP_VARIANT_TAGS = frozenset({"hard_floor", "soft_floor"})

class HumanSoundPlaybackNode(Node):
    def __init__(self) -> None:
        super().__init__("human_sound_playback")

        share_dir = Path(get_package_share_directory("task_generator"))
        self.declare_parameter("sound_events_topic", "human_sound_events")
        self.declare_parameter("sound_dir", str(share_dir / "sounds"))
        self.declare_parameter("asset_catalog",str(share_dir / "config" / "auditory" / "acoustic_assets.yaml"))
        self.declare_parameter("output_sample_rate", 44100)
        self.declare_parameter("output_channels", 2)
        self.declare_parameter("block_size", 1024)
        self.declare_parameter("audio_device", "")
        self.declare_parameter("master_gain_db", 0.0)
        self.declare_parameter("episode_topic", "state/episode")

        sample_rate = int(self.get_parameter("output_sample_rate").value)
        channels = int(self.get_parameter("output_channels").value)
        device = str(self.get_parameter("audio_device").value).strip() or None

        self._catalog = AcousticAssetCatalog(
            config_path=Path(str(self.get_parameter("asset_catalog").value)),
            sound_dir=Path(str(self.get_parameter("sound_dir").value)),
            output_sample_rate=sample_rate,
            output_channels=channels,
        )

        self._mixer = AudioMixer(
            sample_rate=sample_rate,
            channels=channels,
            block_size=int(self.get_parameter("block_size").value),
            device=device,
            master_gain_db=float(self.get_parameter("master_gain_db").value),
        )

        topic = str(self.get_parameter("sound_events_topic").value)
        self.create_subscription(SoundEvent, topic, self._cb_sound_event, transient_event_qos())
        self.get_logger().info(f"playing human sound events from {topic}")

        self._episode_seed = 0
        self._episode_id = -1
        self._occurrences: dict[tuple[int, str], int] = defaultdict(int)

        episode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            EpisodeRecord,
            str(self.get_parameter("episode_topic").value),
            self._cb_episode,
            episode_qos,
        )

        self.get_logger().info(f"playing human sound events from {topic}")

    def _cb_episode(self, msg: EpisodeRecord) -> None:
        if msg.episode_id == self._episode_id:
            return

        self._episode_id = int(msg.episode_id)
        self._episode_seed = int(msg.seed)
        self._occurrences.clear()
        self._mixer.stop_all()

    def _cb_sound_event(self, msg: SoundEvent) -> None:
        asset_id = msg.asset_id.strip() or msg.sound_type.strip()
        key = (int(msg.source_agent_id), asset_id)

        occurrence = self._occurrences[key]
        self._occurrences[key] += 1

        # required_tags = frozenset(
        #     FOOTSTEP_VARIANT_TAGS.intersection(str(tag) for tag in msg.semantic_tags)
        # )

        # selected = self._catalog.select(
        #     asset_id,
        #     episode_seed=self._episode_seed,
        #     agent_id=int(msg.source_agent_id),
        #     occurrence=occurrence,
        #     required_tags=required_tags,
        # )

        selected = self._catalog.select(
            asset_id,
            episode_seed=self._episode_seed,
            agent_id=int(msg.source_agent_id),
            occurrence=occurrence,
        )

        if selected is None:
            self.get_logger().warning(f"no acoustic asset for asset_id={asset_id!r}")
            return

        asset, sample = selected
        # self._mixer.play(sample,loop=bool(msg.loop or asset.loop))
        self._mixer.play( sample,loop=bool(msg.loop or asset.loop),gain_db=asset.playback_gain_db )
        self.get_logger().warning(
            f"playing {asset_id} / {sample.sample_id}: "
            f"{sample.duration_sec:.3f}s, gain={asset.playback_gain_db:.1f} dB"
        )
        # self.get_logger().warning(
        #     f"playing {sample.sample_id}: "
        #     f"{sample.duration_sec:.3f}s, "
        #     f"{sample.sample_rate} Hz, {sample.channels} ch"
        # )
    
    def destroy_node(self) -> bool:
        self._mixer.close()
        return super().destroy_node()
        

def main() -> None:
    rclpy.init()
    node = HumanSoundPlaybackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()