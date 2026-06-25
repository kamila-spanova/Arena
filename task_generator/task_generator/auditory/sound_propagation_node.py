from __future__ import annotations

import math
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from arena_people_msgs.msg import Pedestrians
from arena_simulation_setup.tree.World import WorldIdentifier
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String
from task_generator.auditory.acoustic_scene import AcousticScene
from task_generator.auditory.material_catalog import AcousticMaterialCatalog
from task_generator.auditory.propagation import Level3Propagation
from task_generator_msgs.msg import AcousticPath, HeardSoundEvent, RobotFleet, SoundEvent
from task_generator.auditory.qos_profiles import transient_event_qos



class SoundPropagationNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("sound_propagation_node", **kwargs)

        self.declare_parameter("sound_events_topic", "human_sound_events")
        self.declare_parameter("heard_sound_events_topic", "heard_sound_events")
        self.declare_parameter("arena_peds_topic", "arena_peds")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("robot_fleet_topic", "state/robots")
        self.declare_parameter("default_hearing_threshold_db", 20.0)
        self.declare_parameter("minimum_propagation_distance_m", 1.0)
        self.declare_parameter("self_hearing_distance_m", 0.3)
        self.declare_parameter("occlusion_penalty_db", 20.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("publish_inaudible", False)
        self.declare_parameter("robots_hear_self", True)
        self.declare_parameter("world_topic", "state/world")
        self._scene: AcousticScene | None = None
        self._world_name = ""
        self._pending_world_name = ""
        self._world_load_future: Future | None = None
        self._world_loader = ThreadPoolExecutor(max_workers=1)
        self.create_subscription(String, str(self.get_parameter("world_topic").value), self._cb_world, 1)
        self._world_load_timer = self.create_timer(0.1, self._poll_world_load)
        self._peds: dict[int, object] = {}
        self._robots: dict[str, Point] = {}
        self._map: OccupancyGrid | None = None
        self._odom_subs = []
        sound_events_topic = str(self.get_parameter("sound_events_topic").value)
        heard_sound_events_topic = str(self.get_parameter("heard_sound_events_topic").value)
        peds_topic = str(self.get_parameter("arena_peds_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        robot_fleet_topic = str(self.get_parameter("robot_fleet_topic").value)
        self._heard_pub = self.create_publisher( HeardSoundEvent,  heard_sound_events_topic, transient_event_qos())
        self.create_subscription(SoundEvent, sound_events_topic, self._cb_sound_event, transient_event_qos())
        self.create_subscription(Pedestrians, peds_topic, self._cb_peds, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._cb_map, 1)
        self.create_subscription(RobotFleet, robot_fleet_topic, self._cb_robot_fleet, 1)
        share = Path(get_package_share_directory("task_generator"))
        materials = AcousticMaterialCatalog(share / "config" / "auditory" / "acoustic_materials.yaml")
        self._propagation = Level3Propagation(materials)

    def _cb_peds(self, msg: Pedestrians) -> None:
        self._peds = {int(p.id): p for p in msg.pedestrians}
    
    def _cb_world(self, msg: String) -> None:
        world_name = msg.data.strip()
        if not world_name or world_name == self._world_name:
            return

        self._pending_world_name = world_name

        if self._world_load_future is not None and not self._world_load_future.done():
            return

        self._start_world_load(world_name)
    
    def _start_world_load(self, world_name: str) -> None:
        self.get_logger().info(f"loading acoustic scene for world {world_name!r}")

        self._world_load_future = self._world_loader.submit(
            self._load_acoustic_scene,
            world_name,
        )
    
    @staticmethod
    def _load_acoustic_scene(world_name: str) -> tuple[str, AcousticScene]:
        world_view = WorldIdentifier(world_name).resolve_sync()
        world_description = world_view.load()
        return world_name, AcousticScene.from_world(world_description)
    
    def _poll_world_load(self) -> None:
        if self._world_load_future is None:
            return

        if not self._world_load_future.done():
            return

        future = self._world_load_future
        self._world_load_future = None

        try:
            world_name, scene = future.result()
        except Exception as exc:
            self.get_logger().error(f"failed to load acoustic scene: {exc!r}")
            return

        self._scene = scene
        self._world_name = world_name
        self.get_logger().info(f"loaded acoustic scene for world {world_name!r}")

        if self._pending_world_name and self._pending_world_name != self._world_name:
            self._start_world_load(self._pending_world_name)
    
    def destroy_node(self) -> bool:
        self._world_loader.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _cb_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _cb_robot_fleet(self, msg: RobotFleet) -> None:
        for robot in msg.robots:
            topic = f"{robot.ns}/odom"
            sub = self.create_subscription(Odometry, topic, lambda odom, name=robot.name: self._cb_robot_odom(name, odom), 10)
            self._odom_subs.append(sub)

    def _cb_robot_odom(self, robot_name: str, msg: Odometry) -> None:
        self._robots[f"robot:{robot_name}"] = msg.pose.pose.position


    def _cb_sound_event(self, event: SoundEvent) -> None:
        if not event.sound_type.strip():
            return

        listeners: dict[str, Point] = {}

        for agent_id, ped in self._peds.items():
            if agent_id == event.source_agent_id:
                continue
            listeners[f"agent:{agent_id}"] = ped.pose.position

        listeners.update(self._robots)

        if not bool(self.get_parameter("robots_hear_self").value):
            listeners.pop(f"robot:{event.source_agent_name}", None)

        for listener_id, listener_pos in listeners.items():
            heard = self._calculate_heard_event(event, listener_id, listener_pos)
            if heard.audible or bool(self.get_parameter("publish_inaudible").value):
                self._heard_pub.publish(heard)
    
    def _effective_sound_distance(self, geometric_distance: float, event: SoundEvent, listener_id: str,) -> float:
        if listener_id == f"robot:{event.source_agent_name}":
            return max(float(self.get_parameter("self_hearing_distance_m").value),1e-3)

        return max(geometric_distance, float(self.get_parameter("minimum_propagation_distance_m").value),1e-3)


    def _calculate_legacy_event(self, event: SoundEvent, listener_id: str, listener_pos: Point) -> HeardSoundEvent:
        dx = event.source_position.x - listener_pos.x
        dy = event.source_position.y - listener_pos.y
        # distance = max(math.hypot(dx, dy), 1.0)
        # distance_loss = 20.0 * math.log10(distance)
        geometric_distance = math.hypot(dx, dy)
        effective_distance = self._effective_sound_distance(
            geometric_distance,
            event,
            listener_id,
        )
        distance_loss = 20.0 * math.log10(effective_distance)

        occluded = self._is_occluded(event.source_position, listener_pos)
        occlusion_penalty = (float(self.get_parameter("occlusion_penalty_db").value) if occluded else 0.0)
        received = event.source_volume_db - distance_loss - occlusion_penalty
        threshold = float(self.get_parameter("default_hearing_threshold_db").value)

        msg = HeardSoundEvent()
        msg.header = event.header
        msg.event_id = event.event_id
        msg.listener_id = listener_id
        msg.source_agent_id = event.source_agent_id
        msg.source_agent_name = event.source_agent_name
        msg.sound_type = event.sound_type
        msg.label = event.label
        msg.asset_id = event.asset_id
        msg.source_position = event.source_position
        msg.listener_position = listener_pos
        # msg.distance = float(distance)
        msg.distance = float(geometric_distance)
        msg.source_volume_db = event.source_volume_db
        msg.received_volume_db = float(received)
        msg.hearing_threshold_db = float(threshold)
        msg.audible = received >= threshold
        msg.occluded = occluded
        msg.bearing_rad = float(math.atan2(dy, dx))
        # msg.direct_delay_sec = float(distance / 343.0)
        msg.direct_delay_sec = float(effective_distance / 343.0)
        msg.propagation_level = 0
        msg.reverb_rt60_sec = 0.0
        msg.reverb_gain_db = 0.0
        msg.source_zone = ""
        msg.listener_zone = ""
        return msg


    def _calculate_heard_event(self, event: SoundEvent, listener_id: str,listener_pos: Point) -> HeardSoundEvent:
        # if self._scene is None:
        #     return self._calculate_legacy_event(event, listener_id, listener_pos)
        if self._scene is None or listener_id == f"robot:{event.source_agent_name}":
            return self._calculate_legacy_event(event, listener_id, listener_pos)

        result = self._propagation.calculate(self._scene, event.source_position, listener_pos, event.source_volume_db)
        dx = event.source_position.x - listener_pos.x
        dy = event.source_position.y - listener_pos.y
        distance = max(math.hypot(dx, dy), 1.0)
        threshold = float(self.get_parameter("default_hearing_threshold_db").value)

        msg = HeardSoundEvent()
        msg.header = event.header
        msg.event_id = event.event_id
        msg.listener_id = listener_id
        msg.source_agent_id = event.source_agent_id
        msg.source_agent_name = event.source_agent_name
        msg.sound_type = event.sound_type
        msg.label = event.label
        msg.asset_id = event.asset_id
        msg.source_position = event.source_position
        msg.listener_position = listener_pos
        msg.distance = float(distance)
        msg.bearing_rad = float(math.atan2(dy, dx))
        msg.source_volume_db = event.source_volume_db
        msg.received_volume_db = result.received_volume_db
        msg.hearing_threshold_db = threshold
        msg.audible = result.received_volume_db >= threshold
        msg.occluded = result.occluded
        msg.propagation_level = 3
        msg.direct_delay_sec = result.direct_delay_sec
        msg.reverb_rt60_sec = result.rt60_sec
        msg.reverb_gain_db = result.reverb_gain_db
        msg.source_zone = result.source_zone
        msg.listener_zone = result.listener_zone

        for path in result.paths:
            path_msg = AcousticPath()
            path_msg.delay.sec = int(path.delay_sec)
            path_msg.delay.nanosec = int((path.delay_sec % 1.0) * 1_000_000_000)
            path_msg.gain_db = path.gain_db
            path_msg.bearing_rad = path.bearing_rad
            path_msg.interaction_type = path.interaction_type
            path_msg.material_id = path.material_id

            if path.reflection_point is not None:
                path_msg.reflection_point.x = path.reflection_point[0]
                path_msg.reflection_point.y = path.reflection_point[1]

            msg.early_paths.append(path_msg)

        return msg

    def _is_occluded(self, source: Point, listener: Point) -> bool:
        if self._map is None:
            return False

        a = self._world_to_grid(source)
        b = self._world_to_grid(listener)
        if a is None or b is None:
            return False

        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        for x, y in self._bresenham(a[0], a[1], b[0], b[1]):
            idx = y * self._map.info.width + x
            if 0 <= idx < len(self._map.data) and self._map.data[idx] >= occupied_threshold:
                return True

        return False

    def _world_to_grid(self, point: Point) -> tuple[int, int] | None:
        assert self._map is not None
        origin = self._map.info.origin.position
        resolution = self._map.info.resolution

        x = int((point.x - origin.x) / resolution)
        y = int((point.y - origin.y) / resolution)

        if x < 0 or y < 0 or x >= self._map.info.width or y >= self._map.info.height:
            return None
        return x, y

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy


def main() -> None:
    rclpy.init()
    node = SoundPropagationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()