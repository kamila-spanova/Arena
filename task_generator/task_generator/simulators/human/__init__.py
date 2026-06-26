from __future__ import annotations

import abc
import asyncio
import json
import math
import itertools
from turtle import stamp
import typing
from collections.abc import Mapping, Sequence

import attrs
import rclpy.publisher
import rclpy.qos
from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_rclpy_mixins.registry import AsyncFactoryRegistry as Registry
from arena_rclpy_mixins.shared import Namespace
from arena_runtime._node import NodeInterface
from arena_runtime.sim import BaseSim
from arena_simulation_setup.tree.assets.Pedestrian import PedestrianIdentifier
from arena_simulation_setup.utils.models import ModelType
from task_generator.constants import Constants
from task_generator.manager.realizer import Realizer
from task_generator.shared import Door, DynamicObstacle, Obstacle, Region, Robot, Wall
from task_generator.simulators.human.utils import (
    KnownObstacle,
    KnownObstacles,
    ObstacleLayer,
)
from visualization_msgs.msg import Marker, MarkerArray
from task_generator.simulators.human.auditory_events import AuditoryEventDetector
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from task_generator_msgs.msg import SoundEvent
from task_generator.auditory.qos_profiles import transient_event_qos
from arena_simulation_setup.tree.World import WorldIdentifier
from task_generator.auditory.acoustic_scene import AcousticScene

class BaseHumanSimulator(NodeInterface, abc.ABC):
    _arena_peds_publisher: rclpy.publisher.Publisher
    _marker_publisher: rclpy.publisher.Publisher
    _static_marker_publisher: rclpy.publisher.Publisher
    _known_obstacles: KnownObstacles
    _known_walls: KnownObstacles[Wall]
    _known_doors: KnownObstacles[Door]

    @classmethod
    def _register_task_modes(cls) -> None:
        """Register simulator-specific obstacle task modes.

        Called during __init__. Subclasses override this to register
        task modes specific to their simulator (e.g. TM_Prompt).
        """

    def __init__(self, *args: object, namespace: Namespace, simulator: BaseSim, realizer: Realizer, **kwargs: object) -> None:
        """
        Initialize human simulator.

        Args:
            namespace: global namespace
            simulator: Simulator instance
            realizer: per-env Realizer (used to stamp sim_path on runtime-spawned obstacles)
        """
        super().__init__(*args, **kwargs)
        self._register_task_modes()
        self._simulator = simulator
        self._namespace = namespace
        self._realizer = realizer

        self._known_obstacles = KnownObstacles[Obstacle]()
        self._known_walls = KnownObstacles[Wall]()
        self._known_doors = KnownObstacles[Door]()
        self._wall_counter = itertools.count()
        self._known_regions: dict[str, Region] = {}
        self._sound_event_counter = itertools.count()
        self._acoustic_scene: AcousticScene | None = None

        self._arena_peds_publisher = self.node.create_publisher(Pedestrians, self._namespace("arena_peds"), 10)
        self._marker_publisher = self.node.create_publisher(
            MarkerArray,
            self._namespace("pedestrian_markers", "extra"),
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self._static_marker_publisher = self.node.create_publisher(
            MarkerArray,
            self._namespace("pedestrian_markers", "static"),
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self._sound_events_publisher = self.node.create_publisher(
            SoundEvent,
            self._namespace("human_sound_events"),
            transient_event_qos(),
        )
        self._sound_marker_publisher = self.node.create_publisher(
            MarkerArray,
            self._namespace("human_sound_markers"),
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self._sound_marker_counter = itertools.count()
        self._auditory_events = AuditoryEventDetector(
            self.publish_sound_event,
            walking_speed_threshold=0.05,
            footstep_interval_sec=0.45,
            greeting_distance_m=1.5,
            greeting_fov_deg=90.0,
            greeting_cooldown_sec=5.0,
        )
        world_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.node.create_subscription(
            String,
            self._namespace("state", "world"),
            self._cb_world_for_footsteps,
            world_qos,
        )

    # def publish_arena_peds(self, msg: Pedestrians):
    #     """Publish pedestrian states."""
    #     self._arena_peds_publisher.publish(msg)

    def publish_arena_peds(self, msg: Pedestrians):
        """Publish pedestrian states and derive auditory events."""
        self._arena_peds_publisher.publish(msg)

        now = self.node.sim_time
        now_sec = float(now.sec) + float(now.nanosec) * 1e-9
        self._auditory_events.update(msg, now_sec)

    def _cb_world_for_footsteps(self, msg: String) -> None:
        world_name = msg.data.strip()
        if not world_name:
            return

        try:
            world = WorldIdentifier(world_name).resolve_sync().load()
            self._acoustic_scene = AcousticScene.from_world(world)
        except Exception as exc:
            self._logger.warning(
                f"failed to load acoustic scene for footstep material mapping: {exc!r}"
            )
            self._acoustic_scene = None

    def _footstep_floor_tag(self, ped: Pedestrian) -> str:
        if self._acoustic_scene is None:
            return "default"

        zone = self._acoustic_scene.zone_at(ped.pose.position)
        if zone is None:
            return "default"

        if zone.floor_material_id == "Walnut_Planks":
            return "walnut_planks"

        return "default"

    def publish_sound_event(self, sound_type: str, ped: Pedestrian) -> None:
        sound_type = sound_type.strip()
        if not sound_type:
            return
        
        yaw = self._yaw_from_quaternion(ped.pose.orientation)
        stamp = self.node.sim_time.to_msg()

        msg = SoundEvent()
        msg.header.stamp = stamp
        msg.header.frame_id = "map" 
        msg.event_id = (f"{ped.id}:{stamp.sec}:{stamp.nanosec}:{next(self._sound_event_counter)}")
        msg.source_agent_id = int(ped.id)
        msg.source_agent_name = ped.name
        msg.sound_type = sound_type
        msg.label = sound_type
        msg.asset_id = sound_type
        msg.source_position = ped.pose.position
        msg.source_yaw = float(yaw)
        # msg.semantic_tags = ['default'] #FIXME: add semantic tags
        if sound_type == "footstep":
            msg.semantic_tags = ["footstep", self._footstep_floor_tag(ped)]
        else:
            msg.semantic_tags = ["default"]
        # msg.reference_distance_m = 1.0
        # msg.directivity_factor = 0.0
        msg.loop = False

        default_sound_params = {"footstep": (45.0, 0.2), "greeting": (60.0, 1.0),"motor": (55.0, 0.5)} 

        source_volume_db, duration_sec = default_sound_params.get(sound_type, (60.0, 1.0))
        msg.source_volume_db = float(source_volume_db)
        msg.duration.sec = int(duration_sec)
        msg.duration.nanosec = int((duration_sec % 1.0) * 1_000_000_000)

        self._sound_events_publisher.publish(msg)
        self._publish_sound_cone_marker(sound_type, ped)

    def _publish_sound_cone_marker(self, event: str, ped: Pedestrian) -> None:
        yaw = self._yaw_from_quaternion(ped.pose.orientation)

        source_x = float(ped.pose.position.x)
        source_y = float(ped.pose.position.y)

        apex_offset = 0.15
        cone_radius = 1.25
        cone_angle = math.radians(70.0)
        segments = 16

        apex = Point(x=source_x + math.cos(yaw) * apex_offset, y=source_y + math.sin(yaw) * apex_offset, z=0.08)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.sim_time.to_msg()
        marker.ns = f"human_sound_{event}"
        marker.id = next(self._sound_marker_counter)
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.lifetime = Duration(sec=1, nanosec=200_000_000)
        marker.color = self._sound_marker_color(event)

        arc_points: list[Point] = []
        start_angle = yaw - cone_angle / 2.0
        for i in range(segments + 1):
            angle = start_angle + cone_angle * (i / segments)
            arc_points.append(Point(x=source_x + math.cos(angle) * cone_radius, y=source_y + math.sin(angle) * cone_radius, z=0.08))

        for left, right in zip(arc_points, arc_points[1:], strict=False):
            marker.points.extend([apex, left, right])

        outline = Marker()
        outline.header = marker.header
        outline.ns = f"human_sound_{event}_outline"
        outline.id = next(self._sound_marker_counter)
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.035
        outline.lifetime = marker.lifetime
        outline.color = ColorRGBA(r=marker.color.r, g=marker.color.g, b=marker.color.b, a=0.95)
        outline.points = [apex, *arc_points, apex]

        markers = MarkerArray()
        markers.markers.extend([marker, outline])
        self._sound_marker_publisher.publish(markers)

    # TODO: add a method to publish sound events with arbitrary parameters (volume, duration, etc.)
    @staticmethod
    def _sound_marker_color(event: str) -> ColorRGBA:
        if event == "greeting":
            return ColorRGBA(r=0.2, g=0.75, b=1.0, a=0.35)
        if event == "footstep":
            return ColorRGBA(r=1.0, g=0.8, b=0.25, a=0.28)
        return ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.3)

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_markers(self, markers: MarkerArray) -> None:
        """Publish a transient debug-overlay MarkerArray on `pedestrian_markers/extra`."""
        self._marker_publisher.publish(markers)

    def publish_static_markers(self, markers: MarkerArray) -> None:
        """Publish a latched MarkerArray on `pedestrian_markers/static` (TRANSIENT_LOCAL, late subscribers catch up)."""
        self._static_marker_publisher.publish(markers)

    async def spawn_obstacles(self, obstacles: Sequence[Obstacle], layer: ObstacleLayer = ObstacleLayer.INUSE):
        """Spawns static obstacles.

        Args:
            obstacles (Sequence[Obstacle]): Static obstacles to spawn.
            layer (ObstacleLayer, optional): Layer to assign to spawned obstacles. Defaults to ObstacleLayer.INUSE.
        """
        self._logger.debug(f"spawning {len(obstacles)} static obstacles")

        futures: list[typing.Awaitable] = []
        to_register: list[KnownObstacle[Obstacle]] = []
        to_move: list[Obstacle] = []

        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                to_move.append(known.obstacle)
                known.layer = layer
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                )
            if not known.spawned:
                to_register.append(known)
        if to_move:
            futures.append(self._simulator.obstacle_move(to_move))

        to_spawn: list[Obstacle] = []
        for known, obstacle in zip(
            to_register,
            await self._spawn_obstacles_impl([known.obstacle for known in to_register]),
            strict=False,
        ):
            if not obstacle:
                continue
            known.obstacle = obstacle
            known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                to_spawn.append(known.obstacle)
            known.layer = layer

        if to_spawn:
            futures.append(self._simulator.obstacle_spawn(to_spawn))
        await asyncio.gather(*futures)

    async def spawn_dynamic_obstacles(self, obstacles: typing.Sequence[DynamicObstacle]):
        """Spawns dynamic obstacles.

        Args:
            obstacles (typing.Sequence[DynamicObstacle]): Dynamic obstacles to spawn.
        """
        self._logger.debug(f"spawning {len(obstacles)} dynamic obstacles")

        futures: list[typing.Awaitable] = []
        to_register: list[KnownObstacle[DynamicObstacle]] = []
        to_move: list[DynamicObstacle] = []

        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                to_move.append(known.obstacle)
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(name=obstacle.name, obstacle=obstacle)
            if not known.spawned:
                to_register.append(known)
        if to_move:
            futures.append(self._simulator.pedestrian_move(to_move))

        to_spawn: list[DynamicObstacle] = []
        for known, obstacle in zip(
            to_register,
            await self._spawn_dynamic_obstacles_impl([known.obstacle for known in to_register]),
            strict=False,
        ):
            self._logger.debug(f"Spawned dynamic obstacle: {obstacle}")
            if not obstacle:
                continue

            known.obstacle = obstacle
            known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                to_spawn.append(known.obstacle)
            known.layer = ObstacleLayer.INUSE

        if to_spawn:
            futures.append(self._simulator.pedestrian_spawn(await self._ensure_spawnable(to_spawn)))
        await asyncio.gather(*futures)

    _PEDESTRIAN_FALLBACK: typing.ClassVar[str] = "arenian"

    async def _ensure_spawnable(self, obstacles: Sequence[DynamicObstacle]) -> Sequence[DynamicObstacle]:
        """Swap unresolvable ped models for _PEDESTRIAN_FALLBACK."""

        async def _resolve(obs: DynamicObstacle) -> DynamicObstacle:
            try:
                view = await obs.model.resolve()
                model = await view.model.get(ModelType.SDF)
                if model.type is not ModelType.UNKNOWN:
                    return obs
            except Exception as e:
                self._logger.warning(
                    f"pedestrian {obs.name!r}: model {obs.model.name!r} unresolved ({e}); using fallback",
                )
            else:
                self._logger.warning(
                    f"pedestrian {obs.name!r}: model {obs.model.name!r} has no SDF; using fallback",
                )
            return attrs.evolve(obs, model=PedestrianIdentifier.parse(self._PEDESTRIAN_FALLBACK))

        return await asyncio.gather(*(_resolve(o) for o in obstacles))

    async def spawn_world(
        self,
        walls: Sequence[Wall],
        doors: Sequence[Door],
    ):
        """Spawns world elements.

        Args:
            walls (Sequence[Wall]): Walls to spawn.
            doors (Sequence[Door]): Doors to spawn.
        """
        self._logger.debug(f"spawning {len(walls)} walls and {len(doors)} doors")

        wall_map: dict[str, Wall] = {}
        for wall in walls:
            name = f"wall_{next(self._wall_counter)}"
            self._known_walls.create_or_get(
                name=name,
                obstacle=wall,
                layer=ObstacleLayer.WORLD,
            )
            wall_map[name] = wall

        for door in doors:
            self._known_doors.create_or_get(
                name=door.name,
                obstacle=door,
                layer=ObstacleLayer.WORLD,
            )

        await asyncio.gather(
            self._simulator.spawn_doors(doors),
            self._simulator.spawn_walls(walls),
            self._spawn_walls_impl(wall_map),
        )

    @staticmethod
    def _door_wall_name(door_name: str) -> str:
        return f"__door_{door_name}"

    async def update_doors(
        self,
        doors: Sequence[tuple[Door, bool]],
    ):
        """Update door states. True = open (no wall), False = closed (wall spawned).

        Args:
            doors (Sequence[tuple[Door, bool]]): Door/state pairs.
        """
        to_spawn: dict[str, Wall] = {}
        to_remove: list[str] = []

        for door, is_open in doors:
            name = self._door_wall_name(door.name)
            if is_open:
                if name in self._known_walls:
                    to_remove.append(name)
            else:
                if name not in self._known_walls:
                    wall = Wall(start=door.start, end=door.end)
                    self._known_walls.create_or_get(
                        name=name,
                        obstacle=wall,
                        layer=ObstacleLayer.WORLD,
                    )
                    to_spawn[name] = wall

        futures: list[typing.Awaitable] = []
        if to_spawn:
            self._logger.debug(f"closing {len(to_spawn)} doors: {list(to_spawn)}")
            futures.append(self._spawn_walls_impl(to_spawn))
        if to_remove:
            self._logger.debug(f"opening {len(to_remove)} doors: {to_remove}")
            for name in to_remove:
                self._known_walls.forget(name)
            futures.append(self._remove_walls_impl(to_remove))
        await asyncio.gather(*futures)

    async def unuse_obstacles(self):
        """
        Prepares obstacles for reuse or removal.
        """
        self._logger.debug("unusing obstacles")

        obstacle_names = [name for name, known in self._known_obstacles.items() if not isinstance(known.obstacle, DynamicObstacle) and known.layer == ObstacleLayer.UNUSED]

        await asyncio.gather(
            self._remove_pedestrians_impl(),
            self._remove_obstacles_impl(obstacle_names),
        )

        for name in obstacle_names:
            self._known_obstacles.forget(name)

        for obstacle in self._known_obstacles.values():
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.spawned = False
                obstacle.layer = ObstacleLayer.UNUSED

    def unuse_world(self) -> tuple[frozenset[str], frozenset[str]]:
        """Mark world-level items for replacement.

        Downgrades WORLD-layer obstacles to UNUSED so they can be
        reclaimed after new world obstacles are spawned.

        Returns:
            Tuple of (old_wall_names, old_door_names) for stale-entry cleanup.
        """
        self._logger.debug("unusing world")

        old_walls = frozenset(self._known_walls.keys())
        old_doors = frozenset(self._known_doors.keys())

        for obstacle in self._known_obstacles.values():
            if obstacle.layer >= ObstacleLayer.WORLD:
                obstacle.spawned = False
                obstacle.layer = ObstacleLayer.UNUSED

        return old_walls, old_doors

    def remove_stale_world(
        self,
        old_walls: frozenset[str],
        old_doors: frozenset[str],
    ):
        """Forget wall/door tracking entries that were not reused during respawn.

        Args:
            old_walls: Wall names from before the respawn.
            old_doors: Door names from before the respawn.
        """
        for name in old_walls:
            if name in self._known_walls:
                self._known_walls.forget(name)
        for name in old_doors:
            if name in self._known_doors:
                self._known_doors.forget(name)

    async def remove_walls(
        self,
        walls: Sequence[Wall],
    ):
        """Removes specific walls from the simulation.

        Args:
            walls (Sequence[Wall]): Walls to remove (matched by start/end coordinates).
        """
        matched: dict[str, None] = {}
        for wall in walls:
            for name, known in self._known_walls.items():
                if known.obstacle.start == wall.start and known.obstacle.end == wall.end:
                    matched[name] = None
                    break

        if not matched:
            return

        names = tuple(matched)
        self._logger.debug(f"removing {len(names)} walls: {names}")
        for name in names:
            self._known_walls.forget(name)
        await self._remove_walls_impl(names)

    async def remove_doors(
        self,
        doors: Sequence[Door],
    ):
        """Removes specific doors from the simulation.

        Args:
            doors (Sequence[Door]): Doors to remove (matched by name).
        """
        names = [door.name for door in doors if door.name in self._known_doors]
        if not names:
            return

        self._logger.debug(f"removing {len(names)} doors: {names}")
        for name in names:
            self._known_doors.forget(name)
        await self._remove_doors_impl(names)

    async def setup_regions(self, regions: typing.Sequence[Region]) -> bool:
        """Configure regions (sources/sinks) for dynamic agent spawning.

        Args:
            regions: Already-resolved Region objects with concrete polygons.
        """
        for region in regions:
            self._known_regions[region.name] = region
        return await self._add_regions_impl(regions)

    async def remove_all_regions(self) -> bool:
        """Remove all tracked regions."""
        regions = tuple(self._known_regions.values())
        self._known_regions.clear()
        return await self._remove_regions_impl(regions)

    async def remove_obstacles(self, purge: ObstacleLayer = ObstacleLayer.UNUSED):
        """Removes obstacles from simulator.

        Args:
            purge (ObstacleLayer, optional): Level of obstacles to remove. Defaults to ObstacleLayer.UNUSED.
        """
        self._logger.debug(f"removing obstacles (level {purge})")
        futures: list[typing.Awaitable] = []

        stale_walls = [name for name, known in self._known_walls.items() if purge >= known.layer]
        stale_doors = [name for name, known in self._known_doors.items() if purge >= known.layer]

        if purge >= ObstacleLayer.WORLD:
            futures.append(self._simulator.remove_world())

        if stale_walls:
            futures.append(self._remove_walls_impl(stale_walls))
            for name in stale_walls:
                self._known_walls.forget(name)
        if stale_doors:
            futures.append(self._remove_doors_impl(stale_doors))
            for name in stale_doors:
                self._known_doors.forget(name)

        static: list[Obstacle] = []
        dynamic: list[DynamicObstacle] = []
        for oid, known in list(self._known_obstacles.items()):
            if purge >= known.layer:  # tmp: always respawn all dynamic obstacles
                if isinstance(known.obstacle, DynamicObstacle):
                    dynamic.append(known.obstacle)
                else:
                    static.append(known.obstacle)
                self._known_obstacles.forget(name=oid)

        static_names = [o.name for o in static]
        if static_names:
            futures.append(self._remove_obstacles_impl(static_names))
        futures.append(self._simulator.obstacle_delete(static))
        futures.append(self._simulator.pedestrian_delete(dynamic))
        await asyncio.gather(*futures)

    async def _sim_then_impl(
        self,
        sim_call: typing.Callable[[Sequence[Robot]], typing.Awaitable[Sequence[bool]]],
        impl_call: typing.Callable[[Sequence[Robot]], typing.Awaitable[Sequence[bool]]],
        robots: Sequence[Robot],
    ) -> tuple[bool, ...]:
        sim_success = await sim_call(robots)
        succeeded = tuple(r for r, s in zip(robots, sim_success, strict=False) if s)
        human_success = await impl_call(succeeded)
        human_iter = iter(human_success)
        return tuple(s and next(human_iter) for s in sim_success)

    async def spawn_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """Spawns robots.

        Args:
            robots (Sequence[Robot]): Robots to spawn.

        Returns:
            Sequence[bool]: Success of each robot spawn.
        """
        self._logger.debug(f"spawning {len(robots)} robots")
        return await self._sim_then_impl(
            self._simulator.robot_spawn,
            self._spawn_robot_impl,
            robots,
        )

    async def remove_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """Removes robots from the simulation.

        Args:
            robots (Sequence[Robot]): Robots to remove.

        Returns:
            Sequence[bool]: Success of each robot removal.
        """
        self._logger.debug(f"removing {len(robots)} robots")
        return await self._sim_then_impl(
            self._simulator.robot_delete,
            self._remove_robot_impl,
            robots,
        )

    async def move_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """Moves robots.

        Args:
            robots (Sequence[Robot]): Robots to move.

        Returns:
            Sequence[bool]: Success of each robot move.
        """
        self._logger.debug(f"moving {len(robots)} robots")
        return await self._sim_then_impl(
            self._simulator.robot_move,
            self._move_robot_impl,
            robots,
        )

    # impl

    async def pause(self):
        pass

    async def unpause(self):
        pass

    @abc.abstractmethod
    async def _spawn_obstacles_impl(
        self,
        obstacles: Sequence[Obstacle],
    ) -> Sequence[Obstacle | None]: ...

    @abc.abstractmethod
    async def _spawn_dynamic_obstacles_impl(
        self,
        obstacles: Sequence[DynamicObstacle],
    ) -> Sequence[DynamicObstacle | None]: ...

    @abc.abstractmethod
    async def _remove_obstacles_impl(
        self,
        names: Sequence[str],
    ) -> bool: ...

    @abc.abstractmethod
    async def _remove_pedestrians_impl(
        self,
    ) -> bool: ...

    @abc.abstractmethod
    async def _spawn_walls_impl(
        self,
        walls: Mapping[str, Wall],
    ) -> bool: ...

    @abc.abstractmethod
    async def _spawn_doors_impl(
        self,
        doors: Mapping[str, Door],
    ) -> bool: ...

    @abc.abstractmethod
    async def _remove_walls_impl(
        self,
        names: Sequence[str],
    ) -> bool: ...

    @abc.abstractmethod
    async def _remove_doors_impl(
        self,
        names: Sequence[str],
    ) -> bool: ...

    @abc.abstractmethod
    async def _spawn_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]: ...

    @abc.abstractmethod
    async def _remove_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]: ...

    @abc.abstractmethod
    async def _move_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]: ...

    @abc.abstractmethod
    async def _add_regions_impl(self, regions: Sequence[Region]) -> bool: ...

    @abc.abstractmethod
    async def _remove_regions_impl(self, regions: Sequence[Region]) -> bool: ...


HumanSimulatorRegistry = Registry[Constants.HumanSimulator, BaseHumanSimulator]()


@HumanSimulatorRegistry.register(Constants.HumanSimulator.DUMMY)
async def dummy(**kwargs: object) -> BaseHumanSimulator:
    from .dummy import DummyHumanSimulator

    return DummyHumanSimulator(**kwargs)


@HumanSimulatorRegistry.register(Constants.HumanSimulator.HUNAV)
async def lazy_hunavsim(**kwargs: object) -> BaseHumanSimulator:
    from .hunav.hunav import HunavHumanSimulator

    return await HunavHumanSimulator.create(**kwargs)


@HumanSimulatorRegistry.register(Constants.HumanSimulator.ISAAC)
async def isaacsim(**kwargs: object) -> BaseHumanSimulator:
    from .isaac import IsaacHumanSimulator

    return IsaacHumanSimulator(**kwargs)


@HumanSimulatorRegistry.register(Constants.HumanSimulator.ARENA)
async def arenasim(**kwargs: object) -> BaseHumanSimulator:
    from .arena_humansim.arena_humansim import ArenaHumanSimulator

    return await ArenaHumanSimulator.create(**kwargs)

@HumanSimulatorRegistry.register(Constants.HumanSimulator.AUDITORY)
async def auditorysim(**kwargs: object) -> BaseHumanSimulator:
    from .auditory import AuditoryHumanSimulator

    return AuditoryHumanSimulator(**kwargs)
