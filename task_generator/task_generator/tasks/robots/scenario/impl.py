import asyncio

from arena_rclpy_mixins.ROSParamServer import ROSParamT
from arena_simulation_setup.tree.World import WorldIdentifier
from arena_simulation_setup.tree.World.Scenario import RobotGoal, ScenarioGesturePhase, ScenarioGotoPhase

from task_generator.shared import Pose, Position, PositionRadius
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.robots.request import GoToPhase, PlayGesturePhase, TaskRequest


class TM_Scenario(TM_Robots):
    _config: ROSParamT[list[RobotGoal]]
    _idle_robots: set[str]

    def _parse_scenario(self, scenario: str) -> list[RobotGoal]:
        return WorldIdentifier(self._ctx.world_manager.loaded_world).resolve_sync().scenario(scenario).resolve_sync().load().robots

    async def reset(self, **kwargs: object) -> None:
        await super().reset(**kwargs)
        self._idle_robots = set()

        SCENARIO_ROBOTS = self._config.value

        managed_robots = list(self._ctx.robots.values())

        scenario_robots_length = len(SCENARIO_ROBOTS)
        setup_robot_length = len(managed_robots)

        if setup_robot_length > scenario_robots_length:
            managed_robots = managed_robots[:scenario_robots_length]
            self._logger.warn("Robot setup contains more robots than the scenario file.", once=True)

        if scenario_robots_length > setup_robot_length:
            SCENARIO_ROBOTS = SCENARIO_ROBOTS[:setup_robot_length]
            self._logger.warn("Scenario file contains more robots than setup.", once=True)

        # Shift floor-tagged poses from level-local to the flattened map frame.
        level_origins = self._ctx.world_manager.map.level_origins

        def _to_map_frame(pose: Pose, floor: str) -> Pose:
            if not floor:
                return pose
            ox, oy = level_origins.get(floor, (0.0, 0.0))
            return Pose(
                position=Position(pose.position.x + ox, pose.position.y + oy, pose.position.z),
                orientation=pose.orientation,
            )

        for robot, config in zip(managed_robots, SCENARIO_ROBOTS, strict=False):
            start_pose = _to_map_frame(config.start, config.start_floor)
            self._start_poses[robot.name] = start_pose

            phases: list[GoToPhase | PlayGesturePhase] = []
            forbidden: list[PositionRadius] = [
                PositionRadius(x=start_pose.position.x, y=start_pose.position.y, radius=robot.safe_distance),
            ]
            for phase in config.phase_list():
                if isinstance(phase, ScenarioGotoPhase):
                    goto_pose = _to_map_frame(phase.goto, config.goal_floor)
                    phases.append(GoToPhase(pose=goto_pose))
                    forbidden.append(PositionRadius(x=goto_pose.position.x, y=goto_pose.position.y, radius=robot.safe_distance))
                elif isinstance(phase, ScenarioGesturePhase):
                    phases.append(PlayGesturePhase(gesture=None if phase.gesture in ("", "random") else phase.gesture, instance=phase.instance))

            # An empty phase list intentionally describes a stationary robot.
            # Do not submit it to RobotManager: an empty TaskRequest has no
            # dispatchable phase.  Keep the scenario active until its normal
            # timeout (or until another non-idle robot finishes) so that
            # recordings can capture pedestrians passing a parked robot.
            if phases:
                await robot.submit_task(TaskRequest(phases=phases))
            else:
                self._idle_robots.add(robot.name)
            self._ctx.world_manager.forbid(forbidden)

    @property
    async def done(self) -> bool:
        """Treat empty scenario phase lists as parked, not completed, robots."""
        if (self.node.sim_time.sec - self._last_reset) > self.node.conf.Robot.TIMEOUT.value:
            return True

        active_robots = [
            manager
            for name, manager in self._ctx.robots.items()
            if name not in self._idle_robots
        ]
        if not active_robots:
            return False
        return all(await asyncio.gather(*(manager.is_done for manager in active_robots)))

    def __init__(self, **kwargs: object) -> None:
        TM_Robots.__init__(self, **kwargs)

        self._config = self.node.ROSParam[list[RobotGoal]](
            self.namespace('file'),
            'default.json',
            parse=self._parse_scenario,
        )
