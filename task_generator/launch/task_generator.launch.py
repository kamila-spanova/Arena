import atexit
import contextlib
import os
import tempfile
import time

import launch
import launch.conditions
import launch.event_handlers
import launch.substitutions
import launch_ros.actions
import yaml
from ament_index_python.packages import get_package_share_directory
from arena_bringup.actions import IsolatedGroupAction
from arena_bringup.extensions.NodeLogLevelExtension import SetGlobalLogLevelAction
from arena_bringup.substitutions import LaunchArgument
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

_REGISTER_RETRY_SEC = 1.0
_REGISTER_LOG_INTERVAL_SEC = 10.0
_AUTO_ENV_ID = 0xFFFF


def _allocate_env(env_id: int, ns: str) -> tuple[int, str, str]:
    import rclpy
    from arena_runtime_msgs.srv import RegisterEnv
    from rclpy.node import Node

    if not rclpy.ok():
        rclpy.init(args=[])

    node = Node(f"task_generator_launch_{os.getpid()}_{env_id}")
    logger = node.get_logger()
    try:
        cli = node.create_client(RegisterEnv, "/arena/register_env")

        start = time.monotonic()
        next_log = start + _REGISTER_LOG_INTERVAL_SEC
        while not cli.wait_for_service(timeout_sec=_REGISTER_RETRY_SEC):
            now = time.monotonic()
            if now >= next_log:
                logger.warning(
                    f"waiting for /arena/register_env ({int(now - start)}s elapsed)"
                )
                next_log = now + _REGISTER_LOG_INTERVAL_SEC

        req = RegisterEnv.Request()
        req.caller_id = f"/task_generator_launch_{os.getpid()}"
        req.env_id = env_id
        req.ns = ns

        while True:
            future = cli.call_async(req)
            start = time.monotonic()
            next_log = start + _REGISTER_LOG_INTERVAL_SEC
            while not future.done():
                rclpy.spin_until_future_complete(node, future, timeout_sec=_REGISTER_RETRY_SEC)
                now = time.monotonic()
                if not future.done() and now >= next_log:
                    logger.warning(
                        f"waiting for /arena/register_env response ({int(now - start)}s elapsed)"
                    )
                    next_log = now + _REGISTER_LOG_INTERVAL_SEC
            resp = future.result()
            if resp.success:
                return (resp.env_id, resp.ns, resp.sim)
            if "not ACTIVE" in resp.error_msg:
                time.sleep(_REGISTER_RETRY_SEC)
                continue
            raise RuntimeError(f"/arena/register_env failed: {resp.error_msg}")
    finally:
        node.destroy_node()


def generate_launch_description():
    bringup_dir = get_package_share_directory("arena_bringup")

    ld_items = []
    LaunchArgument.auto_append(ld_items)

    log_level = LaunchArgument(
        name='log_level',
        default_value='warn',
        description='Per-node log level. See launch README.',
    )

    env_id = LaunchArgument(
        name="env_id",
        default_value=str(_AUTO_ENV_ID),
        description="Requested env id; 65535 = auto-allocate via /arena/register_env.",
    )

    managed = LaunchArgument(
        name="managed",
        default_value="false",
        description="true = arena pre-reserved; skip /arena/register_env and use env_id/ns from args. Placement comes via confirm_world either way.",
    )

    ns = LaunchArgument(
        name="ns",
        default_value="",
        description="Explicit ns path (e.g. for sim2real); empty = auto-generate.",
    )

    sim = LaunchArgument(name="sim", default_value="dummy", description="[dummy, gazebo, isaac]")
    # human/mobile defaults are derived from arena's authoritative `sim` (the value
    # arena_node actually configured), not from this launch's local `sim` arg, which only
    # affects how the env *requests* registration. Empty here means "use arena_sim".
    # User can still override by passing e.g. human:=hunav explicitly.
    human = LaunchArgument(
        name="human",
        default_value="",
        description="empty = derive from arena_sim ({dummy: dummy, gazebo|isaac: hunav})",
    )
    enable_auditory = LaunchArgument(
        name="enable_auditory",
        default_value="false",
        description="Start auditory propagation, robot hearing, sound playback, and robot sound nodes.",
    )
    propagation_backend = LaunchArgument(
        name="propagation_backend",
        default_value="level3",
        description="Auditory propagation backend: level3 or pyroomacoustics.",
    )
    robot = LaunchArgument(name="robot", default_value="jackal")
    tm_robots = LaunchArgument(name="tm_robots", default_value="explore")
    task_config = LaunchArgument(name="task_config", default_value="")
    episodes = LaunchArgument(
        name='episodes',
        default_value='-1',
        description='Stop the env after N episodes (-1 = run forever).',
    )
    scenario_file = LaunchArgument(
        name='scenario_file',
        default_value='',
        description='Sets task.scenario.file ROS param (empty = use parameter_file default).',
    )
    tm_obstacles = LaunchArgument(name="tm_obstacles", default_value="random")
    tm_modules = LaunchArgument(name="tm_modules", default_value="rviz_ui")
    optim = LaunchArgument(name="optim", default_value=os.environ.get("ARENA_OPTIM", ""))
    world = LaunchArgument(name="world", default_value="map_empty")
    mobile = LaunchArgument(
        name="mobile",
        default_value="",
        description="mobile adapter kind; empty = derive from arena_sim ({dummy: none, *: nav2})",
    )
    arm = LaunchArgument(
        name="arm",
        default_value="moveit",
        description="arm adapter kind",
    )
    record_data_dir = LaunchArgument(name="record_data_dir", default_value="")
    debug = LaunchArgument(name="debug", default_value="False")
    auto_reset = LaunchArgument(
        name="auto_reset",
        default_value="true",
        description=(
            "true = standalone: node auto-advances episodes. "
            "false = managed: external controller drives resets via lifecycle/reset_episode."
        ),
    )
    train_mode = LaunchArgument(name="train_mode", default_value="false")
    parameter_file = LaunchArgument(
        name="parameter_file",
        default_value=os.path.join(bringup_dir, "configs", "task_generator.yaml"),
    )

    def _build_env_actions(
        allocated_id: int,
        allocated_ns: str,
        arena_sim: str,
        context: launch.LaunchContext,
    ) -> list[launch.LaunchDescriptionEntity]:
        fqn = f"/{allocated_ns}"
        prefix_val = f"env_{allocated_id}"

        _label = f"arena env_{allocated_id}"
        with contextlib.suppress(OSError), open("/dev/tty", "w") as tty:
            tty.write(f"\033]0;{_label}\007\033]30;{_label}\007")
            tty.flush()

        def _restore_terminal_titles():
            with contextlib.suppress(OSError), open("/dev/tty", "w") as tty:
                tty.write("\033]0;\007\033]30;\007")
                tty.flush()
        atexit.register(_restore_terminal_titles)

        human_val = launch.utilities.perform_substitutions(
            context, launch.utilities.normalize_to_list_of_substitutions(human.substitution)
        ) or {"dummy": "dummy", "gazebo": "hunav", "isaac": "hunav"}.get(arena_sim, "dummy")
        mobile_val = launch.utilities.perform_substitutions(
            context, launch.utilities.normalize_to_list_of_substitutions(mobile.substitution)
        ) or {"dummy": "none"}.get(arena_sim, "nav2")
        arm_val = launch.utilities.perform_substitutions(
            context, launch.utilities.normalize_to_list_of_substitutions(arm.substitution)
        )

        human_launch = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("task_generator"),
                "launch", "human", "human.launch.py",
            ]),
            launch_arguments={
                "simulator": human_val,
                "namespace": allocated_ns,
                "enable_auditory": enable_auditory.substitution,
                "propagation_backend": propagation_backend.substitution,
            }.items(),
        )

        pedestrian_marker_node = launch_ros.actions.Node(
            package="rviz_utils",
            executable="pedestrian_marker_publisher",
            name="pedestrian_marker_publisher",
            parameters=[
                {"use_sim_time": True},
                {"body_height": 1.6},
                {"body_radius": 0.25},
                {"head_radius": 0.15},
                {"arrow_length": 0.6},
                {"show_labels": True},
                {"show_velocity_arrows": True},
                {"show_orientation_arrows": True},
                {"namespace": allocated_ns},
            ],
            output="screen",
        )

        def _coerce(raw: str) -> object:
            # accept list/dict/numeric coercions only, raw otherwise
            try:
                parsed = yaml.safe_load(raw)
            except yaml.YAMLError:
                return raw
            if isinstance(parsed, (list, dict, int, float)) and not isinstance(parsed, bool):
                return parsed
            return raw

        dotted_overrides: dict[str, object] = {}
        for k, v in context.launch_configurations.items():
            if k.startswith("task."):
                dotted_overrides[k] = _coerce(v)
            elif k.startswith("mobile.") or k.startswith("arm."):
                # `<cap>.<key>:=<val>` becomes `robot.<cap>.<key>` and lands as a
                # kwarg in RobotManager._adapter_kwargs_for, overlaying the
                # cap-file YAML for the bound adapter.
                dotted_overrides[f"robot.{k}"] = _coerce(v)

        # launch_ros.normalize_parameters turns list values into tuples and
        # yaml.dump emits them with !!python/tuple, which rcl drops silently.
        # Bypass by writing our own params yaml and passing the path.
        overrides_files: list[str] = []
        if dotted_overrides:
            fh = tempfile.NamedTemporaryFile(
                mode='w', prefix='arena_overrides_', suffix='.yaml', delete=False
            )
            yaml.safe_dump(
                {'/**': {'ros__parameters': dotted_overrides}},
                fh,
                default_flow_style=False,
            )
            fh.close()
            overrides_files.append(fh.name)

        task_generator_node = launch_ros.actions.Node(
            package="task_generator",
            executable="task_generator_node",
            namespace=os.path.dirname(allocated_ns),
            name=os.path.basename(allocated_ns),
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "sim": arena_sim,
                    "human": human_val,
                    "robot.mobile_adapter": mobile_val,
                    "robot.arm_adapter": arm_val,
                    **robot.str_param,
                    **tm_robots.str_param,
                    **tm_obstacles.str_param,
                    **tm_modules.str_param,
                    **optim.str_param,
                    **world.str_param,
                    **record_data_dir.str_param,
                    **debug.param(bool),
                    **auto_reset.param(bool),
                    **train_mode.param(bool),
                    "env_id": allocated_id,
                    "prefix": prefix_val,
                },
                parameter_file.substitution,
                {
                    **episodes.param(int),
                    'task.scenario.file': scenario_file.substitution,
                },
                *overrides_files,
            ],
        )

        aiomonitor_port = 20101 + max(allocated_id, 0) * 10
        debug_window_cb = launch.event_handlers.OnProcessStart(
            target_action=task_generator_node,
            on_start=[
                ExecuteProcess(
                    cmd=[
                        "/usr/bin/x-terminal-emulator",
                        "-e",
                        f'bash -c "sleep 5; python -m aiomonitor.cli -p {aiomonitor_port}"',
                    ],
                    output="screen",
                )
            ],
        )

        inner_group = launch.actions.GroupAction([
            PushRosNamespace(namespace=allocated_ns),
            pedestrian_marker_node,
        ])

        shutdown_on_node_exit = RegisterEventHandler(OnProcessExit(
            target_action=task_generator_node,
            on_exit=[launch.actions.Shutdown(reason="task_generator_node exited")],
        ))

        return [
            IsolatedGroupAction([human_launch, inner_group, task_generator_node]),
            launch.actions.RegisterEventHandler(
                debug_window_cb,
                condition=launch.conditions.IfCondition(debug.substitution),
            ),
            shutdown_on_node_exit,
        ]

    def _make_env(context: launch.LaunchContext) -> list[launch.LaunchDescriptionEntity]:
        managed_val = launch.utilities.perform_substitutions(
            context, launch.utilities.normalize_to_list_of_substitutions(managed.substitution)
        ).lower() in ("true", "1")
        sim_val = launch.utilities.perform_substitutions(
            context, launch.utilities.normalize_to_list_of_substitutions(sim.substitution)
        )
        if managed_val:
            allocated_id = int(launch.utilities.perform_substitutions(
                context, launch.utilities.normalize_to_list_of_substitutions(env_id.substitution)
            ))
            allocated_ns = launch.utilities.perform_substitutions(
                context, launch.utilities.normalize_to_list_of_substitutions(ns.substitution)
            ).lstrip("/")
            arena_sim = sim_val
        else:
            requested = int(launch.utilities.perform_substitutions(
                context, launch.utilities.normalize_to_list_of_substitutions(env_id.substitution)
            ))
            ns_val = launch.utilities.perform_substitutions(
                context, launch.utilities.normalize_to_list_of_substitutions(ns.substitution)
            )
            allocated_id, allocated_ns, arena_sim = _allocate_env(requested, ns_val)
        return _build_env_actions(allocated_id, allocated_ns, arena_sim, context)

    return launch.LaunchDescription([
        *ld_items,
        SetGlobalLogLevelAction(log_level.substitution),
        OpaqueFunction(function=_make_env),
    ])


if __name__ == "__main__":
    generate_launch_description()
