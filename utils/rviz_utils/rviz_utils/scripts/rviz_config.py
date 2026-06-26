#! /usr/bin/env python3

import asyncio
import json
import os
import signal
import sys
import tempfile
import typing

import arena_bringup.extensions.NodeLogLevelExtension as NodeLogLevelExtension
import launch
import launch_ros.actions
import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from arena_rclpy_mixins import ArenaMixinNode
from arena_rclpy_mixins.shared import FrameNamespace
from arena_robots.moveit_factory import build_moveit_params
from task_generator_msgs.msg import AdapterVizManifest, RobotDescriptor, RobotFleet

from rviz_utils.utils import Utils


class ConfigFileGenerator(ArenaMixinNode):
    topics: list[tuple[str, list[str]]]
    robots: list[RobotDescriptor]
    viz_manifest: AdapterVizManifest
    _frame_prefix: str
    _env_id: int

    def __init__(self, TASKGEN_NODE: str = '/task_generator_node'):
        super().__init__('rviz_config_generator')

        self._TASKGEN_NODE = TASKGEN_NODE

    async def _await_param(
        self,
        client: rclpy.client.Client,
        param_name: str,
        test_fn: typing.Callable[[typing.Any], bool] | None = None,
        interval: float = 1.0,
    ) -> rcl_interfaces.msg.ParameterValue:
        """Block until parameter passes test function."""
        while True:
            self.get_logger().info(f'waiting for {param_name} to be set')
            req = rcl_interfaces.srv.GetParameters.Request(names=[param_name])
            params = await self.await_ros(client.call_async(req))
            if params and params.values:
                value = params.values[0]
                if (not test_fn) or test_fn(value):
                    self.get_logger().info(f'param {param_name} is set')
                    return value
            await asyncio.sleep(interval)

    async def _await_first_fleet(self) -> RobotFleet:
        loop = asyncio.get_running_loop()
        future: asyncio.Future[RobotFleet] = loop.create_future()
        topic = os.path.join(self._TASKGEN_NODE, 'state', 'robots')
        sub = self.create_subscription(
            RobotFleet,
            topic,
            lambda msg: loop.call_soon_threadsafe(future.set_result, msg) if not future.done() else None,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        try:
            self.get_logger().info(f'waiting for first {topic} message')
            return await future
        finally:
            self.destroy_subscription(sub)

    async def _await_viz_manifest(self) -> AdapterVizManifest:
        loop = asyncio.get_running_loop()
        future: asyncio.Future[AdapterVizManifest] = loop.create_future()
        topic = os.path.join(self._TASKGEN_NODE, 'state', 'viz_manifest')
        sub = self.create_subscription(
            AdapterVizManifest,
            topic,
            lambda msg: loop.call_soon_threadsafe(future.set_result, msg) if not future.done() else None,
            qos_profile=rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        try:
            self.get_logger().info(f'waiting for first {topic} message')
            return await future
        finally:
            self.destroy_subscription(sub)

    async def setup(self) -> None:
        TASKGEN_PARAM_SRV = os.path.join(self._TASKGEN_NODE, 'get_parameters')
        PARAM_INITIALIZED = 'initialized'

        get_parameters_cli = self.create_client(rcl_interfaces.srv.GetParameters, TASKGEN_PARAM_SRV)
        self.get_logger().info(f'waiting for service {TASKGEN_PARAM_SRV} to become available')
        await self.wait_for_service_async(get_parameters_cli)
        self.get_logger().info(f'service {TASKGEN_PARAM_SRV} is available')

        await self._await_param(get_parameters_cli, PARAM_INITIALIZED, lambda x: x.bool_value)
        self._frame_prefix = (await self._await_param(get_parameters_cli, 'prefix')).string_value
        self._env_id = (await self._await_param(get_parameters_cli, 'env_id')).integer_value
        self.robots = list((await self._await_first_fleet()).robots)
        self.viz_manifest = await self._await_viz_manifest()

        config_file = self.create_config()

        rviz_parameters: list[dict[str, object]] = [{"use_sim_time": True}]
        arm_params = self._collect_moveit_params()
        if arm_params:
            rviz_parameters.append(arm_params)

        launch_task = await self._launch_manager.launch_description(
            launch.LaunchDescription(
                [
                    NodeLogLevelExtension.SetGlobalLogLevelAction(rclpy.logging.get_logger_effective_level(self.get_logger().name).name.lower()),
                    launch_ros.actions.Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        arguments=['-d', config_file],
                        parameters=rviz_parameters,
                        output="screen",
                    ),
                ]
            )
        )
        await launch_task
        self.get_logger().info('rviz2 exited, shutting down supervisor')
        os.kill(os.getpid(), signal.SIGINT)

    def _collect_moveit_params(self) -> dict[str, object]:
        """Mirror each arm robot's MoveIt config into rviz2 under a robot-named
        prefix. Per-display ``Robot Description`` properties (see the moveit
        adapter's display hints) point at ``<robot>.robot_description`` so
        every arm robot gets its own Trajectory/PlanningScene display."""
        combined: dict[str, object] = {}
        arm_names: list[str] = []
        for robot in self.robots:
            tf_prefix = FrameNamespace(robot.frame).raw()
            tf_prefix = tf_prefix + "/" if tf_prefix else ""
            params = build_moveit_params(robot.model, tf_prefix=tf_prefix)
            if params is None:
                continue
            arm_names.append(robot.name)
            for key, value in params.items():
                combined[f"{robot.name}.{key}"] = value
        if arm_names:
            self.get_logger().info(f"injecting MoveIt params into rviz2 for: {arm_names}")
        return combined

    def _create_pedestrian_group(self) -> dict[str, object]:
        """Creates a Pedestrian Group with stylized human visualizations"""

        pedestrian_group = {'Class': 'rviz_common/Group', 'Name': 'Pedestrians', 'Enabled': True, 'Displays': []}

        pedestrian_topics = []
        for topic_name, topic_types in self.topics:
            if os.path.basename(topic_name) == 'arena_peds' and 'arena_people_msgs/msg/Pedestrians' in topic_types:
                pedestrian_topics.append((topic_name, 'arena_people_msgs/msg/Pedestrians'))
            elif 'visualization_msgs/msg/MarkerArray' in topic_types and (
                topic_name.endswith('/pedestrian_markers')
                or '/pedestrian_markers/' in topic_name
                or topic_name.endswith('/wall_markers')
                or topic_name.endswith('/human_sound_markers')
            ):
                pedestrian_topics.append((topic_name, 'visualization_msgs/msg/MarkerArray'))
            elif topic_name.endswith('/people') and 'people_msgs/msg/People' in topic_types:
                pedestrian_topics.append((topic_name, 'people_msgs/msg/People'))
            elif topic_name.endswith('/human_states') and 'hunav_msgs/msg/Agents' in topic_types:
                pedestrian_topics.append((topic_name, 'hunav_msgs/msg/Agents'))
        
        expected_sound_topic = os.path.join(self._TASKGEN_NODE, "human_sound_markers")
        if expected_sound_topic not in [topic for topic, _ in pedestrian_topics]:
            pedestrian_topics.append((expected_sound_topic, 'visualization_msgs/msg/MarkerArray'))

        if not pedestrian_topics:
            self.get_logger().info("No pedestrian topics found. Pedestrian group will be empty.")
            return pedestrian_group

        # Add displays for found pedestrian topics
        # Add displays for found pedestrian topics
        for topic_name, topic_type in pedestrian_topics:
            if topic_type == 'arena_people_msgs/msg/Pedestrians':
                self.get_logger().info(f"Found arena_peds topic: {topic_name} - using pedestrian_markers")
                # BaseHumanSimulator publishes MarkerArray on pedestrian_markers

            elif topic_type == 'visualization_msgs/msg/MarkerArray':
                leaf = os.path.basename(topic_name)
                is_static = leaf.startswith('static')
                is_sound = leaf == 'human_sound_markers'
                is_legacy_unbucketed_static = leaf == 'static'
                enabled = not (topic_name.endswith('/wall_markers') or leaf == 'extra' or is_legacy_unbucketed_static)
                display = Utils.Displays.pedestrians(
                    topic_name,
                    name='sound_cones' if is_sound else leaf,
                    enabled=enabled,
                    reliability='Reliable' if is_static else 'Best Effort',
                    # reliability='Best Effort',
                    durability='Transient Local' if is_static else 'Volatile',
                    # durability='Volatile',
                )
                pedestrian_group['Displays'].append(display)
                self.get_logger().info(f"Added MarkerArray display for pedestrians: {topic_name}")

            elif topic_type == 'people_msgs/msg/People':
                # Add raw people display as fallback
                display = Utils.Displays.pedestrians_raw(topic_name)
                pedestrian_group['Displays'].append(display)
                self.get_logger().info(f"Added raw People display: {topic_name}")

            elif topic_type == 'hunav_msgs/msg/Agents':
                # Could add custom agent display here if needed
                self.get_logger().info(f"Found HuNav agents topic: {topic_name} (not yet implemented)")

        # Add TF display for pedestrian frames (disabled fallback only)
        tf_display = {
            'Class': 'rviz_default_plugins/TF',
            'Name': 'Pedestrian TF Frames',
            'Enabled': False,  # Disabled by default since we have proper markers
            'Frame Timeout': 15,
            'Marker Scale': 0.3,
            'Show Arrows': True,
            'Show Axes': False,
            'Show Names': True,
        }
        pedestrian_group['Displays'].append(tf_display)

        return pedestrian_group

    def create_config(self) -> str:
        default_file = self._read_default_file()

        # cache
        self.topics = self.get_topic_names_and_types()

        displays = []

        # Add the map display
        displays.append(
            {
                'Class': 'rviz_default_plugins/Map',
                'Enabled': True,
                'Name': 'Map',
                'Topic': {
                    'Value': os.path.join(self._TASKGEN_NODE, 'map'),
                    'Depth': 20,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Durability Policy': 'Transient Local',
                },
                'Use Timestamp': False,
                'Alpha': 0.7,
            }
        )

        # Add TF display
        displays.append({'Class': 'rviz_default_plugins/TF', 'Enabled': False, 'Name': 'TF', 'Frame Timeout': 15, 'Marker Scale': 1.0, 'Show Arrows': True, 'Show Axes': True, 'Show Names': False})

        published_topics = [topic[0] for topic in self.get_topic_names_and_types()]

        for robot in self.robots:
            robot_group = self._create_robot_group(robot)
            displays.append(robot_group)

        # humans: pedestrian group
        pedestrian_group = self._create_pedestrian_group()
        displays.append(pedestrian_group)

        # PedSim configuration - commented out but kept for future use
        # try:
        #     if not self.has_parameter('pedsim'):
        #         self.declare_parameter('pedsim', False)
        #     if self.get_parameter('pedsim').value:
        #         displays.append(Config.TRACKED_PERSONS)
        #         displays.append(Config.TRACKED_GROUPS)
        #         displays.append(Config.PEDSIM_WALLS)
        #         displays.append(Config.PEDSIM_WAYPOINTS)
        # except Exception as e:
        #     self.get_logger().warn(f"Error checking pedsim parameter: {e}")

        # Set the default view to Orbit (instead of TopDownOrtho)

        python_yaw: float = 3.8
        try:
            python_yaw = sum(2 * (i % 2 - 0.5) * float(d) / 10**i for i, d in enumerate(sys.version.split(' ', 1)[0].split('.')))  # i am going insane
        except BaseException:
            pass

        default_file["Visualization Manager"]["Views"]["Current"] = {
            "Class": "rviz_default_plugins/Orbit",
            "Distance": 50.0,
            "Focal Point": {"X": 15.0, "Y": 10.0, "Z": 0.0},
            "Name": "Current View",
            "Near Clip Distance": 0.01,
            "Pitch": 0.9,
            "Target Frame": "<Fixed Frame>",
            "Value": True,
            "Yaw": python_yaw,
        }

        default_file["Visualization Manager"]["Displays"] = displays

        file_path = self._tmp_config_file(default_file, prefix=f"env{self._env_id}_")
        self.get_logger().info(f'created config file at {file_path}')

        return file_path

    def _start_setup_callback(self, request: object, response: object) -> object:
        self.get_logger().info("Service callback triggered.")
        file_path = self.create_config()
        self._send_load_config(file_path)
        return response

    def _create_robot_group(self, robot: RobotDescriptor) -> dict[str, object]:
        """Creates a Robot Group with all visualizations for a robot"""
        color = Utils.get_random_rviz_color()
        robot_ns = robot.ns
        robot_name = robot.name

        robot_group = {'Class': 'rviz_common/Group', 'Name': f'Robot: {robot_name}', 'Enabled': True, 'Displays': []}

        tf_prefix = FrameNamespace(robot.frame).raw()
        robot_model_topic = f'{robot_ns}/robot_description'
        robot_group['Displays'].append(Utils.Displays.robot_model(topic=robot_model_topic, robot_name=robot_name, tf_prefix=tf_prefix))

        odom_topic = f'{robot_ns}/odom'
        robot_group['Displays'].append(Utils.Displays.odom(odom_topic, color))

        heard_sound_marker_topic = f"{self._TASKGEN_NODE}/{robot_name}/heard_sound_marker"
        robot_group['Displays'].append({
            'Class': 'rviz_default_plugins/Marker',
            'Name': 'Heard Sound Marker',
            'Enabled': True,
            'Topic': {
                'Value': heard_sound_marker_topic,
                'Depth': 10,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Best Effort',
                'Durability Policy': 'Volatile',
            },
        })

        # adapter-declared displays for this robot
        for entry in self.viz_manifest.entries:
            if entry.robot_ns != robot_ns:
                continue
            for entry_display in entry.displays:
                display: dict[str, object] = {
                    'Class': entry_display.rviz_class,
                    'Name': entry_display.name,
                    'Enabled': True,
                }
                if entry_display.config_json:
                    display.update(json.loads(entry_display.config_json))
                topic = display.get('Topic')
                if isinstance(topic, dict):
                    topic['Value'] = entry_display.topic
                else:
                    display['Topic'] = {'Value': entry_display.topic}
                robot_group['Displays'].append(display)

        # SENSORS
        # Map of message types to display creator methods - include all sensor types
        sensor_displays = {
            'sensor_msgs/msg/LaserScan': Utils.Displays.laser_scan,
            'sensor_msgs/msg/PointCloud2': Utils.Displays.pointcloud,
            'sensor_msgs/msg/PointCloud': Utils.Displays.pointcloud_legacy,
            # 'sensor_msgs/msg/Imu': Utils.imu,                          # will be optimised soon
            'foot_contact_msgs/msg/FootContact': Utils.Displays.footcontact,
            'sensor_msgs/msg/Image': Utils.Displays.image,
            # Add more sensor types as needed
        }

        # Track sensor counts for color assignment
        sensor_counts = {}

        # Improved topic discovery for robot sensors
        robot_topics = []

        # Try to discover topics using node-based approach first
        try:
            # Get all nodes in the system
            node_names_and_namespaces = self.get_node_names_and_namespaces()

            # Filter for nodes related to this robot
            robot_nodes = []

            for node_name, node_namespace in node_names_and_namespaces:
                if node_namespace == robot_ns:
                    robot_nodes.append((node_name, node_namespace))

            self.get_logger().info(f"Found {len(robot_nodes)} nodes for robot {robot_name}")

            # Get topics from each robot node
            for node_name, node_namespace in robot_nodes:
                try:
                    node_topics = self.get_publisher_names_and_types_by_node(node_name, node_namespace)
                    robot_topics.extend(node_topics)
                except Exception as e:
                    self.get_logger().debug(f"Failed to get topics from {node_namespace}/{node_name}: {e}")
        except Exception as e:
            self.get_logger().warning(f"Failed to get topics by node: {e}")

        # Fall back to namespace filtering if node-based discovery failed
        if not robot_topics:
            robot_topics = [(t, types) for t, types in self.topics if t.startswith(robot_ns)]
            self.get_logger().info(f"Found {len(robot_topics)} topics using namespace filtering")

        # Add displays for all discovered sensors
        for topic_name, topic_types in robot_topics:
            for topic_type in topic_types:
                if topic_type in sensor_displays:
                    # Track count for this sensor type (for color assignment)
                    if topic_type not in sensor_counts:
                        sensor_counts[topic_type] = 0
                    else:
                        sensor_counts[topic_type] += 1

                    # Get display with appropriate color
                    display_creator = sensor_displays[topic_type]
                    sensor_color = Utils.get_sensor_color(topic_type, sensor_counts[topic_type])
                    display = display_creator(topic_name, sensor_color)

                    robot_group['Displays'].append(display)
                    break  # Use first matching type

        return robot_group

    def _read_default_file(self) -> dict[str, object]:
        package_path = get_package_share_directory("rviz_utils")
        file_path = os.path.join(package_path, "config", "rviz_default.rviz")

        fixed_frame = FrameNamespace(self._frame_prefix).tf('map')

        with open(file_path) as file:
            content = file.read()
            # i'm lazy, bite me
            content = content.format(
                task_generator_node=self._TASKGEN_NODE,
                fixed_frame=fixed_frame,
            )
            return yaml.safe_load(content)

    @classmethod
    def _tmp_config_file(cls, config_file: dict[str, object], prefix: str = "") -> str:
        f = tempfile.NamedTemporaryFile('w', delete=False, prefix=prefix)
        yaml.dump(config_file, f)
        f.close()
        return f.name


def main():
    cli_args = rclpy.utilities.remove_ros_args(sys.argv)
    ConfigFileGenerator.run_main(*cli_args[1:])


if __name__ == "__main__":
    main()
 