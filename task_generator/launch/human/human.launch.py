import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arena_bringup.substitutions import LaunchArgument, SelectAction

from task_generator.constants import Constants
from launch_ros.actions import Node


def generate_launch_description():

    ld = []

    LaunchArgument.auto_append(ld)

    namespace = LaunchArgument(
        name='namespace',
    )

    enable_auditory = LaunchArgument(
        name="enable_auditory",
        default_value="false",
    )

    robot = LaunchArgument(
        name='robot',
        default_value='jackal',
    )

    launch_human_simulator = SelectAction(launch.substitutions.LaunchConfiguration('simulator'))

    launch_human_simulator.add(
        Constants.HumanSimulator.DUMMY.value,
        launch.actions.GroupAction([])
    )

    launch_human_simulator.add(
        Constants.HumanSimulator.AUDITORY.value,
        launch.actions.GroupAction([])
    )

    launch_human_simulator.add(
        Constants.HumanSimulator.ISAAC.value,
        launch.actions.GroupAction([])
    )

    launch_human_simulator.add(
        Constants.HumanSimulator.HUNAV.value,
        launch.actions.IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('task_generator'),
                'launch', 'human', 'hunav', 'hunav.launch.py',
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'world_file': '',
                **namespace.dict
            }.items(),
        )
    )

    # launch_human_simulator.add(
    #     Constants.HumanSimulator.ARENA.value,
    #     launch.actions.IncludeLaunchDescription(
    #         PathJoinSubstitution([
    #             FindPackageShare('task_generator'),
    #             'launch', 'human', 'arena_humansim', 'arena_humansim.launch.py',
    #         ]),
    #         launch_arguments={
    #             'use_sim_time': 'true',
    #             **namespace.dict
    #         }.items(),
    #     )
    # )
    launch_human_simulator.add(
        Constants.HumanSimulator.ARENA.value,
        launch.actions.GroupAction([
            launch.actions.IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('task_generator'),
                    'launch', 'human', 'arena_humansim', 'arena_humansim.launch.py',
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    **namespace.dict
                }.items(),
            ),
            # 1. Sound propagation node
            Node(
                package='task_generator',
                executable='sound_propagation_node',
                name='sound_propagation_node',
                namespace=namespace.substitution,
                output='screen',
                condition=launch.conditions.IfCondition(enable_auditory.substitution),
                parameters=[{
                    "use_sim_time": True,
                    "sound_events_topic": "human_sound_events",
                    "heard_sound_events_topic": "heard_sound_events",
                    "arena_peds_topic": "arena_peds",
                    "map_topic": "map",
                    "world_topic": "state/world",
                    "robot_fleet_topic": "state/robots",
                    "robots_hear_self": True,
                    "propagation_level": 3,
                    "default_hearing_threshold_db": 20.0,
                    # "occlusion_penalty_db": 20.0,
                    "max_first_order_reflections": 8,
                    "reflection_floor_db": -60.0,
                    "ceiling_height_m": 3.0,
                    "publish_inaudible": True,
                    "odom_topic_template": "{namespace}/{name}_velocity_controller/odom",
                }],
            ),

            # 2. Robot motor sound producer
            Node(
                package='task_generator',
                executable='robot_sound_node',
                name='robot_sound_node',
                namespace=namespace.substitution,
                output='screen',
                condition=launch.conditions.IfCondition(enable_auditory.substitution),
                parameters=[{
                    "use_sim_time": True,
                    "robot_fleet_topic": "state/robots",
                    "sound_events_topic": "human_sound_events",
                    "odom_topic_template": "{namespace}/{name}_velocity_controller/odom",
                    "sound_type": "motor",
                    "asset_id": "motor",
                    "source_volume_db": 55.0,
                    "publish_period_sec": 0.5,
                    "only_when_moving": False,
                    "min_speed_mps": 0.02,
                }],
            ),

             # 3. Robot hearing node
            Node(
                package='task_generator',
                executable='robot_hearing_node',
                name='robot_hearing_node',
                namespace=namespace.substitution,
                output='screen',
                condition=launch.conditions.IfCondition(enable_auditory.substitution),
                # parameters=[{
                #     "use_sim_time": True,
                #     "robot_name": launch.substitutions.LaunchConfiguration("robot_name"),
                #     "output_topic": [launch.substitutions.LaunchConfiguration("robot_name"), "/heard_sound"],
                #     "marker_topic": [launch.substitutions.LaunchConfiguration("robot_name"),"/heard_sound_marker" ],
                #     "heard_sound_events_topic": "heard_sound_events",
                #     # "output_topic": "robot1/heard_sound",
                #     "ignore_self": True,
                #     "min_snr_db": 3.0,
                #     "honor_propagation_delay": True,
                #     # "marker_topic": "robot1/heard_sound_marker",
                #     "marker_lifetime_sec": 1.5,
                #     "marker_z_offset": 1.2,
                # }],
                parameters=[{
                    "use_sim_time": True,
                    "robot_fleet_topic": "state/robots",
                    "heard_sound_events_topic": "heard_sound_events",
                    "heard_sound_topic_suffix": "heard_sound",
                    "marker_topic_suffix": "heard_sound_marker",
                    "ignore_self": True,
                    "min_snr_db": -5.0,
                    "honor_propagation_delay": True,
                    "marker_lifetime_sec": 1.5,
                    "marker_z_offset": 1.2,
                }],
            ),

            # 4. Human sound playback node
            Node(
                package='task_generator',
                executable='human_sound_playback',
                name='human_sound_playback',
                namespace=namespace.substitution,
                output='screen',
                condition=launch.conditions.IfCondition(enable_auditory.substitution),
                parameters=[{
                    "sound_events_topic": "human_sound_events",
                    "episode_topic": "state/episode",
                    "output_sample_rate": 44100,
                    "output_channels": 2,
                    "block_size":  4096,
                    "audio_device": "",
                    "master_gain_db": 0.0,
                    # 'player_command': 'aplay',
                }],
            ),
        ])
    )

    # launch_human_simulator.add(
    #     Constants.HumanSimulator.ARENA.value,
    #     launch.actions.GroupAction([
    #         launch.actions.IncludeLaunchDescription(
    #             PathJoinSubstitution([
    #                 FindPackageShare('task_generator'),
    #                 'launch', 'human', 'arena_humansim', 'arena_humansim.launch.py',
    #             ]),
    #             launch_arguments={
    #                 'use_sim_time': 'true',
    #                 **namespace.dict
    #             }.items(),
    #         ),
    #         Node(
    #             package='task_generator',
    #             executable='human_sound_playback',
    #             name='human_sound_playback',
    #             # executable='sound_propagation_node',
    #             # name='sound_propagation_node',
    #             namespace=namespace.substitution,
    #             output='screen',
    #             parameters=[{
    #                 'human_sound_events': 'human_sound_events',
    #                 'arena_peds_topic': 'arena_peds',
    #                 'map_topic': 'map',
    #                 'robot_fleet_topic': 'state/robots',
    #                 'player_command': 'aplay',
    #             }],
    #         ),
    #     ])
    # )

    simulator = LaunchArgument(
        name='simulator',
        choices=launch_human_simulator.keys,
    )

    ld = launch.LaunchDescription([
        *ld,
        launch_human_simulator,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
