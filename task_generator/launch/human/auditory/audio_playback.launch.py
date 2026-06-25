from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="task_generator",
            executable="human_sound_playback",
            name="human_sound_playback",
            output="screen",
            parameters=[{
                "sound_events_topic": "human_sound_events",
                # "sound_dir": "task_generator/sounds",
                # "asset_catalog": "task_generator/config/auditory/acoustic_assets.yaml",
                # "sound_map": '{"footstep": "footstep.wav", "greeting": "greeting.wav"}',
            }],
        ),
    ])