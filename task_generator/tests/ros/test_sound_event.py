from __future__ import annotations

import math
import time
import uuid
import pytest


@pytest.fixture(autouse=True)
def _ros_gate():
    pytest.importorskip("rclpy")
    pytest.importorskip("arena_people_msgs.msg")
    pytest.importorskip("geometry_msgs.msg")
    pytest.importorskip("task_generator_msgs.msg")


def _spin_until(rclpy, nodes, predicate, timeout_sec: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.02)

        if predicate():
            return

    raise AssertionError("timed out waiting for ROS round-trip")


def _make_pedestrian(ped_id: int, x: float, y: float):
    from arena_people_msgs.msg import Pedestrian

    ped = Pedestrian()
    ped.id = ped_id
    ped.name = f"ped_{ped_id}"
    ped.pose.position.x = x
    ped.pose.position.y = y
    ped.pose.position.z = 0.0
    ped.pose.orientation.w = 1.0
    return ped


def _make_sound_event():
    from task_generator_msgs.msg import SoundEvent

    event = SoundEvent()
    event.header.frame_id = "map"
    event.event_id = "roundtrip_001"
    event.source_agent_id = 1
    event.source_agent_name = "ped_1"
    event.sound_type = "greeting"
    event.label = "greeting"
    event.asset_id = "greeting"
    event.source_position.x = 0.0
    event.source_position.y = 0.0
    event.source_position.z = 0.0
    event.source_yaw = 0.0
    event.source_volume_db = 60.0
    event.duration.sec = 1
    event.loop = False
    return event


def _make_heard_sound_event():
    from task_generator_msgs.msg import HeardSoundEvent

    event = HeardSoundEvent()
    event.header.frame_id = "map"
    event.event_id = "heard_roundtrip_001"
    event.listener_id = "robot:robot1"
    event.source_agent_id = 1
    event.source_agent_name = "ped_1"
    event.sound_type = "greeting"
    event.label = "greeting"
    event.asset_id = "greeting"
    event.source_position.x = 0.0
    event.source_position.y = 0.0
    event.listener_position.x = 1.0
    event.listener_position.y = 0.0
    event.distance = 1.0
    event.source_volume_db = 60.0
    event.received_volume_db = 45.0
    event.hearing_threshold_db = 20.0
    event.direct_delay_sec = 0.0
    event.audible = True
    event.occluded = False
    return event


def test_sound_event_round_trips_to_heard_sound_event(rclpy_context):
    import rclpy
    from rclpy.parameter import Parameter
    from arena_people_msgs.msg import Pedestrians
    from task_generator.auditory.qos_profiles import transient_event_qos
    from task_generator.auditory.sound_propagation_node import SoundPropagationNode
    from task_generator_msgs.msg import HeardSoundEvent, SoundEvent

    suffix = uuid.uuid4().hex[:8]
    sound_topic = f"/test/{suffix}/human_sound_events"
    heard_topic = f"/test/{suffix}/heard_sound_events"
    peds_topic = f"/test/{suffix}/arena_peds"
    map_topic = f"/test/{suffix}/map"
    robot_fleet_topic = f"/test/{suffix}/state/robots"
    world_topic = f"/test/{suffix}/state/world"

    
    propagation = SoundPropagationNode(
        parameter_overrides=[
            Parameter("sound_events_topic", Parameter.Type.STRING, sound_topic),
            Parameter("heard_sound_events_topic", Parameter.Type.STRING, heard_topic),
            Parameter("arena_peds_topic", Parameter.Type.STRING, peds_topic),
            Parameter("map_topic", Parameter.Type.STRING, map_topic),
            Parameter("robot_fleet_topic", Parameter.Type.STRING, robot_fleet_topic),
            Parameter("world_topic", Parameter.Type.STRING, world_topic),
        ],
    )

    emitter = rclpy.create_node(f"sound_event_emitter_{suffix}")
    consumer = rclpy.create_node(f"heard_sound_consumer_{suffix}")

    received: list[HeardSoundEvent] = []

    publisher = emitter.create_publisher(
        SoundEvent,
        sound_topic,
        transient_event_qos(),
    )
    consumer.create_subscription(
        HeardSoundEvent,
        heard_topic,
        received.append,
        transient_event_qos(),
    )

    pedestrians = Pedestrians()
    pedestrians.pedestrians.append(_make_pedestrian(2, 3.0, 4.0))
    propagation._cb_peds(pedestrians)

    try:
        _spin_until(
            rclpy,
            [emitter, propagation, consumer],
            lambda: propagation.count_subscribers(heard_topic) > 0
            and emitter.count_subscribers(sound_topic) > 0,
        )

        publisher.publish(_make_sound_event())

        _spin_until(
            rclpy,
            [emitter, propagation, consumer],
            lambda: len(received) == 1,
        )

        heard = received[0]
        assert heard.event_id == "roundtrip_001"
        assert heard.listener_id == "agent:2"
        assert heard.sound_type == "greeting"
        assert heard.distance == pytest.approx(5.0)
        assert heard.occluded is False
        assert heard.audible is True
        assert heard.received_volume_db == pytest.approx(
            60.0 - 20.0 * math.log10(5.0),
            abs=1e-3,
        )
    finally:
        emitter.destroy_node()
        consumer.destroy_node()
        propagation.destroy_node()


def test_robot_hearing_node_republishes_target_robot_event(rclpy_context):
    import rclpy
    from rclpy.parameter import Parameter
    from task_generator.auditory.qos_profiles import transient_event_qos
    from task_generator.auditory.robot_hearing_node import RobotHearingNode
    from task_generator_msgs.msg import HeardSoundEvent
    from visualization_msgs.msg import Marker

    suffix = uuid.uuid4().hex[:8]
    heard_topic = f"/test/{suffix}/heard_sound_events"
    output_topic = f"/test/{suffix}/robot1/heard_sound"
    marker_topic = f"/test/{suffix}/robot1/heard_sound_marker"

    hearing = RobotHearingNode(
        parameter_overrides=[
            Parameter("robot_name", Parameter.Type.STRING, "robot1"),
            Parameter("heard_sound_events_topic", Parameter.Type.STRING, heard_topic),
            Parameter("output_topic", Parameter.Type.STRING, output_topic),
            Parameter("marker_topic", Parameter.Type.STRING, marker_topic),
            Parameter("honor_propagation_delay", Parameter.Type.BOOL, False),
            Parameter("min_snr_db", Parameter.Type.DOUBLE, 3.0),
        ],
    )

    emitter = rclpy.create_node(f"heard_event_emitter_{suffix}")
    consumer = rclpy.create_node(f"robot_heard_consumer_{suffix}")

    received: list[HeardSoundEvent] = []
    markers: list[Marker] = []

    publisher = emitter.create_publisher(
        HeardSoundEvent,
        heard_topic,
        transient_event_qos(),
    )
    consumer.create_subscription(
        HeardSoundEvent,
        output_topic,
        received.append,
        transient_event_qos(),
    )
    consumer.create_subscription(Marker, marker_topic, markers.append, 10)

    try:
        _spin_until(
            rclpy,
            [emitter, hearing, consumer],
            lambda: hearing.count_subscribers(heard_topic) > 0
            and emitter.count_subscribers(heard_topic) > 0,
        )

        publisher.publish(_make_heard_sound_event())

        _spin_until(
            rclpy,
            [emitter, hearing, consumer],
            lambda: len(received) == 1 and len(markers) == 1,
        )

        heard = received[0]
        assert heard.event_id == "heard_roundtrip_001"
        assert heard.listener_id == "robot:robot1"
        assert heard.sound_type == "greeting"
        assert markers[0].text == "Heard: voice"
    finally:
        emitter.destroy_node()
        consumer.destroy_node()
        hearing.destroy_node()
