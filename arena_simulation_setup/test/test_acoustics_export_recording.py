from __future__ import annotations

import csv
import struct

import numpy as np

from arena_simulation_setup.acoustics.export_recording import (
    PCM_F32LE,
    AudioBlock,
    assemble_audio,
    build_labels,
    clip_audio_chunks,
    select_map_snapshot,
    write_map_snapshot,
    occupancy_ray_labels,
    transform_robot_trajectory,
    write_csv,
)


def chunk(index: int, timestamp_ns: int, values: list[tuple[float, float]]) -> AudioBlock:
    payload = b"".join(struct.pack("<ff", *frame) for frame in values)
    return AudioBlock(
        topic="/arena/env_0/jackal/audio/headphones/stereo",
        timestamp_ns=timestamp_ns,
        first_sample_index=index,
        sample_rate=1000,
        channels=2,
        encoding=PCM_F32LE,
        stream_id="rendered",
        channel_names=("left", "right"),
        microphone_frame="robot/microphones",
        payload=payload,
    )


def test_assemble_audio_uses_sample_index_and_reports_gap():
    audio, timing, summary = assemble_audio([
        chunk(0, 1_000_000_000, [(0.1, -0.1), (0.2, -0.2)]),
        chunk(3, 1_003_000_000, [(0.3, -0.3)]),
    ])

    assert audio.shape == (4, 2)
    np.testing.assert_array_equal(audio[2], [0.0, 0.0])
    assert timing[1]["gap_frames_before"] == 1
    assert timing[1]["timestamp_error_ns"] == 0
    assert summary["gap_frames"] == 1


def test_clip_audio_chunks_uses_half_open_simulation_time_window():
    source = chunk(0, 1_000_000_000, [(0.0, 0.0), (0.1, -0.1), (0.2, -0.2), (0.3, -0.3)])

    clipped = clip_audio_chunks([source], 1_001_000_000, 1_003_000_000)

    assert len(clipped) == 1
    assert clipped[0].timestamp_ns == 1_001_000_000
    assert clipped[0].first_sample_index == 1
    np.testing.assert_allclose(np.frombuffer(clipped[0].payload, dtype="<f4").reshape((-1, 2)), [[0.1, -0.1], [0.2, -0.2]])


def test_map_snapshot_selects_latest_preceding_grid_and_is_self_describing(tmp_path):
    rows = [
        {"timestamp_ns": 10, "topic": "/arena/env_0/map", "frame_id": "map", "resolution": 0.1, "width": 2, "height": 1, "origin_x": 1.0, "origin_y": 2.0, "origin_z": 0.0, "origin_yaw": 0.0, "data": np.asarray([[0, 100]], dtype=np.int8)},
        {"timestamp_ns": 20, "topic": "/arena/env_0/map", "frame_id": "map", "resolution": 0.1, "width": 2, "height": 1, "origin_x": 3.0, "origin_y": 4.0, "origin_z": 0.0, "origin_yaw": 0.0, "data": np.asarray([[-1, 0]], dtype=np.int8)},
    ]

    selected = select_map_snapshot(rows, 15)
    metadata = write_map_snapshot(tmp_path / "map.npz", selected)

    assert selected["timestamp_ns"] == 10
    assert len(metadata["sha256"]) == 64
    with np.load(tmp_path / "map.npz") as saved:
        np.testing.assert_array_equal(saved["occupancy"], [[0, 100]])
        np.testing.assert_allclose(saved["origin"], [1.0, 2.0, 0.0, 0.0])
        assert saved["frame_id"].item() == "map"


def test_occupancy_ray_labels_detects_occlusion():
    snapshot = {
        "resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0, "origin_yaw": 0.0,
        "width": 4, "height": 2,
        "data": np.asarray([[0, 0, 100, 0], [0, 0, 0, 0]], dtype=np.int8),
    }

    blocked = occupancy_ray_labels(snapshot, (0.5, 0.5), (3.5, 0.5))
    clear = occupancy_ray_labels(snapshot, (0.5, 1.5), (3.5, 1.5))

    assert blocked["line_of_sight"] is False
    assert blocked["ray_occupied_cell_count"] > 0
    assert clear["line_of_sight"] is True


def test_robot_odometry_is_transformed_into_map_frame():
    odom = [{
        "timestamp_ns": 100, "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0,
        "vx": 1.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0,
        "frame_id": "odom", "child_frame_id": "base_link", "topic": "/env/odom",
    }]
    transforms = {("map", "odom"): [{
        "timestamp_ns": 0, "x": 10.0, "y": 2.0, "z": 0.0,
        "yaw": np.pi / 2, "static": True,
    }]}

    aligned, provenance = transform_robot_trajectory(odom, transforms, "map", 100_000_000)

    np.testing.assert_allclose([aligned[0]["x"], aligned[0]["y"]], [10.0, 3.0])
    np.testing.assert_allclose([aligned[0]["vx"], aligned[0]["vy"]], [0.0, 1.0], atol=1e-7)
    assert aligned[0]["frame_id"] == "map"
    assert provenance["transform"] == "map->odom"


def test_labels_join_audio_to_interpolated_robot_and_pedestrian_pose():
    audio = np.ones((20, 2), dtype=np.float32) * 0.5
    summary = {
        "sample_rate": 1000,
        "first_timestamp_ns": 1_000_000_000,
        "first_sample_index": 0,
    }
    robot = [
        {"timestamp_ns": 1_000_000_000, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0},
        {"timestamp_ns": 1_020_000_000, "x": 2.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 1.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0},
    ]
    peds = {"ped_1": [
        {"timestamp_ns": 1_000_000_000, "pedestrian_id": 1, "pedestrian_name": "ped_1", "x": 0.0, "y": 2.0, "z": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "model_uri": "adult"},
        {"timestamp_ns": 1_020_000_000, "pedestrian_id": 1, "pedestrian_name": "ped_1", "x": 2.0, "y": 2.0, "z": 0.0, "yaw": 0.0, "vx": 1.0, "vy": 0.0, "vz": 0.0, "model_uri": "adult"},
    ]}

    labels = build_labels(audio, summary, audio, summary, robot, peds, frame_ms=10, max_pose_gap_ms=20)

    assert len(labels) == 2
    assert labels[1]["robot_x"] == 1.0
    assert labels[1]["pedestrian_x"] == 1.0
    assert labels[1]["range_m"] == 2.0
    assert labels[1]["bearing_robot_rad"] == np.pi / 2
    assert labels[1]["rendered_ch0_rms"] == 0.5
    assert labels[1]["recording_sample_offset"] == 10
    assert labels[1]["recording_time_seconds"] == 0.01


def test_metadata_csv_preserves_synchronized_scalar_labels(tmp_path):
    path = tmp_path / "0001_meta.csv"
    write_csv(path, [{
        "execution_index": 1,
        "scenario": "hearing_case",
        "timestamp_ns": 1_010_000_000,
        "recording_sample_offset": 480,
        "robot_x": 1.25,
        "pedestrian_x": 2.5,
        "line_of_sight": True,
    }])

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows == [{
        "execution_index": "1",
        "scenario": "hearing_case",
        "timestamp_ns": "1010000000",
        "recording_sample_offset": "480",
        "robot_x": "1.25",
        "pedestrian_x": "2.5",
        "line_of_sight": "True",
    }]
