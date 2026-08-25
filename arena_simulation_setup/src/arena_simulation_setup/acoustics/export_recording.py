"""Export synchronized Arena acoustic recordings from MCAP.

The MCAP is the lossless source of truth for the float32 AudioFrame streams.
The rendered binaural stream is also exported to FLAC for compact
training/inspection workflows.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
import subprocess
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Protocol

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import yaml

PCM_S16LE = 1
PCM_F32LE = 2


class StampLike(Protocol):
    """Structural type shared by ROS Time message instances."""

    sec: int
    nanosec: int


class QuaternionLike(Protocol):
    """Structural type shared by geometry message quaternions."""

    x: float
    y: float
    z: float
    w: float


class HeaderLike(Protocol):
    """Minimum ROS Header interface needed by the exporter."""

    stamp: StampLike


class HeaderWithFrameLike(HeaderLike, Protocol):
    """ROS Header which also supplies the coordinate-frame identifier."""

    frame_id: str


class HeaderMessageLike(Protocol):
    """Message which may expose a ROS Header."""

    header: HeaderLike | None


class Pose2DLike(Protocol):
    x: float
    y: float
    theta: float


class Vector3Like(Protocol):
    x: float
    y: float
    z: float


class AgentStateLike(Protocol):
    agent_id: int
    kind: int
    pose: Pose2DLike
    velocity: Vector3Like
    desired_velocity: float
    radius: float
    agent_type: str
    policy: str


class AgentStatesLike(Protocol):
    header: HeaderWithFrameLike
    agents: Iterable[AgentStateLike]


@dataclass(frozen=True)
class AudioBlock:
    topic: str
    timestamp_ns: int
    first_sample_index: int
    sample_rate: int
    channels: int
    encoding: int
    stream_id: str
    channel_names: tuple[str, ...]
    microphone_frame: str
    payload: bytes
    microphone_positions: tuple[tuple[float, float, float], ...] = ()
    microphone_yaw_rad: tuple[float, ...] = ()

    @property
    def frame_count(self) -> int:
        width = {PCM_S16LE: 2, PCM_F32LE: 4}.get(self.encoding)
        if not width or self.channels <= 0 or len(self.payload) % (width * self.channels):
            raise ValueError(f"{self.topic}: malformed PCM payload")
        return len(self.payload) // (width * self.channels)


def stamp_ns(stamp: StampLike) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_yaw(q: QuaternionLike) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pcm_as_float32(chunk: AudioBlock) -> np.ndarray:
    if chunk.encoding == PCM_S16LE:
        values = np.frombuffer(chunk.payload, dtype="<i2").astype(np.float32) / 32768.0
    elif chunk.encoding == PCM_F32LE:
        values = np.frombuffer(chunk.payload, dtype="<f4").astype(np.float32, copy=False)
    else:
        raise ValueError(f"{chunk.topic}: unsupported encoding {chunk.encoding}")
    return values.reshape((-1, chunk.channels))


def clip_audio_chunks(chunks: Iterable[AudioBlock], start_ns: int, end_ns: int | None) -> list[AudioBlock]:
    """Clip chunks to a half-open simulation-time interval at sample precision."""
    clipped: list[AudioBlock] = []
    for chunk in chunks:
        width = {PCM_S16LE: 2, PCM_F32LE: 4}.get(chunk.encoding)
        if width is None:
            raise ValueError(f"{chunk.topic}: unsupported encoding {chunk.encoding}")
        frames = chunk.frame_count
        first = max(0, math.ceil((start_ns - chunk.timestamp_ns) * chunk.sample_rate / 1_000_000_000))
        last = frames
        if end_ns is not None:
            last = min(last, math.ceil((end_ns - chunk.timestamp_ns) * chunk.sample_rate / 1_000_000_000))
        first, last = min(frames, first), max(0, last)
        if first >= last:
            continue
        frame_bytes = width * chunk.channels
        clipped.append(AudioBlock(
            topic=chunk.topic,
            timestamp_ns=chunk.timestamp_ns + round(first * 1_000_000_000 / chunk.sample_rate),
            first_sample_index=chunk.first_sample_index + first,
            sample_rate=chunk.sample_rate,
            channels=chunk.channels,
            encoding=chunk.encoding,
            stream_id=chunk.stream_id,
            channel_names=chunk.channel_names,
            microphone_frame=chunk.microphone_frame,
            payload=chunk.payload[first * frame_bytes:last * frame_bytes],
            microphone_positions=chunk.microphone_positions,
            microphone_yaw_rad=chunk.microphone_yaw_rad,
        ))
    return clipped


def assemble_audio(chunks: Iterable[AudioBlock]) -> tuple[np.ndarray, list[dict[str, Any]], dict[str, Any]]:
    ordered = sorted(chunks, key=lambda item: (item.first_sample_index, item.timestamp_ns))
    if not ordered:
        raise ValueError("audio stream contains no chunks")
    first = ordered[0]
    rate, channels, encoding = first.sample_rate, first.channels, first.encoding
    if rate <= 0 or channels <= 0:
        raise ValueError("invalid audio format")

    pieces: list[np.ndarray] = []
    timing: list[dict[str, Any]] = []
    expected_index = first.first_sample_index
    origin_ns = first.timestamp_ns
    origin_index = first.first_sample_index
    gap_total = 0
    max_timestamp_error_ns = 0
    for chunk in ordered:
        if (chunk.sample_rate, chunk.channels, chunk.encoding) != (rate, channels, encoding):
            raise ValueError(f"{chunk.topic}: format changes are not allowed within a stream")
        delta = chunk.first_sample_index - expected_index
        if delta < 0:
            raise ValueError(f"{chunk.topic}: overlapping/out-of-order chunk at sample {chunk.first_sample_index}")
        if delta:
            pieces.append(np.zeros((delta, channels), dtype=np.float32))
            gap_total += delta
        expected_timestamp_ns = origin_ns + round(
            (chunk.first_sample_index - origin_index) * 1_000_000_000 / rate
        )
        timestamp_error_ns = chunk.timestamp_ns - expected_timestamp_ns
        max_timestamp_error_ns = max(max_timestamp_error_ns, abs(timestamp_error_ns))
        frames = pcm_as_float32(chunk)
        output_sample_index = sum(piece.shape[0] for piece in pieces)
        pieces.append(frames)
        timing.append({
            "topic": chunk.topic,
            "timestamp_ns": chunk.timestamp_ns,
            "expected_timestamp_ns": expected_timestamp_ns,
            "timestamp_error_ns": timestamp_error_ns,
            "first_sample_index": chunk.first_sample_index,
            "output_sample_index": output_sample_index,
            "frame_count": chunk.frame_count,
            "gap_frames_before": delta,
            "sample_rate": rate,
            "channels": channels,
            "encoding": encoding,
            "stream_id": chunk.stream_id,
            "microphone_frame": chunk.microphone_frame,
            "channel_names": list(chunk.channel_names),
            "microphone_positions": list(chunk.microphone_positions),
            "microphone_yaw_rad": list(chunk.microphone_yaw_rad),
        })
        expected_index = chunk.first_sample_index + chunk.frame_count

    audio = np.concatenate(pieces, axis=0)
    summary = {
        "topic": first.topic,
        "stream_id": first.stream_id,
        "sample_rate": rate,
        "channels": channels,
        "encoding": encoding,
        "channel_names": list(first.channel_names),
        "microphone_frame": first.microphone_frame,
        "microphone_positions": list(first.microphone_positions),
        "microphone_yaw_rad": list(first.microphone_yaw_rad),
        "first_timestamp_ns": origin_ns,
        "first_sample_index": origin_index,
        "frames_with_gap_fill": int(audio.shape[0]),
        "recorded_frames": int(audio.shape[0] - gap_total),
        "gap_frames": gap_total,
        "max_timestamp_error_ns": max_timestamp_error_ns,
        "duration_seconds": float(audio.shape[0] / rate),
    }
    return audio, timing, summary


def _header_time_or_log_time(msg: HeaderMessageLike, log_time: int) -> int:
    header = msg.header
    if header is None:
        return int(log_time)
    return stamp_ns(header.stamp)


def _agent_states_pedestrians(
    message: AgentStatesLike,
    topic: str,
    log_time: int,
) -> dict[str, list[dict[str, Any]]]:
    """Convert HumanSim's batched AgentStates stream into pedestrian samples.

    AgentState.pose uses the world occupancy-map coordinate system.  The
    HumanSim publisher historically leaves ``header.frame_id`` empty, so map is
    its documented implicit frame in that case.  Robots can be published in
    the same batch and must not become acoustic sound-source labels.
    """
    timestamp_ns = _header_time_or_log_time(message, log_time)
    frame_id = str(message.header.frame_id).strip() or "map"
    pedestrians: dict[str, list[dict[str, Any]]] = {}
    for agent in message.agents:
        # AgentState.KIND_HUMAN is 0 and KIND_ROBOT is 1.  Retain the explicit
        # comparison rather than assuming every state in the batch is a human.
        if int(agent.kind) != 0:
            continue
        agent_id = int(agent.agent_id)
        agent_type = str(agent.agent_type)
        policy = str(agent.policy)
        key = f"agent_{agent_id}"
        pedestrians.setdefault(key, []).append({
            "timestamp_ns": timestamp_ns,
            "pedestrian_id": agent_id,
            "pedestrian_name": key,
            "x": float(agent.pose.x), "y": float(agent.pose.y), "z": 0.0,
            "yaw": float(agent.pose.theta),
            "vx": float(agent.velocity.x), "vy": float(agent.velocity.y),
            "vz": float(agent.velocity.z),
            "animation_state": None,
            "model_uri": "",
            "radius": float(agent.radius),
            "desired_velocity": float(agent.desired_velocity),
            "agent_type": agent_type,
            "policy": policy,
            "state_source": "agent_states",
            "topic": topic,
            "frame_id": frame_id,
        })
    return pedestrians


def read_mcap(path: Path) -> dict[str, Any]:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    audio: dict[str, list[AudioBlock]] = {"raw": [], "rendered": []}
    odom: dict[str, list[dict[str, Any]]] = {}
    arena_pedestrians: dict[str, list[dict[str, Any]]] = {}
    agent_state_pedestrians: dict[str, list[dict[str, Any]]] = {}
    maps: dict[str, list[dict[str, Any]]] = {"map": [], "door_mask": []}
    transforms: dict[tuple[str, str], list[dict[str, Any]]] = {}
    clocks: list[int] = []
    episode_events: list[dict[str, Any]] = []
    topic_types: dict[str, str] = {}

    with path.open("rb") as source:
        reader = make_reader(source, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(log_time_order=True):
            topic = "/" + channel.topic.strip("/")
            topic_types[topic] = schema.name
            audio_role = (
                "raw" if topic.endswith("/audio/raw_array")
                else "rendered" if topic.endswith("/audio/headphones/stereo")
                else None
            )
            if audio_role is not None:
                if str(ros_msg.encoding) != "32FC1" or not bool(ros_msg.interleaved):
                    raise ValueError(
                        f"{topic}: expected interleaved 32FC1 AudioFrame, got "
                        f"encoding={ros_msg.encoding!r} interleaved={ros_msg.interleaved!r}"
                    )
                channels = int(ros_msg.channel_count)
                frames = int(ros_msg.frame_count)
                values = np.asarray(ros_msg.data, dtype="<f4")
                if channels <= 0 or frames <= 0 or values.size != channels * frames:
                    raise ValueError(f"{topic}: malformed AudioFrame dimensions")
                audio[audio_role].append(AudioBlock(
                    topic=topic,
                    timestamp_ns=stamp_ns(ros_msg.header.stamp),
                    first_sample_index=0,
                    sample_rate=int(ros_msg.sample_rate),
                    channels=channels,
                    encoding=PCM_F32LE,
                    stream_id=topic,
                    channel_names=tuple(str(item) for item in ros_msg.channel_names),
                    microphone_frame=str(ros_msg.header.frame_id),
                    payload=values.tobytes(),
                    microphone_positions=tuple(
                        (float(point.x), float(point.y), float(point.z))
                        for point in ros_msg.microphone_positions
                    ),
                    microphone_yaw_rad=tuple(float(value) for value in ros_msg.microphone_yaw_rad),
                ))
            elif topic == "/clock":
                clocks.append(stamp_ns(ros_msg.clock))
            elif topic.endswith("/door_mask") or (topic.endswith("/map") and "/costmap" not in topic):
                role = "door_mask" if topic.endswith("/door_mask") else "map"
                info = ros_msg.info
                map_data = np.asarray(ros_msg.data, dtype=np.int8).reshape((int(info.height), int(info.width)))
                maps[role].append({
                    "timestamp_ns": _header_time_or_log_time(ros_msg, message.log_time),
                    "topic": topic,
                    "frame_id": str(ros_msg.header.frame_id),
                    "resolution": float(info.resolution),
                    "width": int(info.width),
                    "height": int(info.height),
                    "origin_x": float(info.origin.position.x),
                    "origin_y": float(info.origin.position.y),
                    "origin_z": float(info.origin.position.z),
                    "origin_yaw": quaternion_yaw(info.origin.orientation),
                    "data": map_data,
                })
            elif topic in ("/tf", "/tf_static"):
                for transform in ros_msg.transforms:
                    transform_time = stamp_ns(transform.header.stamp)
                    if transform_time == 0:
                        transform_time = int(message.log_time)
                    parent = str(transform.header.frame_id).strip("/")
                    child = str(transform.child_frame_id).strip("/")
                    value = transform.transform
                    transforms.setdefault((parent, child), []).append({
                        "timestamp_ns": transform_time,
                        "x": float(value.translation.x),
                        "y": float(value.translation.y),
                        "z": float(value.translation.z),
                        "yaw": quaternion_yaw(value.rotation),
                        "static": topic == "/tf_static",
                    })
            elif topic.endswith("/odom"):
                ts = _header_time_or_log_time(ros_msg, message.log_time)
                pose, twist = ros_msg.pose.pose, ros_msg.twist.twist
                odom.setdefault(topic, []).append({
                    "timestamp_ns": ts,
                    "x": float(pose.position.x), "y": float(pose.position.y),
                    "z": float(pose.position.z), "yaw": quaternion_yaw(pose.orientation),
                    "vx": float(twist.linear.x), "vy": float(twist.linear.y),
                    "vz": float(twist.linear.z), "yaw_rate": float(twist.angular.z),
                    "frame_id": str(ros_msg.header.frame_id),
                    "child_frame_id": str(ros_msg.child_frame_id),
                    "topic": topic,
                })
            elif topic.endswith("/arena_peds") and hasattr(ros_msg, "pedestrians"):
                ts = _header_time_or_log_time(ros_msg, message.log_time)
                for ped in ros_msg.pedestrians:
                    key = str(ped.name) or str(ped.id)
                    arena_pedestrians.setdefault(key, []).append({
                        "timestamp_ns": ts,
                        "pedestrian_id": int(ped.id), "pedestrian_name": str(ped.name),
                        "x": float(ped.pose.position.x), "y": float(ped.pose.position.y),
                        "z": float(ped.pose.position.z), "yaw": quaternion_yaw(ped.pose.orientation),
                        "vx": float(ped.twist.linear.x), "vy": float(ped.twist.linear.y),
                        "vz": float(ped.twist.linear.z),
                        "animation_state": int(ped.animation_state),
                        "model_uri": str(ped.model_uri),
                        "radius": None, "desired_velocity": None,
                        "agent_type": "", "policy": "", "state_source": "arena_peds",
                        "topic": topic,
                        "frame_id": str(ros_msg.header.frame_id),
                    })
            elif topic.endswith("/agent_states") and hasattr(ros_msg, "agents"):
                decoded = _agent_states_pedestrians(ros_msg, topic, message.log_time)
                for key, rows in decoded.items():
                    agent_state_pedestrians.setdefault(key, []).extend(rows)
            elif topic.endswith("/state/episode"):
                episode_events.append({
                    "timestamp_ns": int(message.log_time),
                    "start_time_ns": stamp_ns(ros_msg.start_time),
                    "episode_id": str(ros_msg.episode_id),
                    "outcome_state": int(ros_msg.outcome_state),
                    "outcome_info": str(ros_msg.outcome_info),
                    "world": str(ros_msg.world),
                })
    # AudioFrame does not carry a sample counter. Its publisher guarantees that
    # header.stamp is the first sample's simulation time, so derive the stable
    # frame index from that clock. Timestamp gaps therefore become sample gaps.
    for role, chunks in audio.items():
        if not chunks:
            continue
        ordered = sorted(chunks, key=lambda item: item.timestamp_ns)
        origin_ns = ordered[0].timestamp_ns
        rate = ordered[0].sample_rate
        audio[role] = [
            replace(
                chunk,
                first_sample_index=round((chunk.timestamp_ns - origin_ns) * rate / 1_000_000_000),
            )
            for chunk in ordered
        ]

    # arena_peds is the task-generator's richer pedestrian projection.  Older
    # recordings may contain only HumanSim's agent_states stream, which is an
    # equally time-stamped source of human poses; use it as a strict fallback
    # so a bag recording both topics does not generate duplicate labels.
    pedestrians = arena_pedestrians or agent_state_pedestrians
    return {
        "audio": audio, "odom": odom, "pedestrians": pedestrians,
        "pedestrian_state_source": "arena_peds" if arena_pedestrians else (
            "agent_states" if agent_state_pedestrians else None
        ),
        "clock": clocks, "episodes": episode_events, "topic_types": topic_types,
        "maps": maps,
        "transforms": transforms,
    }


def _choose_odom(odom: dict[str, list[dict[str, Any]]], requested: str | None) -> tuple[str, list[dict[str, Any]]]:
    if requested:
        key = "/" + requested.strip("/")
        if key not in odom:
            raise ValueError(f"requested odometry topic {key!r} is absent; found {sorted(odom)}")
        return key, odom[key]
    preferred = {key: rows for key, rows in odom.items() if "velocity_controller" not in key}
    candidates = preferred or odom
    if len(candidates) != 1:
        raise ValueError(f"cannot infer one robot odometry topic; pass --robot-odom-topic from {sorted(candidates)}")
    return next(iter(candidates.items()))


def _interp(rows: list[dict[str, Any]], timestamp_ns: int, max_gap_ns: int) -> dict[str, Any] | None:
    if not rows:
        return None
    times = np.fromiter((row["timestamp_ns"] for row in rows), dtype=np.int64)
    right = int(np.searchsorted(times, timestamp_ns, side="left"))
    if right == 0:
        return rows[0] if abs(int(times[0]) - timestamp_ns) <= max_gap_ns else None
    if right == len(rows):
        return rows[-1] if abs(timestamp_ns - int(times[-1])) <= max_gap_ns else None
    left = right - 1
    if timestamp_ns - int(times[left]) > max_gap_ns or int(times[right]) - timestamp_ns > max_gap_ns:
        return None
    denominator = int(times[right]) - int(times[left])
    if denominator == 0:
        result = dict(rows[right])
        result["timestamp_ns"] = timestamp_ns
        return result
    fraction = (timestamp_ns - int(times[left])) / denominator
    result = dict(rows[left])
    for field in ("x", "y", "z", "vx", "vy", "vz", "yaw_rate"):
        if field in rows[left] and field in rows[right]:
            result[field] = float(rows[left][field] + fraction * (rows[right][field] - rows[left][field]))
    if "yaw" in rows[left]:
        delta = math.atan2(math.sin(rows[right]["yaw"] - rows[left]["yaw"]), math.cos(rows[right]["yaw"] - rows[left]["yaw"]))
        result["yaw"] = math.atan2(math.sin(rows[left]["yaw"] + fraction * delta), math.cos(rows[left]["yaw"] + fraction * delta))
    result["timestamp_ns"] = timestamp_ns
    return result


def _same_frame(left: str, right: str) -> bool:
    left, right = left.strip("/"), right.strip("/")
    return left == right or left.endswith("/" + right) or right.endswith("/" + left)


def transform_robot_trajectory(
    rows: list[dict[str, Any]],
    transforms: dict[tuple[str, str], list[dict[str, Any]]],
    target_frame: str,
    max_gap_ns: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Express odometry poses/velocities in the same map frame as pedestrians."""
    if not rows:
        raise ValueError("robot odometry is empty")
    source_frames = {str(row["frame_id"]).strip("/") for row in rows}
    if len(source_frames) != 1:
        raise ValueError(f"robot odometry changes frame_id: {sorted(source_frames)}")
    source_frame = next(iter(source_frames))
    target_frame = target_frame.strip("/")
    if _same_frame(source_frame, target_frame):
        return rows, {"source_frame": source_frame, "target_frame": target_frame, "transform": "identity"}

    candidates = [
        (key, values) for key, values in transforms.items()
        if _same_frame(key[0], target_frame) and _same_frame(key[1], source_frame)
    ]
    if len(candidates) != 1:
        raise ValueError(
            f"need exactly one TF transform {target_frame!r}->{source_frame!r} to align robot and pedestrians; "
            f"found {[key for key, _ in candidates]}"
        )
    (parent, child), tf_rows = candidates[0]
    tf_rows = sorted(tf_rows, key=lambda row: row["timestamp_ns"])
    is_static = all(row["static"] for row in tf_rows)
    aligned: list[dict[str, Any]] = []
    for row in rows:
        transform = tf_rows[-1] if is_static else _interp(tf_rows, row["timestamp_ns"], max_gap_ns)
        if transform is None:
            continue
        cosine, sine = math.cos(transform["yaw"]), math.sin(transform["yaw"])
        aligned.append({
            **row,
            "x": transform["x"] + cosine * row["x"] - sine * row["y"],
            "y": transform["y"] + sine * row["x"] + cosine * row["y"],
            "z": transform["z"] + row["z"],
            "yaw": math.atan2(math.sin(transform["yaw"] + row["yaw"]), math.cos(transform["yaw"] + row["yaw"])),
            "vx": cosine * row["vx"] - sine * row["vy"],
            "vy": sine * row["vx"] + cosine * row["vy"],
            "source_frame_id": row["frame_id"],
            "frame_id": target_frame,
        })
    if not aligned:
        raise ValueError(f"TF {parent}->{child} has no samples close enough to robot odometry")
    return aligned, {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "transform": f"{parent}->{child}",
        "static": is_static,
    }


def _audio_features(audio: np.ndarray, start: int, stop: int, prefix: str) -> dict[str, float]:
    window = audio[start:stop]
    result: dict[str, float] = {}
    for channel in range(window.shape[1]):
        values = window[:, channel]
        result[f"{prefix}_ch{channel}_rms"] = float(np.sqrt(np.mean(values * values)))
        result[f"{prefix}_ch{channel}_peak"] = float(np.max(np.abs(values)))
    return result


def occupancy_ray_labels(
    snapshot: dict[str, Any], start_xy: tuple[float, float], end_xy: tuple[float, float]
) -> dict[str, Any]:
    """Trace a world-space source/listener ray through the recorded OccupancyGrid."""
    resolution = float(snapshot["resolution"])
    if resolution <= 0:
        raise ValueError("occupancy-map resolution must be positive")
    origin_yaw = float(snapshot["origin_yaw"])
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)

    def grid_xy(point: tuple[float, float]) -> tuple[float, float]:
        dx = point[0] - float(snapshot["origin_x"])
        dy = point[1] - float(snapshot["origin_y"])
        return ((cosine * dx + sine * dy) / resolution, (-sine * dx + cosine * dy) / resolution)

    start_grid, end_grid = grid_xy(start_xy), grid_xy(end_xy)
    steps = max(1, math.ceil(math.dist(start_grid, end_grid) * 2.0))
    cols = np.floor(np.linspace(start_grid[0], end_grid[0], steps + 1)).astype(np.int64)
    rows = np.floor(np.linspace(start_grid[1], end_grid[1], steps + 1)).astype(np.int64)
    valid = (cols >= 0) & (rows >= 0) & (cols < int(snapshot["width"])) & (rows < int(snapshot["height"]))
    if not np.all(valid):
        return {
            "line_of_sight": False,
            "ray_out_of_map": True,
            "ray_occupied_cell_count": 0,
            "ray_unknown_fraction": 1.0,
        }
    cells = np.unique(np.column_stack((rows, cols)), axis=0)
    values = np.asarray(snapshot["data"], dtype=np.int8)[cells[:, 0], cells[:, 1]]
    occupied = int(np.count_nonzero(values >= 50))
    unknown = int(np.count_nonzero(values < 0))
    return {
        "line_of_sight": occupied == 0 and unknown == 0,
        "ray_out_of_map": False,
        "ray_occupied_cell_count": occupied,
        "ray_unknown_fraction": float(unknown / len(values)),
    }


def build_labels(
    rendered: np.ndarray,
    rendered_summary: dict[str, Any],
    raw: np.ndarray,
    raw_summary: dict[str, Any],
    robot_rows: list[dict[str, Any]],
    pedestrians: dict[str, list[dict[str, Any]]],
    *,
    frame_ms: float,
    max_pose_gap_ms: float,
    emit_robot_only: bool = False,
    context: dict[str, Any] | None = None,
    occupancy_map: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    rate = int(rendered_summary["sample_rate"])
    hop = max(1, round(rate * frame_ms / 1000.0))
    origin_ns = int(rendered_summary["first_timestamp_ns"])
    max_gap_ns = round(max_pose_gap_ms * 1_000_000)
    rows: list[dict[str, Any]] = []
    for start in range(0, rendered.shape[0], hop):
        stop = min(start + hop, rendered.shape[0])
        timestamp_ns = origin_ns + round(start * 1_000_000_000 / rate)
        robot = _interp(robot_rows, timestamp_ns, max_gap_ns)
        if robot is None:
            continue
        common: dict[str, Any] = {
            **(context or {}),
            "timestamp_ns": timestamp_ns,
            "timestamp_seconds": timestamp_ns / 1_000_000_000,
            "recording_time_seconds": start / rate,
            "recording_sample_offset": start,
            "audio_sample_index": int(rendered_summary["first_sample_index"]) + start,
            "audio_frame_count": stop - start,
            "audio_sample_rate": rate,
            "microphone_frame": rendered_summary.get("microphone_frame"),
            "raw_channel_names": raw_summary.get("channel_names"),
            "raw_microphone_positions": raw_summary.get("microphone_positions"),
            "raw_microphone_yaw_rad": raw_summary.get("microphone_yaw_rad"),
            "robot_x": robot["x"], "robot_y": robot["y"], "robot_z": robot["z"],
            "robot_yaw": robot["yaw"], "robot_vx": robot["vx"], "robot_vy": robot["vy"],
            "robot_yaw_rate": robot["yaw_rate"],
            **_audio_features(rendered, start, stop, "rendered"),
        }
        if raw_summary["sample_rate"] == rate:
            raw_offset = round((timestamp_ns - raw_summary["first_timestamp_ns"]) * rate / 1_000_000_000)
            if 0 <= raw_offset < raw.shape[0]:
                common.update(_audio_features(raw, raw_offset, min(raw_offset + hop, raw.shape[0]), "raw"))
        if not pedestrians and emit_robot_only:
            rows.append({**common, "pedestrian_present": False})
            continue
        for ped_key, ped_rows in pedestrians.items():
            ped = _interp(ped_rows, timestamp_ns, max_gap_ns)
            if ped is None:
                continue
            dx, dy = ped["x"] - robot["x"], ped["y"] - robot["y"]
            dz = ped["z"] - robot["z"]
            relative_x_robot = math.cos(robot["yaw"]) * dx + math.sin(robot["yaw"]) * dy
            relative_y_robot = -math.sin(robot["yaw"]) * dx + math.cos(robot["yaw"]) * dy
            bearing = math.atan2(math.sin(math.atan2(dy, dx) - robot["yaw"]), math.cos(math.atan2(dy, dx) - robot["yaw"]))
            distance = math.hypot(dx, dy)
            radial_velocity = ((ped["vx"] - robot["vx"]) * dx + (ped["vy"] - robot["vy"]) * dy) / distance if distance else 0.0
            ray_labels = occupancy_ray_labels(
                occupancy_map, (robot["x"], robot["y"]), (ped["x"], ped["y"])
            ) if occupancy_map is not None else {}
            rows.append({
                **common,
                "pedestrian_key": ped_key,
                "pedestrian_id": ped["pedestrian_id"], "pedestrian_name": ped["pedestrian_name"],
                "pedestrian_x": ped["x"], "pedestrian_y": ped["y"], "pedestrian_z": ped["z"],
                "pedestrian_yaw": ped["yaw"], "pedestrian_vx": ped["vx"], "pedestrian_vy": ped["vy"],
                "pedestrian_model_uri": ped["model_uri"],
                "pedestrian_radius_m": ped.get("radius"),
                "pedestrian_desired_velocity_mps": ped.get("desired_velocity"),
                "pedestrian_agent_type": ped.get("agent_type", ""),
                "pedestrian_policy": ped.get("policy", ""),
                "pedestrian_state_source": ped.get("state_source", "arena_peds"),
                "relative_x_world": dx, "relative_y_world": dy,
                "relative_z_world": dz,
                "relative_x_robot": relative_x_robot, "relative_y_robot": relative_y_robot,
                "range_m": distance, "range_3d_m": math.sqrt(dx * dx + dy * dy + dz * dz),
                "bearing_robot_rad": bearing, "elevation_robot_rad": math.atan2(dz, distance),
                "radial_velocity_mps": radial_velocity,
                **ray_labels,
            })
    if not rows:
        raise ValueError("no labels could be aligned; check odometry/pedestrian topics and --max-pose-gap-ms")
    return rows


def write_parquet(path: Path, rows: list[dict[str, Any]]) -> None:
    if rows:
        pq.write_table(pa.Table.from_pylist(rows), path, compression="zstd")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    """Write synchronized training labels in a dependency-free tabular form."""
    if not rows:
        raise ValueError("cannot write an empty metadata CSV")
    fieldnames = list(rows[0])
    for row in rows[1:]:
        fieldnames.extend(key for key in row if key not in fieldnames)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: json.dumps(value, separators=(",", ":"))
                if isinstance(value, (dict, list, tuple)) else value
                for key, value in row.items()
            })


def write_flac(path: Path, audio: np.ndarray, sample_rate: int) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg is required for FLAC export (Ubuntu: sudo apt install ffmpeg)")
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-f", "f32le", "-ar", str(sample_rate), "-ac", str(audio.shape[1]), "-i", "pipe:0",
        "-c:a", "flac", "-compression_level", "8", str(path),
    ]
    subprocess.run(command, input=np.asarray(audio, dtype="<f4").tobytes(), check=True)


def select_map_snapshot(rows: list[dict[str, Any]], timestamp_ns: int) -> dict[str, Any] | None:
    if not rows:
        return None
    ordered = sorted(rows, key=lambda row: row["timestamp_ns"])
    preceding = [row for row in ordered if row["timestamp_ns"] <= timestamp_ns]
    return preceding[-1] if preceding else ordered[0]


def write_map_snapshot(path: Path, snapshot: dict[str, Any]) -> dict[str, Any]:
    data = np.asarray(snapshot["data"], dtype=np.int8)
    metadata = {key: value for key, value in snapshot.items() if key != "data"}
    digest = hashlib.sha256(
        json.dumps(metadata, sort_keys=True, separators=(",", ":")).encode("utf-8") + data.tobytes()
    ).hexdigest()
    np.savez_compressed(
        path,
        occupancy=data,
        timestamp_ns=np.int64(metadata["timestamp_ns"]),
        resolution=np.float64(metadata["resolution"]),
        origin=np.asarray(
            [metadata["origin_x"], metadata["origin_y"], metadata["origin_z"], metadata["origin_yaw"]],
            dtype=np.float64,
        ),
        frame_id=np.asarray(metadata["frame_id"]),
        topic=np.asarray(metadata["topic"]),
    )
    return {**metadata, "sha256": digest, "file": path.name}


def resolve_mcap(path: Path) -> tuple[Path, Path]:
    path = path.resolve()
    if path.is_file():
        run_dir = path.parent.parent if path.parent.name == "recording" else path.parent
        return path, run_dir
    candidates = (
        sorted(path.glob("recording/*.mcap"))
        + sorted(path.glob("episode_*/*.mcap"))
        + sorted(path.glob("*.mcap"))
    )
    if len(candidates) != 1:
        raise ValueError(f"expected exactly one MCAP below {path}, found {len(candidates)}")
    run_dir = path.parent if path.name == "recording" else path
    return candidates[0], run_dir


def export(args: argparse.Namespace) -> Path:
    mcap_path, run_dir = resolve_mcap(args.input)
    indexed_run = re.fullmatch(r"(?P<index>[0-9]+)_(?P<scenario>.+)", run_dir.name)
    output = (args.output or run_dir / "acoustics_export").resolve()
    if output.exists() and any(output.iterdir()) and not args.force:
        raise FileExistsError(f"refusing to replace non-empty {output}; use --force")
    output.mkdir(parents=True, exist_ok=True)

    data = read_mcap(mcap_path)
    running_events = [event for event in data["episodes"] if event["outcome_state"] == 1]
    if not running_events:
        raise ValueError("recording has no RUNNING EpisodeRecord; refusing to mix simulator startup audio into the episode")
    episode_event = running_events[0]
    episode_start_ns = episode_event["start_time_ns"] or episode_event["timestamp_ns"]
    episode_end_ns = None if args.expected_duration is None else episode_start_ns + round(args.expected_duration * 1_000_000_000)
    raw_chunks = clip_audio_chunks(data["audio"]["raw"], episode_start_ns, episode_end_ns)
    rendered_chunks = clip_audio_chunks(data["audio"]["rendered"], episode_start_ns, episode_end_ns)
    raw, raw_timing, raw_summary = assemble_audio(raw_chunks)
    rendered, rendered_timing, rendered_summary = assemble_audio(rendered_chunks)
    if not np.all(np.isfinite(raw)) or not np.all(np.isfinite(rendered)):
        raise ValueError("audio contains NaN or infinite values")
    if not args.allow_audio_gaps and (raw_summary["gap_frames"] or rendered_summary["gap_frames"]):
        raise ValueError("audio has missing sample frames; inspect audio_timing.parquet or use --allow-audio-gaps")
    if args.expected_duration is not None:
        for summary in (raw_summary, rendered_summary):
            recorded_duration = summary["recorded_frames"] / summary["sample_rate"]
            if recorded_duration + args.duration_tolerance < args.expected_duration:
                raise ValueError(
                    f"{summary['topic']} has only {recorded_duration:.3f}s of samples; "
                    f"expected {args.expected_duration:.3f}s"
                )
    if rendered.shape[1] != 2:
        raise ValueError(f"headphones/stereo must be stereo, got {rendered.shape[1]} channels")
    if rendered_summary["channel_names"] != ["left", "right"]:
        raise ValueError("headphones/stereo channel_names must be [left, right]")
    if len(raw_summary["channel_names"]) != raw_summary["channels"]:
        raise ValueError("raw_array must provide one channel name per channel")
    if raw_summary["max_timestamp_error_ns"] > round(1_000_000_000 / raw_summary["sample_rate"]):
        raise ValueError("raw_array timestamps are not sample-contiguous")
    if rendered_summary["max_timestamp_error_ns"] > round(1_000_000_000 / rendered_summary["sample_rate"]):
        raise ValueError("headphones/stereo timestamps are not sample-contiguous")
    raw_match = re.fullmatch(
        r"(?P<environment>/.+)/(?P<robot>[^/]+)/audio/raw_array",
        raw_summary["topic"],
    )
    rendered_match = re.fullmatch(
        r"(?P<environment>/.+)/(?P<robot>[^/]+)/audio/headphones/stereo",
        rendered_summary["topic"],
    )
    if (
        raw_match is None
        or rendered_match is None
        or raw_match.groupdict() != rendered_match.groupdict()
    ):
        raise ValueError("raw and rendered audio must share one robot and environment namespace")
    raw_namespace = raw_match.group("environment")
    robot_name = raw_match.group("robot")
    rendered_rms = float(np.sqrt(np.mean(rendered * rendered)))
    rendered_clipped_fraction = float(np.mean(np.abs(rendered) >= 0.999))
    if rendered_rms <= args.silence_rms_threshold:
        raise ValueError(f"headphones/stereo is silent (RMS {rendered_rms:g})")
    if rendered_clipped_fraction > args.clipping_fraction_threshold:
        raise ValueError(
            f"headphones/stereo clipping fraction {rendered_clipped_fraction:g} exceeds "
            f"{args.clipping_fraction_threshold:g}"
        )

    odom_topic, robot_rows = _choose_odom(data["odom"], args.robot_odom_topic)
    map_snapshot = select_map_snapshot(data["maps"]["map"], episode_start_ns)
    if map_snapshot is None:
        raise ValueError("recording has no environment occupancy map")
    if map_snapshot["data"].size != map_snapshot["width"] * map_snapshot["height"]:
        raise ValueError("recorded occupancy map dimensions do not match its data")
    if map_snapshot["topic"] != f"{raw_namespace}/map":
        raise ValueError(
            f"recorded map {map_snapshot['topic']!r} does not belong to audio environment {raw_namespace!r}"
        )
    map_frame = str(map_snapshot["frame_id"]).strip("/")
    if not map_frame:
        raise ValueError("recorded occupancy map has an empty frame_id")
    pedestrian_frames = {
        str(row["frame_id"]).strip("/")
        for rows in data["pedestrians"].values()
        for row in rows
    }
    if not pedestrian_frames:
        if not args.allow_missing_pedestrians:
            raise ValueError(
                "recording has no pedestrian pose samples; it cannot produce source-position labels "
                "(use --allow-missing-pedestrians only for audio/robot-only export)"
            )
    elif any(not frame or not _same_frame(frame, map_frame) for frame in pedestrian_frames):
        raise ValueError(
            f"pedestrian poses must use occupancy-map frame {map_frame!r}; found {sorted(pedestrian_frames)}"
        )
    robot_rows, robot_frame_transform = transform_robot_trajectory(
        robot_rows,
        data["transforms"],
        map_frame,
        round(args.max_pose_gap_ms * 1_000_000),
    )
    artifact_prefix = args.artifact_prefix or (indexed_run.group("index") if indexed_run else None)
    prefix = f"{artifact_prefix}_" if artifact_prefix else ""
    context = {
        "world": args.world_name or episode_event["world"] or run_dir.parent.name,
        "scenario": args.scenario_name or (indexed_run.group("scenario") if indexed_run else run_dir.name),
        "execution_index": (
            args.execution_index
            if args.execution_index is not None
            else (int(indexed_run.group("index")) if indexed_run else None)
        ),
        "recording_file": f"{prefix}recording.flac" if prefix else "rendered.flac",
        "scenario_config_file": "scenario.yaml",
        "episode_id": episode_event["episode_id"],
    }
    labels = build_labels(
        rendered, rendered_summary, raw, raw_summary, robot_rows, data["pedestrians"],
        frame_ms=args.label_frame_ms, max_pose_gap_ms=args.max_pose_gap_ms,
        emit_robot_only=args.allow_missing_pedestrians,
        context=context, occupancy_map=map_snapshot,
    )
    rendered_audio_name = f"{prefix}recording.flac" if prefix else "rendered.flac"
    raw_audio_name = f"{prefix}raw.flac" if prefix else "raw.flac"
    metadata_csv_name = f"{prefix}meta.csv" if prefix else "metadata.csv"
    timing_name = f"{prefix}audio_timing.parquet"
    robot_positions_name = f"{prefix}robot_positions.parquet"
    pedestrian_positions_name = f"{prefix}pedestrian_positions.parquet"
    frame_labels_name = f"{prefix}frame_labels.parquet"
    episode_events_name = f"{prefix}episode_events.parquet"
    occupancy_map_name = f"{prefix}occupancy_map.npz"
    door_mask_name = f"{prefix}door_mask.npz"
    validation_name = f"{prefix}validation.json"
    manifest_name = f"{prefix}manifest.yaml" if prefix else "dataset_manifest.yaml"

    write_flac(output / rendered_audio_name, rendered, rendered_summary["sample_rate"])
    if args.raw_flac:
        write_flac(output / raw_audio_name, raw, raw_summary["sample_rate"])
    write_csv(output / metadata_csv_name, labels)
    write_parquet(output / timing_name, raw_timing + rendered_timing)
    write_parquet(output / robot_positions_name, robot_rows)
    write_parquet(output / pedestrian_positions_name, [row for rows in data["pedestrians"].values() for row in rows])
    write_parquet(output / frame_labels_name, labels)
    write_parquet(output / episode_events_name, data["episodes"])
    map_metadata = write_map_snapshot(output / occupancy_map_name, map_snapshot)
    door_snapshot = select_map_snapshot(data["maps"]["door_mask"], episode_start_ns)
    door_metadata = write_map_snapshot(output / door_mask_name, door_snapshot) if door_snapshot is not None else None

    validation = {
        "valid": True,
        "timestamp_semantics": "AudioFrame.header.stamp is simulation time of first sample frame",
        "raw_lossless_location": str(mcap_path),
        "raw": raw_summary,
        "rendered": rendered_summary,
        "robot_odom_topic": odom_topic,
        "robot_frame_transform": robot_frame_transform,
        "environment_namespace": raw_namespace,
        "robot_name": robot_name,
        "occupancy_map": map_metadata,
        "door_mask": door_metadata,
        "pedestrian_count": len(data["pedestrians"]),
        "pedestrian_state_source": data["pedestrian_state_source"],
        "has_pedestrian_labels": bool(data["pedestrians"]),
        "label_rows": len(labels),
        "rendered_rms": rendered_rms,
        "rendered_peak": float(np.max(np.abs(rendered))),
        "rendered_clipped_fraction": rendered_clipped_fraction,
        "rendered_left_right_difference_rms": float(np.sqrt(np.mean((rendered[:, 0] - rendered[:, 1]) ** 2))),
        "clock_first_ns": min(data["clock"]) if data["clock"] else None,
        "clock_last_ns": max(data["clock"]) if data["clock"] else None,
        "episode_start_ns": episode_start_ns,
        "episode_end_ns": episode_end_ns,
        "rendered_audio_file": rendered_audio_name,
        "metadata_csv_file": metadata_csv_name,
    }
    (output / validation_name).write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")
    (output / manifest_name).write_text(yaml.safe_dump({
        "schema_version": 1,
        **context,
        "recording_mcap": str(mcap_path),
        "rendered_audio": rendered_audio_name,
        "metadata_csv": metadata_csv_name,
        "raw_audio": raw_audio_name if args.raw_flac else None,
        "raw_lossless_location": str(mcap_path),
        "audio_timing": timing_name,
        "frame_labels": frame_labels_name,
        "robot_positions": robot_positions_name,
        "pedestrian_positions": pedestrian_positions_name,
        "robot_frame_transform": robot_frame_transform,
        "occupancy_map": map_metadata,
        "door_mask": door_metadata,
        "timestamp_unit": "nanoseconds",
        "timestamp_clock": "ROS simulation time (/clock)",
        "label_frame_ms": args.label_frame_ms,
        "max_pose_gap_ms": args.max_pose_gap_ms,
        "expected_duration_seconds": args.expected_duration,
        "topics": data["topic_types"],
    }, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export synchronized raw/rendered acoustics and source-position labels from an Arena MCAP")
    parser.add_argument("input", type=Path, help="run directory or recording MCAP")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--artifact-prefix", help="safe filename prefix, for example the execution index 0001")
    parser.add_argument("--execution-index", type=int)
    parser.add_argument("--world-name")
    parser.add_argument("--scenario-name")
    parser.add_argument("--robot-odom-topic")
    parser.add_argument("--label-frame-ms", type=float, default=20.0)
    parser.add_argument("--max-pose-gap-ms", type=float, default=100.0)
    parser.add_argument("--expected-duration", type=float)
    parser.add_argument("--duration-tolerance", type=float, default=0.25)
    parser.add_argument("--allow-audio-gaps", action="store_true")
    parser.add_argument("--silence-rms-threshold", type=float, default=1e-5)
    parser.add_argument("--clipping-fraction-threshold", type=float, default=0.01)
    parser.add_argument("--raw-flac", action="store_true", help="also make a listening-oriented FLAC; float MCAP remains the lossless raw representation")
    parser.add_argument(
        "--allow-missing-pedestrians",
        action="store_true",
        help="export audio/robot-only metadata when no pedestrian state samples were recorded; not valid source-position training data",
    )
    parser.add_argument("--force", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.artifact_prefix and (
        args.artifact_prefix in {".", ".."}
        or any(character in args.artifact_prefix for character in "/\\\0")
    ):
        print("export_acoustics_recording: ERROR: --artifact-prefix must be one safe filename component")
        return 2
    try:
        output = export(args)
    except (FileExistsError, FileNotFoundError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"export_acoustics_recording: ERROR: {exc}")
        return 2
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
