# Automated acoustic dataset recording

`record_acoustics_dataset.sh` launches each selected `scenario.yaml` in a fresh
Arena/Gazebo process and records ROS simulation-time data. It records the
Jackal four-microphone raw float stream and the rendered stereo headphone
stream together with `/clock`, episode state, robot odometry, pedestrian
states, TF, the occupancy map, and the door mask.

The default output is:

```text
data/audio_train_set/
└── 0001_<scenario-name>/
    ├── 0001_recording.flac
    ├── 0001_meta.csv
    ├── 0001_validation.json
    ├── 0001_manifest.yaml
    ├── 0001_audio_timing.parquet
    ├── 0001_robot_positions.parquet
    ├── 0001_pedestrian_positions.parquet
    ├── 0001_frame_labels.parquet
    ├── 0001_occupancy_map.npz
    ├── scenario.yaml
    └── episode_000/episode_000.mcap
```

The FLAC contains the robot's final left/right hearing signal. FLAC is compact
and lossless for its integer PCM representation. The MCAP remains the source
of truth for the raw propagated float32 microphone values; converting those
arbitrary floats to FLAC would not preserve them exactly.

Audio and labels use the same ROS simulation clock. `AudioFrame.header.stamp`
is set by the microphone-array publisher to the simulation time of the first
sample in the block. A sample at offset `i` therefore has time
`header.stamp + i/sample_rate`. The exporter creates one metadata row per
20 ms audio window and interpolates robot and pedestrian poses at that exact
timestamp. Each row includes sample offset, absolute simulation timestamp,
robot/source pose and velocity, relative Cartesian position, range, bearing,
elevation, radial velocity, occupancy line-of-sight labels, and audio RMS/peak
features.

Build the changed packages on Ubuntu first:

```bash
cd /opt/arena_ws
source src/Arena/_meta/tools/source
arena rebuild task_generator arena_evaluation arena_simulation_setup
source install/setup.bash
sudo apt install ffmpeg
```

List the deterministic execution order without launching anything:

```bash
arena_simulation_setup/acoustics/record_acoustics_dataset.sh --list
```

Record all worlds and scenarios:

```bash
arena_simulation_setup/acoustics/record_acoustics_dataset.sh
```

Small test runs can select a world, scenario-name pattern, and count:

```bash
arena_simulation_setup/acoustics/record_acoustics_dataset.sh \
  --world-glob 'straight_corridor_O' \
  --scenario-glob '*robot-idle*pedestrians-1*' \
  --max-scenarios 1 \
  --duration 5 \
  --output-relative data/audio_train_set_test
```

Use `--force` to move an incomplete/existing case to a timestamped backup and
repeat it. Completed cases with a valid validation JSON are skipped, so a full
run is resumable. Use `--gazebo-fullscreen` to request a visible fullscreen
Gazebo window. This is best effort: it requires an X/Wayland desktop visible to
the shell and `wmctrl`; AnyDesk does not itself provide recording timestamps.

Validation is performed by the Python waiter/exporter invoked by the Bash
orchestrator, not by Bash syntax alone. A case is accepted only when both audio
streams cover the requested simulation-time interval, the rendered stream is
stereo and non-silent, sample timing is contiguous, robot/pedestrian poses can
be aligned, and the occupancy map belongs to the same environment. Bash exits
on any failed preflight, action, capture, export, or missing validation file.
