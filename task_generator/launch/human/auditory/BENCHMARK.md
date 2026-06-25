# Auditory CPU and Latency Benchmark

Run the same scenario twice: once without auditory nodes, once with auditory nodes.
Keep simulator, world, robot count, seed, headless setting, and duration identical.

Example:

```bash
ros2 run task_generator auditory_benchmark \
  --baseline-cmd "arena launch sim:=gazebo headless:=true human:=hunav rviz:=false" \
  --auditory-cmd "arena launch sim:=gazebo headless:=true human:=arena rviz:=false" \
  --duration-sec 120 \
  --startup-delay-sec 20 \
  --output-json /tmp/auditory_benchmark.json \
  --output-csv /tmp/auditory_benchmark.csv
```

Use the actual two commands you normally use for baseline and auditory runs.
This repository does not install `arena_bringup/start.launch.py`; the usual
entry point is the `arena launch` CLI or `arena_runtime.launch.py` plus
`task_generator.launch.py`.

For a deterministic auditory workload, add synthetic events:

```bash
ros2 run task_generator auditory_benchmark \
  --baseline-cmd "<baseline launch command>" \
  --auditory-cmd "<auditory launch command>" \
  --inject-rate-hz 5 \
  --duration-sec 120
```

Reported metrics:

- `cpu_percent_avg` / `cpu_percent_max`: process-tree CPU, where 100 means one full core.
- `sound_events`: observed `SoundEvent` count.
- `heard_events`: observed `HeardSoundEvent` count.
- `latency_ms_*`: wall-clock time from observed/published `SoundEvent` to matching `HeardSoundEvent`.

The baseline run normally has no `heard_events`; use its CPU values as the
same-condition comparison against the auditory run.
