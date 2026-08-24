# task_generator human simulator

`BaseHumanSimulator` manages the pedestrian lifecycle (spawn, move, remove)
and the per-episode obstacle bookkeeping layer. Implementations are registered
in `HumanSimulatorRegistry`. See also [sim interface](../../../../arena_runtime/arena_runtime/arena_runtime/sim/README.md) for the
physics-simulator counterpart (`BaseSim`).

## `BaseHumanSimulator`

[`__init__.py:19`](__init__.py#L19)

```python
class BaseHumanSimulator(NodeInterface, abc.ABC):
```

Holds a `KnownObstacles` table that tracks every spawned obstacle by name,
its `spawned` flag, and its `ObstacleLayer`. On `__init__` it subscribes to
`<namespace>/arena_peds`, caches ped positions in `_ped_positions_xy`, and
calls `self._simulator.attach_human_simulator(self)` so the mechanism shim
(`MechanismITF`, see [sim interface](../../../../arena_runtime/arena_runtime/arena_runtime/sim/README.md))
can read ground-truth ped positions and dispatch ped teleports through this
class. Public methods delegate to both the physics `BaseSim` and the
human-sim `_*_impl` methods:

| Public method | Purpose |
| --- | --- |
| `spawn_obstacles(obstacles, layer)` | spawn or move static obstacles; layer defaults to `INUSE` |
| `spawn_dynamic_obstacles(obstacles)` | spawn or move dynamic obstacles (`INUSE`) |
| `spawn_world(walls, doors, collision_walls=())` | spawn world geometry in both sim and human-sim layers; `collision_walls` register in the human-sim layer only (avoidance), never spawned visually |
| `unuse_obstacles()` | call `_remove_obstacles_impl`, then flip all `INUSE` layers to `UNUSED` |
| `remove_obstacles(purge)` | remove all obstacles at or below `purge` layer from both layers; `WORLD` survives unless `purge >= WORLD` |
| `spawn_robot(robots)` | spawn in physics sim, then call `_spawn_robot_impl` |
| `remove_robot(robots)` | remove from physics sim, then call `_remove_robot_impl` |
| `move_robot(robots)` | move in physics sim, then call `_move_robot_impl` |

### `HumanSimulator` Protocol surface

`BaseHumanSimulator` satisfies the Protocol the mechanism shim reads from.
Inherited defaults work for all current subclasses; override only for
specialized teleport semantics (e.g. resetting an internal agent list).

| Method | Signature |
| --- | --- |
| `pedestrian_discs()` | `() -> Iterable[tuple[str, tuple[float, float], float]]` (sync, reads `_ped_positions_xy`, radius is `PED_RADIUS`) |
| `pedestrian_teleport(destinations)` | `(Mapping[str, tuple[float, float]]) -> bool` (async, dispatches via `relay_pedestrian_update`) |

### Abstract `_impl` methods

Every subclass must implement:

| Method | Purpose |
| --- | --- |
| `_spawn_obstacles_impl(obstacles)` | human-sim side: register or prepare static obstacles |
| `_spawn_dynamic_obstacles_impl(obstacles)` | human-sim side: register pedestrian agents |
| `_remove_obstacles_impl()` | human-sim side: signal removal of current episode's obstacles |
| `_spawn_walls_impl(walls)` | human-sim side: ingest wall geometry |
| `_spawn_doors_impl(doors)` | human-sim side: ingest door geometry |
| `_spawn_robot_impl(robots)` | human-sim side: register robots |
| `_remove_robot_impl(robots)` | human-sim side: deregister robots |
| `_move_robot_impl(robots)` | human-sim side: update robot positions |

## GaitGenerator as articulation ground truth

`BaseHumanSimulator.publish_arena_peds` is the single point where skeletal
joint angles are committed to the `arena_peds` bus.  For each `Pedestrian` in
the outgoing message it checks `ped.joint_state.name`:

- **non-empty**: upstream backend supplied its own joint state, published unchanged (override path: an upstream producer that already computes joint angles).
- **empty**: `publish_arena_peds` calls `GaitGenerator.compute` + `GaitGenerator.joint_state` and fills the field with bare semantic joint names (20 DOF, ~9 active per gait mode, no body suffix).

The filled field feeds the ROS4HRI skeleton in rviz through `hri_producer`.  The 3D engines do not read it: Isaac animates pedestrians with its native omni.anim.people AnimGraph and Gazebo clip-scrubs `walk.dae`, both driven by pedestrian pose and twist.  So `GaitGenerator` is the ROS-side articulation ground truth, while the in-engine meshes play plausible locomotion that is not bone-for-bone identical to it.

## Visualization topics

Pedestrian visualization flows through one data feed plus a few marker layers, all at
env level (`<env_ns>/`) and shared by every backend:

| Topic | Producer | QoS | Role |
| --- | --- | --- | --- |
| `arena_peds` | each backend (`publish_arena_peds`) | reliable, volatile | Pedestrian state feed (positions/velocities/joint_state). `joint_state` carries bare semantic joint names filled by `GaitGenerator` unless the backend overrides it (non-empty = upstream wins). |
| `humans/bodies/tracked`, `humans/persons/*`, `humans/bodies/<id>/joint_states` | `hri_producer` node | per REP-155 | **Canonical** ROS4HRI projection of `arena_peds`: id lists, per-person engagement, per-body joint states, per-body URDF on param `human_description_<id>`, TF `body_<id>`. |
| `pedestrian_markers/extra` | base class (`publish_markers`) | best-effort, volatile | Backend-internal debug overlay (e.g. arena_humansim forwards its planner viz). Off by default. |
| `pedestrian_markers/static` | base class (`publish_static_markers`) | reliable, transient-local, depth 1 | Latched static scene as one combined topic. |
| `pedestrian_markers/static_*` | adapter | reliable, transient-local, depth 1 | Latched static scene split per bucket (`/static_walls`, `/static_objects`, ...). |

**Rendering contract.** The canonical human view is an animated articulated skeleton, produced by the
[`hri_producer`](../../../../utils/rviz_utils/rviz_utils/scripts/hri_producer.py) node, which subscribes
`arena_peds` and projects it into the ROS4HRI (REP-155) `humans/` namespace: id lists, per-person
engagement, and a per-body `robot_state_publisher` (pooled) driven from `humans/bodies/<id>/joint_states`
against the `human_description` URDF rig. The [`hri_rviz/Skeletons3D`](https://github.com/ros4hri/hri_rviz)
display renders one kinematic model per body.

`hri_producer` is a relay: it re-suffixes joint names from `arena_peds.joint_state` per body ID and
publishes them directly.  The producer's own `GaitGenerator` instance is a **fallback only** for peds whose
`joint_state` arrives empty (backends that do not fill joint_state on the bus).
`extra` is backend debug, disabled by default.

**Display kinds** (`arena_viz.DisplayKind`):
- `PEDESTRIANS`: the canonical `hri_rviz/Skeletons3D` skeleton display, keyed on the env `humans/`
  namespace. Note: the upstream display uses absolute `/humans` paths via libhri, so per-env namespacing
  is a known limitation.
- `MARKER_ARRAY`: generic MarkerArray passthrough, no namespace assumptions; used for `extra` and all
  `static*` layers.

The auto-rviz manifest ([`node.py` `_publish_viz_manifest`](../../node.py)) groups these into a
**Pedestrians** folder (skeleton display + `extra`, off) and a separate **Static** folder (`static`,
`static_walls`, `static_objects`). The `pedestrian_markers/` prefix on the debug/static layers is
historical, they are overlays, not pedestrian data.

Current adapters:

| Adapter | Static topics published |
| --- | --- |
| `arena_humansim` | `pedestrian_markers/static_walls`, `pedestrian_markers/static_objects` |
| `dummy` | `pedestrian_markers/static_walls`, `pedestrian_markers/static_objects` |

## PROMPT registration

`TM_Prompt` is not registered centrally. Each `BaseHumanSimulator` subclass
that supports LLM-driven obstacle generation registers its own `TM_Obstacles`
variant (including the system-prompt text and response parser) via
`_register_task_modes` at class-definition time. This co-locates the prompt
with the simulator that will animate the resulting agents.

## Registered implementations

`HumanSimulatorRegistry` ([`__init__.py`](__init__.py)) maps
`Constants.HumanSimulator` keys to async factory functions. Per-sim defaults
(resolved in `task_generator.launch.py`): `gazebo`/`isaac` sims default to
`human:=arena`, the `dummy` sim (and bare/test contexts) to
`human:=dummy`.

| Key | Class | File | Notes |
| --- | --- | --- | --- |
| `dummy` | `DummyHumanSimulator` | [`dummy.py`](dummy.py) | no-op stubs; used in test/offline contexts |
| `arena` | `ArenaHumanSimulator` | [`arena_humansim/arena_humansim.py`](arena_humansim/arena_humansim.py) | integrates with the arena_humansim pedestrian simulator (subsystem mode) |

## arena_humansim agent types

`ArenaHumanSimulator` derives pedestrian parameters from the `agent:` block of
each dynamic obstacle (`ArenaHumanDynamicObstacle`,
[`arena_humansim/__init__.py`](arena_humansim/__init__.py)). `agent_type`
references either a built-in arena_humansim type (`adult`, `elder`, defined in
`arena_humansim/config/agent_types/`) or a scenario-local YAML file resolved
relative to the scenario (`agent_type: ./doctor.yaml`).

## Adding a new BaseHumanSimulator

1. Create `simulators/human/<name>.py` with a class extending
   `BaseHumanSimulator`; implement all eight `_*_impl` abstract methods.
2. Add `<NAME> = "<name>"` to `Constants.HumanSimulator` in
   [`constants/__init__.py`](../../constants/__init__.py).
3. Register a lazy async factory in [`simulators/human/__init__.py`](__init__.py):

```python
@HumanSimulatorRegistry.register(Constants.HumanSimulator.MY_SIM)
async def _my_sim(**kwargs):
    from .my_sim import MyHumanSimulator
    return await MyHumanSimulator.create(**kwargs)
```

If the implementation supports LLM-driven obstacle generation, call
`_register_task_modes` in the class body to register a `TM_Obstacles`
subclass (with prompt text) under `Constants.TaskMode.TM_Obstacles.PROMPT`.
