#!/usr/bin/env bash
# Record every acoustics scenario with simulation-time-synchronized ROS audio.

set -Eeo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WORLDS_ROOT="${SCRIPT_DIR}/worlds"
OUTPUT_REL='data/audio_train_set'
SCENARIO_GLOB='*'
WORLD_GLOB='*'
MAX_SCENARIOS=0
DURATION=30
WALL_TIMEOUT=90
READY_TIMEOUT=120
RUNTIME='auto'
CONTAINER=''
FORCE=0
LIST_ONLY=0
GAZEBO_FULLSCREEN=0
LAUNCH_CLIENT_PID=''
ACTION_PID=''
CAPTURE_PID=''
SUPERVISOR_PID_FILE=''

usage() {
    cat <<'EOF'
Usage: record_acoustics_dataset.sh [options] [-- extra launch arguments]

Records every selected scenario to:
  data/audio_train_set/<index>_<scenario>/

The MCAP contains the Jackal's raw four-microphone AudioFrame stream and
rendered stereo headphone stream, /clock, occupancy map, odometry, pedestrian
states, TF, and episode events. The exporter writes <index>_recording.flac, <index>_meta.csv, the
exact map grid, timing tables, position tables, and frame-label Parquet.

Options:
  --worlds-root DIR       Worlds directory (default: acoustics/worlds)
  --output-relative DIR   Output below the Arena workspace (default: data/audio_train_set)
  --scenario-glob GLOB    Scenario-directory glob (default: *, all scenarios)
  --world-glob GLOB       World-directory glob (default: *, all worlds)
  --max-scenarios N       Stop after N selected cases (default: 0, unlimited)
  --duration SEC          Required simulation-time audio duration (default: 30)
  --wall-timeout SEC      Per-capture wall-time watchdog (default: 90)
  --ready-timeout SEC     Simulator/audio startup timeout (default: 120)
  --runtime MODE          auto, docker, or native (default: auto)
  --container NAME_OR_ID  Arena container (implies docker)
  --gazebo-fullscreen     Open the Gazebo GUI and request fullscreen when the desktop permits it
  --force                 Move an existing run to a timestamped backup and repeat it
  --list                  List selected index/world/scenario triples
  -h, --help              Show this help
EOF
}

die() { printf 'record_acoustics_dataset: ERROR: %s\n' "$*" >&2; exit 1; }
note() { printf 'record_acoustics_dataset: %s\n' "$*" >&2; }
is_number() { [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; }

EXTRA_LAUNCH_ARGS=()
while (($#)); do
    case "$1" in
        --worlds-root) WORLDS_ROOT="${2:?missing value}"; shift 2 ;;
        --output-relative) OUTPUT_REL="${2:?missing value}"; shift 2 ;;
        --scenario-glob) SCENARIO_GLOB="${2:?missing value}"; shift 2 ;;
        --world-glob) WORLD_GLOB="${2:?missing value}"; shift 2 ;;
        --max-scenarios) MAX_SCENARIOS="${2:?missing value}"; shift 2 ;;
        --duration) DURATION="${2:?missing value}"; shift 2 ;;
        --wall-timeout) WALL_TIMEOUT="${2:?missing value}"; shift 2 ;;
        --ready-timeout) READY_TIMEOUT="${2:?missing value}"; shift 2 ;;
        --runtime) RUNTIME="${2:?missing value}"; shift 2 ;;
        --container) CONTAINER="${2:?missing value}"; RUNTIME='docker'; shift 2 ;;
        --gazebo-fullscreen) GAZEBO_FULLSCREEN=1; shift ;;
        --force) FORCE=1; shift ;;
        --list) LIST_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; EXTRA_LAUNCH_ARGS=("$@"); break ;;
        *) die "unknown argument: $1" ;;
    esac
done

[[ -d "$WORLDS_ROOT" ]] || die "worlds directory not found: $WORLDS_ROOT"
[[ "$OUTPUT_REL" != /* && "$OUTPUT_REL" != *'..'* ]] || die '--output-relative must be a safe workspace-relative path'
[[ "$OUTPUT_REL" == data || "$OUTPUT_REL" == data/* ]] || die '--output-relative must be below data/'
[[ "$MAX_SCENARIOS" =~ ^[0-9]+$ ]] || die '--max-scenarios must be a non-negative integer'
is_number "$DURATION" || die '--duration must be numeric'
is_number "$WALL_TIMEOUT" || die '--wall-timeout must be numeric'
is_number "$READY_TIMEOUT" || die '--ready-timeout must be numeric'
case "$RUNTIME" in auto|docker|native) ;; *) die '--runtime must be auto, docker, or native' ;; esac

for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
    case "$arg" in
        sim:=*|world:=*|robot:=*|human:=*|auditory:=*|auditory.playback:=*|microphone_mode:=*|scenario_file:=*|tm_robots:=*|tm_obstacles:=*|auto_reset:=*|env_n:=*|viz:=*|headless:=*|record_data_dir:=*)
            die "the script owns launch argument '$arg'"
            ;;
    esac
done

SCENARIO_FILES=()
while IFS= read -r -d '' scenario_file; do
    scenario_dir="$(dirname -- "$scenario_file")"
    world_dir="$(dirname -- "$(dirname -- "$scenario_dir")")"
    [[ "$(basename -- "$world_dir")" == $WORLD_GLOB ]] || continue
    [[ "$(basename -- "$scenario_dir")" == $SCENARIO_GLOB ]] || continue
    SCENARIO_FILES+=("$scenario_file")
    if ((MAX_SCENARIOS > 0 && ${#SCENARIO_FILES[@]} >= MAX_SCENARIOS)); then break; fi
done < <(find "$WORLDS_ROOT" -type f -path '*/scenarios/*/scenario.yaml' -print0 | sort -z)
((${#SCENARIO_FILES[@]})) || die "no scenario.yaml matched world '${WORLD_GLOB}' and scenario '${SCENARIO_GLOB}'"

if ((LIST_ONLY)); then
    execution_number=0
    for scenario_file in "${SCENARIO_FILES[@]}"; do
        execution_number=$((execution_number + 1))
        printf -v run_index '%04d' "$execution_number"
        scenario_dir="$(dirname -- "$scenario_file")"
        world_dir="$(dirname -- "$(dirname -- "$scenario_dir")")"
        printf '%s\t%s\t%s\n' "$run_index" "$(basename -- "$world_dir")" "$(basename -- "$scenario_dir")"
    done
    exit 0
fi

find_arena_container() {
    local -a ids=()
    while IFS= read -r id; do [[ -z "$id" ]] || ids+=("$id"); done \
        < <(docker ps --filter label=com.docker.compose.service=arena --format '{{.ID}}')
    ((${#ids[@]} <= 1)) || die 'multiple Arena containers found; pass --container'
    ((${#ids[@]} == 1)) && printf '%s\n' "${ids[0]}"
}

if [[ "$RUNTIME" == auto ]]; then
    if command -v docker >/dev/null; then CONTAINER="$(find_arena_container)"; fi
    [[ -z "$CONTAINER" ]] || RUNTIME='docker'
    [[ "$RUNTIME" != auto ]] || RUNTIME='native'
fi
if [[ "$RUNTIME" == docker ]]; then
    command -v docker >/dev/null || die 'docker is unavailable'
    [[ -n "$CONTAINER" ]] || CONTAINER="$(find_arena_container)"
    [[ -n "$CONTAINER" ]] || die 'no running Arena container found'
    docker exec "$CONTAINER" true >/dev/null || die "cannot access container $CONTAINER"
    HOST_DATA_ROOT="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/opt/arena_ws/data"}}{{.Source}}{{end}}{{end}}' "$CONTAINER")"
    [[ -n "$HOST_DATA_ROOT" ]] || die 'Arena container has no /opt/arena_ws/data host mount'
    if [[ "$OUTPUT_REL" == data ]]; then OUTPUT_ROOT="$HOST_DATA_ROOT"; else OUTPUT_ROOT="${HOST_DATA_ROOT}/${OUTPUT_REL#data/}"; fi
    ARENA_OUTPUT_ROOT="/opt/arena_ws/${OUTPUT_REL}"
else
    command -v ros2 >/dev/null || die 'ROS 2 is unavailable; source Arena first'
    if [[ -n "${ARENA_DATA_DIR:-}" ]]; then
        if [[ "$OUTPUT_REL" == data ]]; then OUTPUT_ROOT="$ARENA_DATA_DIR"; else OUTPUT_ROOT="${ARENA_DATA_DIR}/${OUTPUT_REL#data/}"; fi
    elif [[ "$(basename -- "$(dirname -- "$WORKSPACE_ROOT")")" == src ]]; then
        OUTPUT_ROOT="$(dirname -- "$(dirname -- "$WORKSPACE_ROOT")")/${OUTPUT_REL}"
    else
        OUTPUT_ROOT="${WORKSPACE_ROOT}/${OUTPUT_REL}"
    fi
    ARENA_OUTPUT_ROOT="$OUTPUT_ROOT"
fi

run_in_arena() {
    if [[ "$RUNTIME" == docker ]]; then
        docker exec "$CONTAINER" bash --norc -c 'cd /opt/arena_ws && source ./source >/dev/null && "$@"' _ "$@"
    else
        (cd "$WORKSPACE_ROOT" && "$@")
    fi
}

request_gazebo_fullscreen() {
    local window_id=''
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        note 'Gazebo GUI was requested, but this shell has no desktop display; continuing windowed/headless-display automation'
        return 0
    fi
    if ! command -v wmctrl >/dev/null; then
        note 'Gazebo GUI is enabled, but wmctrl is unavailable; install it with: sudo apt install wmctrl'
        return 0
    fi
    for _ in {1..20}; do
        window_id="$(wmctrl -lx 2>/dev/null | awk 'BEGIN { IGNORECASE=1 } /gazebo|gz[-_ ]?(sim|gui)/ { print $1; exit }')"
        if [[ -n "$window_id" ]]; then
            if wmctrl -ir "$window_id" -b add,fullscreen,maximized_vert,maximized_horz 2>/dev/null; then
                note "Gazebo window $window_id set to fullscreen"
                return 0
            fi
        fi
        sleep 1
    done
    note 'Gazebo GUI is running, but the window manager did not expose a fullscreen-capable Gazebo window'
}

stop_launch() {
    [[ -n "$LAUNCH_CLIENT_PID" ]] || return 0
    if [[ "$RUNTIME" == docker ]]; then
        docker exec "$CONTAINER" bash --norc -c 'test -s "$1" && kill -INT "$(cat "$1")" 2>/dev/null || true' _ "$SUPERVISOR_PID_FILE" >/dev/null 2>&1 || true
    else
        kill -INT -- "-${LAUNCH_CLIENT_PID}" 2>/dev/null || true
    fi
    for _ in {1..40}; do kill -0 "$LAUNCH_CLIENT_PID" 2>/dev/null || break; sleep 1; done
    if kill -0 "$LAUNCH_CLIENT_PID" 2>/dev/null; then
        note 'graceful shutdown timed out; sending SIGTERM'
        if [[ "$RUNTIME" == docker ]]; then
            docker exec "$CONTAINER" bash --norc -c 'test -s "$1" && kill -TERM "$(cat "$1")" 2>/dev/null || true' _ "$SUPERVISOR_PID_FILE" >/dev/null 2>&1 || true
        else
            kill -TERM -- "-${LAUNCH_CLIENT_PID}" 2>/dev/null || true
        fi
    fi
    wait "$LAUNCH_CLIENT_PID" 2>/dev/null || true
    LAUNCH_CLIENT_PID=''
}

cleanup() {
    local status=$?
    if [[ -n "$CAPTURE_PID" ]] && kill -0 "$CAPTURE_PID" 2>/dev/null; then kill "$CAPTURE_PID" 2>/dev/null || true; fi
    if [[ -n "$ACTION_PID" ]] && kill -0 "$ACTION_PID" 2>/dev/null; then kill "$ACTION_PID" 2>/dev/null || true; fi
    stop_launch
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

mkdir -p "$OUTPUT_ROOT"
note "runtime=$RUNTIME scenarios=${#SCENARIO_FILES[@]} duration=${DURATION}s simulation time output=$OUTPUT_ROOT"

execution_number=0
for scenario_file in "${SCENARIO_FILES[@]}"; do
    execution_number=$((execution_number + 1))
    printf -v run_index '%04d' "$execution_number"
    scenario_dir="$(dirname -- "$scenario_file")"
    scenario_name="$(basename -- "$scenario_dir")"
    world_dir="$(dirname -- "$(dirname -- "$scenario_dir")")"
    world_name="$(basename -- "$world_dir")"
    run_name="${run_index}_${scenario_name}"
    run_host="${OUTPUT_ROOT}/${run_name}"
    run_arena="${ARENA_OUTPUT_ROOT}/${run_name}"
    validation="${run_host}/${run_index}_validation.json"

    if [[ -s "$validation" && "$FORCE" -eq 0 ]]; then note "SKIP $run_name (validated)"; continue; fi
    existing_actions="$(run_in_arena ros2 action list 2>/dev/null | grep '/lifecycle/run_episode$' || true)"
    [[ -z "$existing_actions" ]] || die "another Arena environment is already running; stop it before recording: $existing_actions"
    if [[ -e "$run_host" && "$FORCE" -eq 1 ]]; then
        backup="${run_host}.backup.$(date +%Y%m%d-%H%M%S)"
        mv -- "$run_host" "$backup"
        note "moved previous run to $backup"
    elif [[ -e "$run_host" ]]; then
        die "incomplete existing run at $run_host (use --force after inspecting it)"
    fi
    mkdir -p "$run_host"
    cp -- "$scenario_file" "${run_host}/scenario.yaml"
    launch_log="${run_host}/launch.log"
    action_log="${run_host}/action.log"
    waiter_log="${run_host}/capture_wait.json"
    note "START $run_name world=$world_name"

    if ((GAZEBO_FULLSCREEN)); then
        display_args=('viz:=false' 'headless:=false')
    else
        display_args=('viz:=false' 'headless:=true')
    fi
    launch_args=(
        'sim:=gazebo' "world:=${world_name}" 'robot:=jackal' 'human:=arena' 'auditory:=arena'
        'microphone_mode:=four_mic' 'auditory.playback:=none'
        'tm_robots:=scenario' 'tm_obstacles:=scenario' "scenario_file:=${scenario_name}"
        'auto_reset:=false' 'env_n:=1' "${display_args[@]}"
        "record_data_dir:=${run_arena}" "${EXTRA_LAUNCH_ARGS[@]}"
    )
    if [[ "$RUNTIME" == docker ]]; then
        SUPERVISOR_PID_FILE="/tmp/arena-acoustics-${BASHPID}.pid"
        docker exec "$CONTAINER" bash --norc -c \
            'cd /opt/arena_ws && source ./source >/dev/null && printf "%s\n" "$BASHPID" > "$1" && shift && exec python3 -m arena_bringup.supervisor "$@"' \
            _ "$SUPERVISOR_PID_FILE" "${launch_args[@]}" >"$launch_log" 2>&1 &
    else
        setsid python3 -m arena_bringup.supervisor "${launch_args[@]}" >"$launch_log" 2>&1 &
    fi
    LAUNCH_CLIENT_PID=$!

    deadline=$((SECONDS + ${READY_TIMEOUT%.*}))
    episode_action=''
    env_namespace=''
    raw_type=''
    rendered_type=''
    map_type=''
    map_state=''
    microphone_node=''
    map_requested=0
    while ((SECONDS <= deadline)); do
        kill -0 "$LAUNCH_CLIENT_PID" 2>/dev/null || { tail -n 80 "$launch_log" >&2; die 'Arena exited during startup'; }
        episode_action="$(run_in_arena ros2 action list 2>/dev/null | grep '/lifecycle/run_episode$' | head -n 1 || true)"
        if [[ -n "$episode_action" ]]; then
            env_namespace="${episode_action%/lifecycle/run_episode}"
            if ((map_requested == 0)); then
                require_map_type="$(run_in_arena ros2 service type "${env_namespace}/runtime/require_map" 2>/dev/null || true)"
                if [[ "$require_map_type" == 'std_srvs/srv/Empty' ]]; then
                    note "requesting map server for ${env_namespace}"
                    run_in_arena ros2 service call "${env_namespace}/runtime/require_map" std_srvs/srv/Empty '{}' >/dev/null
                    map_requested=1
                fi
            fi
            raw_type="$(run_in_arena ros2 topic type "${env_namespace}/jackal/audio/raw_array" 2>/dev/null || true)"
            rendered_type="$(run_in_arena ros2 topic type "${env_namespace}/jackal/audio/headphones/stereo" 2>/dev/null || true)"
            map_type="$(run_in_arena ros2 topic type "${env_namespace}/map" 2>/dev/null || true)"
            map_state="$(run_in_arena ros2 lifecycle get "${env_namespace}/map_server" 2>/dev/null || true)"
            microphone_node="$(run_in_arena ros2 node list 2>/dev/null | grep -x "${env_namespace}/microphone_array_node" || true)"
        fi
        if [[ "$raw_type" == 'task_generator_msgs/msg/AudioFrame' && "$rendered_type" == 'task_generator_msgs/msg/AudioFrame' && "$map_type" == 'nav_msgs/msg/OccupancyGrid' && "$map_state" == *active* && -n "$microphone_node" ]]; then break; fi
        sleep 1
    done
    [[ -n "$episode_action" ]] || die "episode action not ready; see $launch_log"
    [[ -n "$microphone_node" ]] || die "${env_namespace}/microphone_array_node is not running"
    [[ "$map_type" == 'nav_msgs/msg/OccupancyGrid' && "$map_state" == *active* ]] || die "environment map server is not active at ${env_namespace}/map"
    [[ "$raw_type" == 'task_generator_msgs/msg/AudioFrame' ]] || die "${env_namespace}/jackal/audio/raw_array publisher is missing or has the wrong type"
    [[ "$rendered_type" == 'task_generator_msgs/msg/AudioFrame' ]] || die "${env_namespace}/jackal/audio/headphones/stereo publisher is missing or has the wrong type"
    if ((GAZEBO_FULLSCREEN)); then request_gazebo_fullscreen; fi

    run_in_arena python3 -m arena_simulation_setup.acoustics.wait_capture \
        --namespace "$env_namespace" --robot-name jackal \
        --duration "$DURATION" --wall-timeout "$WALL_TIMEOUT" >"$waiter_log" &
    CAPTURE_PID=$!
    capture_ready=0
    for _ in {1..20}; do
        kill -0 "$CAPTURE_PID" 2>/dev/null || { wait "$CAPTURE_PID" 2>/dev/null || true; die "capture waiter exited before subscribing; see $waiter_log"; }
        if run_in_arena ros2 node list 2>/dev/null | grep -qx '/arena_acoustics_capture_waiter'; then
            capture_ready=1
            break
        fi
        sleep 0.25
    done
    ((capture_ready)) || die 'capture waiter did not become ready before the episode action'
    run_in_arena ros2 action send_goal "$episode_action" task_generator_msgs/action/RunEpisode \
        "{world: '${world_name}', seed: -1}" >"$action_log" 2>&1 &
    ACTION_PID=$!
    if ! wait "$CAPTURE_PID"; then
        CAPTURE_PID=''
        [[ ! -s "$waiter_log" ]] || tail -n 20 "$waiter_log" >&2
        die "capture did not reach ${DURATION}s of synchronized raw and rendered audio"
    fi
    CAPTURE_PID=''

    if kill -0 "$ACTION_PID" 2>/dev/null; then kill "$ACTION_PID" 2>/dev/null || true; fi
    wait "$ACTION_PID" 2>/dev/null || true
    ACTION_PID=''
    stop_launch

    run_in_arena python3 -m arena_simulation_setup.acoustics.export_recording \
        "$run_arena" --output "$run_arena" --force --expected-duration "$DURATION" \
        --artifact-prefix "$run_index" --execution-index "$execution_number" \
        --world-name "$world_name" --scenario-name "$scenario_name"
    [[ -s "$validation" ]] || die "export did not create $validation"
    note "DONE  $run_host"
done

trap - EXIT INT TERM
note 'all selected scenarios have valid synchronized recordings'
