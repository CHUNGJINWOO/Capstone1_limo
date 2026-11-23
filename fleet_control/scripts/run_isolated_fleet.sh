#!/usr/bin/env bash
# Interactive localhost stack for the final integration stage. Requires a sourced overlay.
set -euo pipefail
test "$#" -eq 3 || { echo "Usage: $0 NEW_RUN_DIR HTTP_PORT ROS_DOMAIN_ID" >&2; exit 2; }
run_dir=$1
port=$2
domain=$3
[[ $port =~ ^[0-9]{4,5}$ && $domain =~ ^[0-9]{1,3}$ ]] || exit 2
(( 10#$domain >= 1 && 10#$domain <= 232 && 10#$port >= 20000 && 10#$port <= 65535 )) || exit 2

fleet_dir=$(cd "$(dirname "$0")/.." && pwd)
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$((10#$domain)) ROS2CLI_NO_DAEMON=1
umask 077
mkdir "$run_dir"
run_dir=$(cd "$run_dir" && pwd)
export ROS_LOG_DIR="$run_dir/ros"
export PYTHONPATH="$fleet_dir/server${PYTHONPATH:+:$PYTHONPATH}"
pids=()
bringup_pid=''
cleanup() {
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    if [[ -n $bringup_pid ]]; then kill -TERM -- "-$bringup_pid" 2>/dev/null || true; fi
    local deadline=$((SECONDS + 16))
    while (( SECONDS < deadline )); do
        if [[ -z $(jobs -pr) ]] && { [[ -z $bringup_pid ]] || ! kill -0 -- "-$bringup_pid" 2>/dev/null; }; then
            break
        fi
        sleep .2
    done
    if [[ -n $bringup_pid ]]; then kill -KILL -- "-$bringup_pid" 2>/dev/null || true; fi
    # jobs -pr enumerates only this harness's outstanding shell children.
    jobs -pr | xargs -r kill -KILL || true
    for pid in "${pids[@]}"; do wait "$pid" || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 -m fleet_server init --data-dir "$run_dir/fleet" --secrets-dir "$run_dir/secrets"     --robots locked_bench second > "$run_dir/init.log"
python3 -m fleet_server serve --data-dir "$run_dir/fleet" --bind 127.0.0.1 --port "$port"     > "$run_dir/fleet.log" 2>&1 &
pids+=("$!")
python3 "$fleet_dir/tests/isolated_sensor_fixture.py" > "$run_dir/sensors.log" 2>&1 &
pids+=("$!")
setsid ros2 launch limo_fleet_bringup isolated_locked.launch.py     token_file:="$run_dir/secrets/agent-locked_bench.token" state_dir:="$run_dir/agent"     server_url:="http://127.0.0.1:$port" mapping:=true bag:=true manual:=true velocity:=true camera:=true     > "$run_dir/bringup.log" 2>&1 &
bringup_pid=$!
pids+=("$bringup_pid")
# Second card uses a separate DDS domain so it cannot observe or command the test stack.
next_domain=$((10#$domain % 232 + 1))
agent_exec="$(ros2 pkg prefix limo_robot_agent)/lib/limo_robot_agent/robot_agent"
ROS_DOMAIN_ID=$next_domain "$agent_exec" --ros-args     -p robot_id:=second -p mode:=mock -p simulated:=true -p allow_missions:=false     -p token_file:="$run_dir/secrets/agent-second.token" -p state_dir:="$run_dir/second"     -p server_url:="http://127.0.0.1:$port" > "$run_dir/second.log" 2>&1 &
pids+=("$!")
printf 'Fleet: http://127.0.0.1:%s/\nLogs and test tokens: %s\n' "$port" "$run_dir"
wait -n "${pids[@]}"
