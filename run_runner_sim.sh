#!/usr/bin/env bash

set -e

cd "$(dirname "$0")/diploma_ws"

read -r -p "Number of robots [1]: " robot_count
read -r -p "Spawn spacing in meters [2.0]: " spawn_spacing
read -r -p "Localization mode (single/all) [single]: " localization_mode

robot_count=${robot_count:-1}
spawn_spacing=${spawn_spacing:-2.0}
localization_mode=${localization_mode:-single}

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select runner_sim
source install/setup.bash

setsid ros2 launch runner_sim runner_sim.launch.py \
  robot_count:="$robot_count" \
  spawn_spacing:="$spawn_spacing" \
  localization_mode:="$localization_mode" \
  rviz:=true &

simulation_pid=$!
cleanup_done=false

cleanup() {
  if [[ "$cleanup_done" == true ]]; then
    return
  fi
  cleanup_done=true

  # setsid gives the launch tree its own process group; stop the whole tree first.
  kill -INT -- -"$simulation_pid" 2>/dev/null || kill -INT "$simulation_pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$simulation_pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -TERM -- -"$simulation_pid" 2>/dev/null || true
  kill -KILL -- -"$simulation_pid" 2>/dev/null || true
  wait "$simulation_pid" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

printf '\nSimulation is running. Press q to quit.\n'
while read -r -n 1 key; do
  if [[ "$key" == "q" ]]; then
    break
  fi
done

cleanup