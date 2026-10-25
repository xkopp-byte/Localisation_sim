#!/usr/bin/env bash

set -e

cd "$(dirname "$0")/diploma_ws"

select_mission() {
  local mission_directory="$PWD/src/runner_sim/missions"
  local selected_index=0
  local key
  local mission_name
  local -a mission_options=("No mission")

  while IFS= read -r mission_name; do
    mission_options+=("$mission_name")
  done < <(find "$mission_directory" -maxdepth 1 -type f -name '*.csv' -printf '%f\n' | sort)

  while true; do
    printf '\033[2J\033[H'
    printf 'Select mission (Up/Down, Enter):\n\n'
    for ((index = 0; index < ${#mission_options[@]}; index++)); do
      if [[ "$index" -eq "$selected_index" ]]; then
        printf '  > %s\n' "${mission_options[$index]}"
      else
        printf '    %s\n' "${mission_options[$index]}"
      fi
    done

    IFS= read -rsn1 key
    if [[ "$key" == $'\033' ]]; then
      IFS= read -rsn2 key
      case "$key" in
        '[A') selected_index=$(((selected_index - 1 + ${#mission_options[@]}) % ${#mission_options[@]})) ;;
        '[B') selected_index=$(((selected_index + 1) % ${#mission_options[@]})) ;;
      esac
    elif [[ -z "$key" ]]; then
      break
    fi
  done

  mission_name="${mission_options[$selected_index]}"
  if [[ "$mission_name" == "No mission" ]]; then
    mission_file=""
    run_mission=false
  else
    mission_file="$mission_name"
    run_mission=true
  fi
}

read -r -p "Number of robots [1]: " robot_count
read -r -p "Spawn spacing in meters [2.0]: " spawn_spacing
read -r -p "Localization mode (single/all) [single]: " localization_mode
read -r -p "EKF mode (ICP/IMU_ICP) [ICP]: " ekf_mode

robot_count=${robot_count:-1}
spawn_spacing=${spawn_spacing:-2.0}
localization_mode=${localization_mode:-single}
ekf_mode=${ekf_mode:-ICP}
ekf_mode=${ekf_mode^^}
if [[ "$ekf_mode" != "ICP" && "$ekf_mode" != "IMU_ICP" ]]; then
  echo "Unknown EKF mode '$ekf_mode', expected ICP or IMU_ICP" >&2
  exit 1
fi
select_mission

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select runner_sim
source install/setup.bash

if [[ "$run_mission" == true ]]; then
  mission_file="$PWD/install/runner_sim/share/runner_sim/missions/$mission_file"
fi

setsid ros2 launch runner_sim runner_sim.launch.py \
  robot_count:="$robot_count" \
  spawn_spacing:="$spawn_spacing" \
  localization_mode:="$localization_mode" \
  ekf_mode:="$ekf_mode" \
  run_mission:="$run_mission" \
  mission_file:="$mission_file" \
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
  pkill -TERM -f "$PWD/install/runner_sim/worlds/.*\.sdf" 2>/dev/null || true
  pkill -KILL -f "$PWD/install/runner_sim/worlds/.*\.sdf" 2>/dev/null || true
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