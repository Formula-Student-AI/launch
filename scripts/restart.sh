#!/bin/bash

# --- Configuration ---
# Define the unique command patterns for each process.
EUFS_CMD="ros2 launch eufs_launcher hardware.launch.py"
ZED_CMD="ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2"
CAN_CMD="candump can0" # Command pattern for the CAN logger

# Define the workspace and log directory.
WORKSPACE_DIR="/home/bristol-fsai/core-sim"
LOG_DIR="/home/bristol-fsai/logs"
# --- End Configuration ---


# Helper function to terminate a process by its command pattern.
kill_process() {
  local pattern="$1"
  local name="$2" # A friendly name for logging.
  
  echo "🔎  Checking for '$name' process..."
  if pgrep -f "$pattern" > /dev/null; then
    echo "⛔  Terminating '$name'..."
    pkill -f "$pattern"
    sleep 2 # Allow time for graceful shutdown.

    # If it's still alive, force the kill.
    if pgrep -f "$pattern" > /dev/null; then
      echo "⚠️  Forcing kill on '$name' (SIGKILL)..."
      pkill -9 -f "$pattern"
      sleep 1
    fi
    echo "✅  '$name' process stopped."
  else
    echo "👍  No running '$name' process found."
  fi
}

# Helper function to start a ROS process and verify its launch.
start_ros_process() {
  local command="$1"
  local name="$2"
  local log_base_name="$3"

  echo "🚀  Starting '$name' process..."
  local log_file="${LOG_DIR}/${log_base_name}_$(date +%Y-%m-%d_%H-%M-%S).log"

  # Use nohup to run in the background and survive terminal closure.
  nohup $command >> "$log_file" 2>&1 &
  
  sleep 3 # Give the process a few seconds to initialize.

  # Verify it started by checking for its PID.
  if pgrep -f "$command" > /dev/null; then
    echo "✅  Success! '$name' is running. Logging to: $log_file"
  else
    echo "❌  Error! Failed to start '$name'. Check logs in $LOG_DIR."
  fi
}

# Helper function to start the CAN logger.
start_can_logger() {
    echo "WIP - manually restart candump for now"
}


# --- Main Execution ---
echo "🔄  Restarting all nodes and loggers..."

# --- 1. ENVIRONMENT AND CODE SETUP ---
echo
echo "--- Sourcing ROS environment ---"
# Source the necessary ROS environment setup files.
source /opt/ros/galactic/setup.bash
echo "✅  ROS Galactic environment sourced."
echo "--------------------------------"

# Navigate to the workspace directory.
cd "$WORKSPACE_DIR" || { echo "❌ Error: Could not navigate to $WORKSPACE_DIR. Exiting."; exit 1; }

echo
echo "--- Updating and Building Workspace ---"
echo "⬇️  Pulling latest changes from git..."
git pull
echo "✅  Git pull complete."
echo

echo "🛠️  Building workspace with colcon..."
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo "✅  Colcon build complete."
echo

echo "📦  Sourcing the local workspace..."
source install/setup.bash
echo "✅  Workspace environment ready."
echo "--------------------------------"


# Ensure the log directory exists.
mkdir -p "$LOG_DIR"

# --- 2. TERMINATION PHASE ---
echo
echo "--- Stopping all processes first ---"
kill_process "$EUFS_CMD" "EUFS"
kill_process "$ZED_CMD" "ZED Camera"
# kill_process "$CAN_CMD" "CAN Logger"
echo "--------------------------------"


# --- 3. LAUNCH PHASE ---
echo
echo "--- Starting all processes now ---"
start_ros_process "$EUFS_CMD" "EUFS" "ros1"
start_ros_process "$ZED_CMD" "ZED Camera" "ros2"
start_can_logger # Start the CAN logger
echo "------------------------------"
echo "✅  Restart sequence complete."

exit 0
