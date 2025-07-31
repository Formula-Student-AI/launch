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


# --- Helper Functions for Script Output ---
print_info() { echo -e "\e[34m[INFO] $1\e[0m"; }
print_success() { echo -e "\e[32m[SUCCESS] $1\e[0m"; }
print_warning() { echo -e "\e[33m[WARNING] $1\e[0m"; }
print_alert() { echo -e "\e[31m[ALERT] $1\e[0m"; }
print_action() { echo -e "\n\e[31m[ACTION REQUIRED] $1\e[0m"; }
print_stage() { echo -e "\e[1;34m\n================================\n$1\n================================\e[0m"; }
# --- End Helper Functions ---


# --- Argument Parsing ---
# Default to not rebuilding.
REBUILD=false
# Check if the first argument is '-r'.
if [[ "$1" == "-r" ]]; then
  REBUILD=true
  print_info "Rebuild flag '-r' detected. Workspace will be updated and rebuilt."
fi
# --- End Argument Parsing ---


kill_ros_processes() {
  local pattern="ros"
  local name="ROS / ROS 2"

  print_info "Checking for '$name' processes..."
  if pgrep -f "$pattern" > /dev/null; then
    print_warning "Terminating all '$name' processes..."
    pkill -f "$pattern"
    sleep 2 # Allow time for graceful shutdown.

    # If any are still alive, force the kill.
    if pgrep -f "$pattern" > /dev/null; then
      print_alert "Forcing kill on remaining '$name' processes (SIGKILL)..."
      pkill -9 -f "$pattern"
      sleep 1
    fi
    print_success "All '$name' processes stopped."
  else
    print_success "No running '$name' processes found."
  fi
}

# Helper function to start a ROS process and verify its launch.
start_ros_process() {
  local command="$1"
  local name="$2"
  local log_base_name="$3"

  print_info "Starting '$name' process..."
  local log_file="${LOG_DIR}/${log_base_name}_$(date +%Y-%m-%d_%H-%M-%S).log"

  # Use nohup to run in the background and survive terminal closure.
  nohup $command >> "$log_file" 2>&1 &
  
  sleep 3 # Give the process a few seconds to initialize.

  # Verify it started by checking for its PID.
  if pgrep -f "$command" > /dev/null; then
    print_success "Success! '$name' is running. Logging to: $log_file"
  else
    print_alert "Error! Failed to start '$name'. Check logs in $LOG_DIR."
  fi
}

# Helper function to start the CAN logger.
start_can_logger() {
    print_warning "CAN logger start is a WIP - manually restart candump for now."
}


# --- Main Execution ---
print_info "Restarting all nodes and loggers..."

# --- 1. ENVIRONMENT AND CODE SETUP ---
print_stage "Sourcing ROS Environment"
# Source the necessary ROS environment setup files.
source /opt/ros/galactic/setup.bash
print_success "ROS Galactic environment sourced."

# Navigate to the workspace directory.
cd "$WORKSPACE_DIR" || { print_alert "Could not navigate to $WORKSPACE_DIR. Exiting."; exit 1; }

# Conditionally update and build the workspace if -r flag was passed.
if [ "$REBUILD" = true ]; then
  print_stage "Updating and Building Workspace"
  print_info "Pulling latest changes from git..."
  git pull
  print_success "Git pull complete."
  
  print_info "Building workspace with colcon..."
  colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
  print_success "Colcon build complete."
else
  print_stage "Skipping Rebuild (use -r to force)"
fi

print_info "Sourcing the local workspace..."
source install/setup.bash
print_success "Workspace environment ready."


# Ensure the log directory exists.
mkdir -p "$LOG_DIR"

# --- 2. TERMINATION PHASE ---
print_stage "Stopping All Processes"
kill_ros_processes # Stop all ROS processes
# kill_process "$CAN_CMD" "CAN Logger"


# --- 3. LAUNCH PHASE ---
print_stage "Starting All Processes"
start_ros_process "$EUFS_CMD" "EUFS" "ros1"
start_ros_process "$ZED_CMD" "ZED Camera" "ros2"
start_can_logger # Start the CAN logger

print_success "Restart sequence complete."

exit 0
