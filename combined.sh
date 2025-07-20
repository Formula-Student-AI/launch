#!/bin/bash

FSAI_USER=$(ls /home | grep -v root | head -n 1)
WORKSPACE_DIR="/home/$FSAI_USER"

set -e

# --- Helper Functions for Script Output ---
print_info() { echo -e "\e[34m[INFO] $1\e[0m"; }
print_success() { echo -e "\e[32m[SUCCESS] $1\e[0m"; }
print_warning() { echo -e "\e[33m[WARNING] $1\e[0m"; }
print_alert() { echo -e "\e[31m[ALERT] $1\e[0m"; }
print_action() { echo -e "\n\e[31m[ACTION REQUIRED] $1\e[0m"; }
print_stage() { echo -e "\e[1;34m\n================================\n$1\n================================\e[0m"; }

# --- Check for Internet Connectivity ---
function check_internet_connection() {
    print_info "Checking for internet connection..."
    if ping -c 1 google.com &>/dev/null; then
        print_success "Internet connection detected. Pulling git changes"
        return 0
    else
        print_warning "No internet connection detected. Skipping git pull."
        return 1
    fi
}

# --- Core Functions ---

function colcon_build() {
    print_stage "STAGE 1: Setting up Directories & Permissions"
    print_info "Colcon building repo"

    cd "${WORKSPACE_DIR}/core-sim"
    source install/setup.bash
    source /opt/ros/galactic/setup.bash
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

    print_success "Successfully rebuilt"
}

##
# @brief Sets up necessary directories and file permissions.
##
function setup_directories_and_permissions() {    
    print_info "Creating log directory: ${WORKSPACE_DIR}/logs"
    mkdir -p "${WORKSPACE_DIR}/logs"
    
    print_info "Setting ownership for '${FSAI_USER}'"
    chown ${FSAI_USER}:${FSAI_USER} "${WORKSPACE_DIR}/logs"
    
    print_info "Setting permissions on helper scripts"
    chmod +x "${WORKSPACE_DIR}/launch/scripts/restart.sh"
    
    print_success "Directories and permissions are set."
}

##
# @brief Enables NVIDIA drivers and CAN kernel modules.
##
function enable_hardware_modules() {
    print_stage "STAGE 2: Enabling Hardware Modules"
    
    print_info "Selecting NVIDIA drivers and loading module..."
    prime-select nvidia
    modprobe nvidia
    
    print_info "Loading CAN kernel modules..."
    modprobe can_dev
    modprobe can
    modprobe can_raw
    modprobe vcan
    
    print_success "Hardware modules enabled."
}

##
# @brief Configures the CAN0 interface and starts a logger.
# @note Skips gracefully if the can0 interface is not available.
##
function configure_can_interface() {
    print_stage "STAGE 3: Configuring CAN0 Interface"
    
    # Check if the CAN interface exists before trying to configure it
    if ip link set up can0 type can bitrate 500000 &>/dev/null; then
        print_success "CAN0 link is UP."
        print_info "Starting background CAN logger..."
        
        local LOG_TIMESTAMP
        LOG_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        
        # Run candump as the target user
        su - ${FSAI_USER} -c "stdbuf -o0 candump 'can0' > '${WORKSPACE_DIR}/logs/candump_log_${LOG_TIMESTAMP}.log' &"
        print_success "CAN traffic logging initiated to candump_log_${LOG_TIMESTAMP}.log"
    else
        print_warning "CAN interface 'can0' not available or failed to configure. Skipping CAN setup."
    fi
}

##
# @brief Launches the ROS 2 nodes as the specified user.
##
function launch_ros_nodes() {
    print_stage "STAGE 4: Launching ROS Nodes as '${FSAI_USER}'"
    
    # Use a 'heredoc' to execute a multi-line script as the user.
    # By not quoting 'EOF', we pass variables like ${WORKSPACE_DIR} from this script.
    # Variables for the user's shell must be escaped with a backslash (\$).
    su - ${FSAI_USER} <<EOF
# Exit immediately if any command in this user script fails
set -e

print_info() { echo -e "\e[34m[INFO] \$1\e[0m"; } # Re-define for subshell
print_success() { echo -e "\e[32m[SUCCESS] \$1\e[0m"; }

print_info "Sourcing ROS 2 environments..."
source /opt/ros/galactic/setup.bash
source ${WORKSPACE_DIR}/core-sim/install/setup.bash

print_info "Launching ROS applications in the background..."
ROS_LOG_TIMESTAMP=\$(date +%Y-%m-%d_%H-%M-%S)

# Launch EUFS autonomy system and log its output
ros2 launch eufs_launcher hardware.launch.py >> "${WORKSPACE_DIR}/logs/ros_eufs_\${ROS_LOG_TIMESTAMP}.log" 2>&1 &

# Launch ZED camera driver and log its output
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 >> "${WORKSPACE_DIR}/logs/ros_zed_\${ROS_LOG_TIMESTAMP}.log" 2>&1 &

print_success "ROS launch commands dispatched."
EOF
}

##
# @brief Main execution function.
##
function main() {
    print_stage "🚀 Starting FSAI System Launch for user '${FSAI_USER}'"

    # --- Check for Internet Connection and Git Pull ---
    check_internet_connection
    if [ $? -eq 0 ]; then
        print_info "Pulling latest changes from git..."
        cd ${WORKSPACE_DIR}/core-sim || { print_alert "Failed to access workspace directory."; exit 1; }
        git pull || { print_warning "Git pull failed. Continuing without pulling."; }
    fi

    colcon_build

    setup_directories_and_permissions
    enable_hardware_modules
    configure_can_interface
    launch_ros_nodes
    
    print_stage "🏁 LAUNCH SCRIPT COMPLETE"
    print_info "The system is now running in the background."
    print_info "Monitor logs in ${WORKSPACE_DIR}/logs for status."
}

# --- Script Entrypoint ---
if [ "$EUID" -ne 0 ]; then
  print_warning "This script must be run as root."
  echo "Please run with sudo: sudo $0"
  exit 1
fi

main "$@"