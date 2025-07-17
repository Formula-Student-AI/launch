#!/bin/bash

# ==============================================================================
# EUFS Full Setup Script with NVIDIA, CUDA, and ZED Support
# ==============================================================================
# This is a multi-stage script. It will prompt you to reboot and then
# you must re-run it to continue the installation.
# ==============================================================================

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
CORE_SIM_REPO="git@github.com:Formula-Student-AI/core-sim.git"
LAUNCH_REPO="git@github.com:Formula-Student-AI/launch.git"
GIT_EMAIL="bristol.fsai@gmail.com"
GIT_USERNAME="bristol-fsai"
WORKSPACE_DIR="$HOME/eufs_ws"
STATE_FILE="$HOME/.eufs_setup_state"

# --- Helper Functions ---
print_info() { echo -e "\n\e[34m[INFO] $1\e[0m"; }
print_success() { echo -e "\e[32m[SUCCESS] $1\e[0m"; }
print_warning() { echo -e "\e[33m[WARNING] $1\e[0m"; }
print_action() { echo -e "\n\e[31m[ACTION REQUIRED] $1\e[0m"; }
print_stage() { echo -e "\n\e[35m\n================================\n$1\n================================\e[0m\n"; }

# --- Stage 1: Initial System and ROS Setup ---
run_stage_1() {
    print_stage "STAGE 1: System, ROS 2, and NVIDIA Driver Installation"

    # --- Initial Setup ---
    print_info "Updating system packages..."
    sudo apt update && sudo apt upgrade -y

    print_info "Setting locale to en_US.UTF-8..."
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    print_info "Adding required repositories and installing ROS 2 Galactic..."
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-galactic-desktop ros-dev-tools

    print_info "Sourcing ROS 2 environment in .bashrc..."
    if ! grep -q "source /opt/ros/galactic/setup.bash" ~/.bashrc; then
      echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
    fi

    print_info "Setting up SSH key for GitHub..."
    if [ ! -f ~/.ssh/id_ed25519 ]; then
        ssh-keygen -t ed25519 -C "$GIT_EMAIL" -N "" -f ~/.ssh/id_ed25519
    else
        print_warning "Existing SSH key found. Skipping generation."
    fi
    eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519
    print_action "Please add the following public SSH key to your GitHub account."
    echo -e "\n--- Your Public SSH Key ---\n$(cat ~/.ssh/id_ed25519.pub)\n--------------------------"
    read -p "Press [Enter] to continue after adding the key to GitHub..."

    print_info "Creating workspace and cloning core-sim..."
    mkdir -p "$WORKSPACE_DIR/src" && cd "$WORKSPACE_DIR/src"
    git clone "$CORE_SIM_REPO"
    EUFS_MASTER_PATH="$WORKSPACE_DIR/src/core-sim"
    if ! grep -q "export EUFS_MASTER" ~/.bashrc; then
      echo "export EUFS_MASTER=$EUFS_MASTER_PATH" >> ~/.bashrc
    fi
    export EUFS_MASTER=$EUFS_MASTER_PATH

    # --- NVIDIA Driver Installation ---
    print_info "Installing NVIDIA Drivers..."
    sudo apt install -y ubuntu-drivers-common
    print_info "Recommended drivers for your system:"
    ubuntu-drivers devices
    print_action "The script will now install the recommended drivers using 'autoinstall'."
    sudo ubuntu-drivers autoinstall

    print_warning "If your system has both Intel and NVIDIA graphics (especially a newer GPU), it may crash on reboot."
    print_action "Selecting Intel graphics before reboot to prevent a crash."
    print_warning "After rebooting, you will be reminded to switch back."
    sudo prime-select intel

    # --- Prepare for next stage ---
    echo "STAGE_2" > "$STATE_FILE"
    print_action "NVIDIA drivers are installed. A system reboot is required."
    print_info "After rebooting, please run this script again to continue."
    sudo reboot
}

# --- Stage 2: CUDA and ZED SDK Installation ---
run_stage_2() {
    print_stage "STAGE 2: CUDA and ZED SDK Installation"

    print_info "Verifying NVIDIA driver installation..."
    if ! nvidia-smi; then
        print_warning "nvidia-smi command failed. The driver may not have installed correctly. Please check and try again."
        exit 1
    fi
    print_success "NVIDIA drivers are active."

    # Reminder for dual-graphics users
    if sudo prime-select query | grep -q "intel"; then
        print_action "Your system is currently using Intel graphics. To use the NVIDIA GPU, run the following command and then re-run this script:"
        print_warning "sudo prime-select nvidia"
        exit 0
    fi

    print_info "Installing NVIDIA CUDA Toolkit..."
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
    sudo dpkg -i cuda-keyring_1.1-1_all.deb
    sudo apt-get update
    sudo apt-get -y install cuda-toolkit
    rm cuda-keyring_1.1-1_all.deb

    print_info "Adding CUDA paths to .bashrc..."
    if ! grep -q "cuda/bin" ~/.bashrc; then
        echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
    fi
    if ! grep -q "cuda/lib64" ~/.bashrc; then
        echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
    fi

    print_info "Installing ZED SDK..."
    print_action "Please download the ZED SDK v5.0 for Ubuntu 20.04 from:"
    print_info "https://www.stereolabs.com/developers/release/"
    print_action "Place the downloaded '.run' file in your '~/Downloads' directory."

    ZED_INSTALLER_PATH=$(find ~/Downloads -name "ZED_SDK_Ubuntu20_cuda*.run" | head -n 1)
    while [ -z "$ZED_INSTALLER_PATH" ]; do
        read -p "Press [Enter] once the file is in ~/Downloads..."
        ZED_INSTALLER_PATH=$(find ~/Downloads -name "ZED_SDK_Ubuntu20_cuda*.run" | head -n 1)
    done

    print_info "Found ZED installer: $ZED_INSTALLER_PATH"
    print_info "Installing dependencies and running the installer in silent mode..."
    sudo apt install -y zstd
    chmod +x "$ZED_INSTALLER_PATH"
    "$ZED_INSTALLER_PATH" -- silent

    # --- Prepare for next stage ---
    echo "STAGE_3" > "$STATE_FILE"
    print_action "ZED SDK is installed. A final system reboot is required."
    print_info "After rebooting, please run this script one last time."
    sudo reboot
}

# --- Stage 3: Final Configuration and Build ---
run_stage_3() {
    print_stage "STAGE 3: Final Configuration and Build"

    # --- NEW: Clone launch repo and set up rc.local ---
    print_info "Cloning FSAI Launch repository to set up rc.local service..."
    cd "$HOME"
    # Clean up previous clone if it exists
    rm -rf launch
    git clone "$LAUNCH_REPO"
    print_info "Copying rc.local file to /etc/ and making it executable..."
    sudo cp "$HOME/launch/rc.local" "/etc/rc.local"
    sudo chmod +x /etc/rc.local
    # Clean up the cloned repository
    rm -rf "$HOME/launch"
    print_success "rc.local service file has been configured."
    # --- End of new section ---

    print_action "Please plug in your ZED 2 camera to a USB 3.0 port."
    print_info "You can test the installation by running the ZED Diagnostics tool:"
    print_info "/usr/local/zed/tools/ZED_Diagnostic"
    read -p "Press [Enter] to continue once you have tested the camera..."

    print_info "Installing zed-ros2-wrapper..."
    cd "$WORKSPACE_DIR/src"

    print_action "The 'core-sim' project may already include a compatible ZED wrapper."
    read -p "Do you need to manually clone and patch the zed-ros2-wrapper? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Cloning zed-ros2-wrapper and its dependencies..."
        git clone https://github.com/stereolabs/zed-ros2-wrapper.git
        cd zed-ros2-wrapper
        git clone https://github.com/stereolabs/zed-ros2-interfaces.git
        git clone https://github.com/ros-drivers/nmea_msgs.git
        cd ..

        print_info "Patching wrapper packages for ROS 2 Galactic..."
        ZED_WRAPPER_DIR="$WORKSPACE_DIR/src/zed-ros2-wrapper"
        find "$ZED_WRAPPER_DIR" -type f \( -name "CMakeLists.txt" -o -name "package.xml" \) -exec sed -i 's/foxy/galactic/g' {} +
        find "$ZED_WRAPPER_DIR" -type f \( -name "CMakeLists.txt" -o -name "package.xml" \) -exec sed -i 's/FOUND_FOXY/FOUND_GALACTIC/g' {} +
        print_info "Patching shutdown callback function..."
        find "$ZED_WRAPPER_DIR" -type f \( -name "*.cpp" -o -name "*.hpp" \) -exec sed -i 's/add_pre_shutdown_callback/add_on_shutdown_callback/g' {} +
        print_success "Patching complete."
    fi

    print_info "Installing final ROS dependencies..."
    sudo apt install -y ros-galactic-backward-ros ros-galactic-diagnostic-updater ros-galactic-geographic-msgs ros-galactic-robot-localization

    print_info "Updating and installing all workspace dependencies with rosdep..."
    source /opt/ros/galactic/setup.bash
    cd "$WORKSPACE_DIR"
    rosdep install --from-paths src --ignore-src -r -y

    print_info "Building the complete workspace..."
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

    print_info "Sourcing the final workspace overlay in .bashrc..."
    if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
      echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    fi

    print_info "Configuring git credentials..."
    git config --global user.name "$GIT_USERNAME"
    git config --global user.email "$GIT_EMAIL"

    # --- Cleanup ---
    rm -f "$STATE_FILE"
    print_success "🎉 All stages complete! Your environment is ready. 🎉"
    print_info "Open a NEW terminal to ensure all environment variables are loaded."
}


# ==============================================================================
# SCRIPT EXECUTION LOGIC
# ==============================================================================
if [ ! -f "$STATE_FILE" ]; then
    run_stage_1
elif [ "$(cat "$STATE_FILE")" = "STAGE_2" ]; then
    run_stage_2
elif [ "$(cat "$STATE_FILE")" = "STAGE_3" ]; then
    run_stage_3
else
    print_warning "Invalid state found in $STATE_FILE. Please remove it and start over."
    exit 1
fi