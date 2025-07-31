#!/bin/bash

# ==============================================================================
# Setting up University of Bristol FSAI device
# ==============================================================================

set -e

FSAI_USER=$(ls /home | grep -v root | head -n 1)
CORE_SIM_REPO="git@github.com:Formula-Student-AI/core-sim.git"
LAUNCH_REPO="git@github.com:Formula-Student-AI/launch.git"
GIT_EMAIL="bristol.fsai@gmail.com"
GIT_USERNAME="bristol-fsai"
WORKSPACE_DIR="/home/$FSAI_USER"
STATE_FILE="/root/.ssd_setup"

# --- Helper Functions ---
print_info() { echo -e "\e[34m[INFO] $1\e[0m"; }
print_success() { echo -e "\e[32m[SUCCESS] $1\e[0m"; }
print_warning() { echo -e "\e[33m[WARNING] $1\e[0m"; }
print_alert() { echo -e "\e[31m[ALERT] $1\e[0m"; }
print_action() { echo -e "\n\e[31m[ACTION REQUIRED] $1\e[0m"; }
print_stage() { echo -e "\e[1;34m\n================================\n$1\n================================\e[0m\n"; }

# --- Stage 1: Initial System and ROS Setup ---
run_stage_1() {
    print_stage "STAGE 1: System, ROS 2, and Optional NVIDIA Setup"

    # --- Initial Setup ---
    print_info "Updating system packages..."
    apt-get update > /dev/null && apt-get upgrade -y

    print_info "Setting locale to en_US.UTF-8..."
    apt-get install -y locales > /dev/null
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    print_info "Installing SSH..."
    apt-get install -y openssh-server > /dev/null
    ufw allow ssh
    systemctl disable ssh # Manually start SSH when needed

    print_info "Installing can-utils..."
    apt-get install -y can-utils > /dev/null

    print_info "Installing Visual Studio Code..."
    apt-get install -y wget gpg apt-transport-https > /dev/null
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /usr/share/keyrings/packages.microsoft.gpg

    # echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | tee /etc/apt/sources.list.d/vscode.list > /dev/null
    # Remove any existing conflicting VS Code sources
    rm -f /etc/apt/sources.list.d/*code*.list

    # Add Microsoft repo with consistent keyring
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /usr/share/keyrings/packages.microsoft.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
        | tee /etc/apt/sources.list.d/vscode.list > /dev/null

        apt-get update > /dev/null
    apt-get install -y code > /dev/null
    print_success "Visual Studio Code installed successfully."

    print_info "Adding required repositories and installing ROS 2 Galactic..."
    apt-get install -y software-properties-common curl > /dev/null
    add-apt-repository universe
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    mkdir -p /etc/apt/sources.list.d
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    apt-get update > /dev/null
    apt-get install -y ros-galactic-desktop ros-dev-tools > /dev/null

    print_info "Sourcing ROS 2 environment in user's .bashrc..."
    if ! grep -q "source /opt/ros/galactic/setup.bash" /home/$FSAI_USER/.bashrc; then
      echo "source /opt/ros/galactic/setup.bash" >> /home/$FSAI_USER/.bashrc
    fi

    # --- SSH Key Check and Setup (run as the target user) ---
    print_info "Checking SSH key for GitHub as user '$FSAI_USER'..."
    su - ${FSAI_USER} -c "
        if [ ! -f ~/.ssh/id_ed25519 ]; then
            echo 'Generating new SSH key...'
            ssh-keygen -t ed25519 -C '$GIT_EMAIL' -N '' -f ~/.ssh/id_ed25519
        fi
        
        eval \$(ssh-agent -s) > /dev/null
        ssh-add ~/.ssh/id_ed25519 > /dev/null

        echo 'Testing SSH connection to GitHub...'
        AUTH_RESPONSE=\$(ssh -o StrictHostKeyChecking=no -T git@github.com 2>&1 || true)

        if [[ \"\$AUTH_RESPONSE\" == *\"successfully authenticated\"* ]]; then
            echo -e \"\e[32m[SUCCESS] SSH key is already configured on GitHub.\e[0m\"
        else
            echo -e \"\e[33m[WARNING] SSH key is not yet configured on GitHub.\e[0m\"
            echo -e \"\n\e[31m[ACTION REQUIRED] Please add the following public SSH key to your GitHub account:\e[0m\"
            echo -e \"\n--- Your Public SSH Key ---\n\$(cat ~/.ssh/id_ed25519.pub)\n--------------------------\"
            read -p \"Press [Enter] to continue after adding the key to GitHub...\"
        fi

        git config --global user.name '$GIT_USERNAME'
        git config --global user.email '$GIT_EMAIL'
    "

    print_info "Checking for core-sim repository..."
    cd "$WORKSPACE_DIR"

    if [ ! -d "core-sim" ]; then
        print_info "Cloning core-sim repository..."
        su - ${FSAI_USER} -c "git clone $CORE_SIM_REPO && cd core-sim && git switch dev"
    else
        print_info "core-sim directory found. Pulling latest changes..."candump
        chown -R ${FSAI_USER}:${FSAI_USER} core-sim
        su - ${FSAI_USER} -c "cd core-sim && git switch dev && git pull"
    fi

    EUFS_MASTER_PATH="$WORKSPACE_DIR/core-sim"
    if ! grep -q "export EUFS_MASTER" /home/$FSAI_USER/.bashrc; then
      echo "export EUFS_MASTER=$EUFS_MASTER_PATH" >> /home/$FSAI_USER/.bashrc
    fi
    export EUFS_MASTER=$EUFS_MASTER_PATH

    print_info "Cloning FSAI Launch repository to set up rc.local service..."
    LAUNCH_CLONE_DIR="/home/$FSAI_USER/launch"
    rm -rf "$LAUNCH_CLONE_DIR"
    su - ${FSAI_USER} -c "git clone $LAUNCH_REPO $LAUNCH_CLONE_DIR"
    
    print_info "Copying rc.local file to /etc/ and making it executable..."
    cp "$LAUNCH_CLONE_DIR/rc.local" "/etc/rc.local"
    chmod +x /etc/rc.local
    print_success "rc.local service file has been configured."

    print_info "Installing additional ROS 2 dependencies..."
    apt-get install libgazebo11-dev ros-galactic-gazebo-ros-pkgs ros-galactic-joint-state-publisher ros-galactic-xacro ros-galactic-ackermann-msgs -y > /dev/null
    apt-get install ros-galactic-gazebo-plugins libyaml-cpp-dev ros-galactic-rqt ros-galactic-rqt-common-plugins ros-galactic-ament-package -y > /dev/null
    print_success "ROS 2 Galactic and dependencies installed successfully."

    print_info "Installing colcon and rosdep..."
    apt-get install python3-pip -y > /dev/null
    su - ${FSAI_USER} -c "pip3 install colcon-common-extensions -U"
    apt-get install python3-rosdep -y > /dev/null
    
    print_info "Initializing rosdep..."
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list
    rosdep init
    su - ${FSAI_USER} -c "rosdep update"

    print_info "Installing additional Python dependencies..."
    su - ${FSAI_USER} -c "source /opt/ros/galactic/setup.bash && rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y"
    su - ${FSAI_USER} -c "pip install -r $EUFS_MASTER/eufs_sim/perception/requirements.txt"
    su - ${FSAI_USER} -c "pip install --upgrade numpy"
    apt-get install ros-galactic-vision-msgs -y > /dev/null
    print_success "Python dependencies installed successfully."

    print_info "Building and sourcing core-sim (excluding ZED packages)..."
    su - ${FSAI_USER} -c "cd $WORKSPACE_DIR/core-sim && source /opt/ros/galactic/setup.bash && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --packages-ignore-regex='^(zed).*'"
    print_success "Successfully built core-sim"

    # --- NVIDIA Driver Installation Check ---
    print_info "Checking for a supported NVIDIA GPU..."
    if lspci | grep -i -q nvidia; then
        print_success "NVIDIA GPU detected. Proceeding with full driver, CUDA, and ZED installation."
        
        # --- NVIDIA Driver Installation ---
        print_info "Purging any existing NVIDIA and CUDA installations for a clean setup..."
        apt-get purge --autoremove nvidia* -y > /dev/null
        rm -f /etc/apt/sources.list.d/cuda*
        apt-get autoremove -y > /dev/null && apt-get autoclean -y > /dev/null
        rm -rf /usr/local/cuda*
        print_success "Old NVIDIA/CUDA packages removed."

        print_info "Fixing any broken package dependencies..."
        apt-get --fix-broken install -y > /dev/null

        print_info "Installing NVIDIA Drivers..."
        apt-get update > /dev/null
        apt-get install -y ubuntu-drivers-common > /dev/null
        print_info "Recommended drivers for your system:"
        ubuntu-drivers devices
        print_alert "The script will now install the recommended drivers using 'autoinstall'."
        ubuntu-drivers autoinstall > /dev/null
        prime-select intel

        # --- Prepare for next stage ---
        echo "STAGE_2" > "$STATE_FILE"
        print_action "NVIDIA drivers are installed. A system reboot is required."
        print_info "After rebooting, please run this script again to continue."
        read -p "Press [Enter] to reboot or Ctrl+C to cancel and reboot later"
        reboot
    else
        # --- No NVIDIA GPU Found ---
        print_alert "No NVIDIA GPU detected. Skipping NVIDIA, CUDA, and ZED SDK installation."
        print_info "Setup will complete for a non-GPU configuration."
        
        print_info "Sourcing the workspace overlay in user's .bashrc..."
        BASHRC_PATH="/home/$FSAI_USER/.bashrc"
        if ! grep -q "source $WORKSPACE_DIR/core-sim/install/setup.bash" $BASHRC_PATH; then
          echo "source $WORKSPACE_DIR/core-sim/install/setup.bash" >> $BASHRC_PATH
        fi

        # --- Cleanup ---
        rm -f "$STATE_FILE"
        print_success "🎉 All stages complete for non-GPU setup! Your environment is ready. 🎉"
        print_info "Open a NEW terminal as user '$FSAI_USER' to ensure all environment variables are loaded."
        exit 0 # Exit successfully, as no further stages are needed.
    fi
}

# --- Stage 2: CUDA and ZED SDK Installation ---
run_stage_2() {
    print_stage "STAGE 2: CUDA and ZED SDK Installation"

    print_alert "Don't terminate this while selecting NVIDIA"
    prime-select nvidia
    modprobe nvidia
    print_success "Successfully selected NVIDIA"

    print_info "Verifying NVIDIA driver installation..."
    if ! nvidia-smi; then
        print_warning "nvidia-smi command failed. Driver may not be installed correctly."
        exit 1
    fi
    print_success "NVIDIA drivers are active."

    if prime-select query | grep -q "intel"; then
        print_action "Your system is using Intel graphics. To use NVIDIA, run 'sudo prime-select nvidia' and re-run this script."
        exit 0
    fi

    # --- Check for existing CUDA installation ---
    if command -v nvcc &> /dev/null; then
        print_success "CUDA Toolkit is already installed. Skipping installation."
        print_info "Detected version:"
        nvcc --version
    else
        print_info "CUDA Toolkit not found. Installing NVIDIA CUDA Toolkit..."
        wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
        mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
        wget -c https://developer.download.nvidia.com/compute/cuda/12.9.1/local_installers/cuda-repo-ubuntu2004-12-9-local_12.9.1-575.57.08-1_amd64.deb
        dpkg -i cuda-repo-ubuntu2004-12-9-local_12.9.1-575.57.08-1_amd64.deb
        cp /var/cuda-repo-ubuntu2004-12-9-local/cuda-*-keyring.gpg /usr/share/keyrings/
        apt-get update > /dev/null
        apt-get -y install cuda-toolkit-12-9 > /dev/null
        print_success "CUDA Toolkit installed successfully."
    fi

    # --- Automated ZED SDK Download and Installation ---
    print_info "Downloading ZED SDK v5.0 for CUDA..."
    wget -c "https://download.stereolabs.com/zedsdk/5.0/cu12/ubuntu20" -P "/home/$FSAI_USER/Downloads/"
    ZED_INSTALLER_PATH="/home/$FSAI_USER/Downloads/ubuntu20"
    
    if [ -z "$ZED_INSTALLER_PATH" ]; then
        print_warning "ZED SDK installer download failed or not found."
        exit 1
    fi

    print_success "Found ZED installer: $ZED_INSTALLER_PATH"
    print_info "Installing dependencies and running the installer..."
    apt-get install -y zstd > /dev/null
    chmod +x "$ZED_INSTALLER_PATH"
    su - $FSAI_USER -c "'$ZED_INSTALLER_PATH' -- silent"

    echo "STAGE_3" > "$STATE_FILE"
    print_action "ZED SDK is installed. A final system reboot is required."
    print_info "After rebooting, please run this script one last time."
    read -p "Press [Enter] to reboot or Ctrl+C to cancel and reboot later"
    reboot
}

# --- Stage 3: Final Configuration and Build ---
run_stage_3() {
    EUFS_MASTER_PATH="$WORKSPACE_DIR/core-sim"
    print_stage "STAGE 3: Final Configuration and Build"

    print_stage "Setting up permissions for zed folder"

    print_action "Please plug in your ZED 2 camera to a USB 3.0 port."
    print_info "You can test the installation by running: /usr/local/zed/tools/ZED_Diagnostic"
    read -p "Press [Enter] to continue once you have tested the camera..."

    print_info "Installing final ROS dependencies..."
    apt-get install -y ros-galactic-backward-ros ros-galactic-diagnostic-updater ros-galactic-geographic-msgs ros-galactic-robot-localization > /dev/null

    print_info "Updating and installing all workspace dependencies with rosdep..."
    su - "${FSAI_USER}" -c "source /opt/ros/galactic/setup.bash"
    rosdep install --from-paths $EUFS_MASTER_PATH --ignore-src -r -y

    print_info "Building the complete workspace (as sudo)..."
    cd $WORKSPACE_DIR/core-sim
    source /opt/ros/galactic/setup.bash
    source $EUFS_MASTER_PATH/install/setup.bash
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

    print_info "Sourcing the final workspace overlay in user's .bashrc..."
    BASHRC_PATH="/home/$FSAI_USER/.bashrc"
    if ! grep -q "source $WORKSPACE_DIR/core-sim/install/setup.bash" $BASHRC_PATH; then
      echo "source $WORKSPACE_DIR/core-sim/install/setup.bash" >> $BASHRC_PATH
    fi

    print_info "Configuring git credentials for user '$FSAI_USER'..."
    su - ${FSAI_USER} -c "git config --global user.name '$GIT_USERNAME' && git config --global user.email '$GIT_EMAIL'"

    # --- Cleanup ---
    rm -f "$STATE_FILE"
    print_success "All stages complete! Your environment is ready. 🎉"
    print_info "Open a NEW terminal as user '$FSAI_USER' to ensure all environment variables are loaded."
}

# Check for sudo privileges
if [ "$EUID" -ne 0 ]; then
  print_warning "This script must be run as root."
  echo "Please run with sudo: sudo $0"
  exit 1
fi

if [ -z "$FSAI_USER" ]; then
    print_warning "Could not determine a non-root user in /home."
    print_info "Please ensure a user account exists before running this script. If you have a user account, please set the FSAI_USER variable manually at the top of this script."
    exit 1
fi

if [ ! -f "$STATE_FILE" ]; then
    run_stage_1
elif [ "$(cat "$STATE_FILE")" = "STAGE_2" ]; then
    run_stage_2
elif [ "$(cat "$STATE_FILE")" = "STAGE_3" ]; then
    run_stage_3
else
    print_warning "Invalid state found in $STATE_FILE. Removing state file."
    rm -f "$STATE_FILE"
    run_stage_1
fi
