# Quadruped ROS2 Control

This repository contains the ros2-control based controllers for the quadruped robot.

* [Controllers](Mdog/controllers): contains the ros2-control controllers
* [Commands](Mdog/commands): contains command node used to send command to the controller
* [Descriptions](Mdog/descriptions): contains the urdf model of the robot
* [Hardwares](Mdog/ardwares): contains the ros2-control hardware interface for the robot

# RL Controller
Tested environment:
* Ubuntu 22.04
    * ROS2 Humble

## Quick Start

### Installing libtorch

> You can also choose `libtorch` with cuda. Just remember to download for c++ 11 ABI version. The position to place `libtorch` is also not fixed, just need to config the `.bashrc`.

```bash
cd ~/CLionProjects/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
```

```bash
cd ~
rm -rf libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib' >> ~/.bashrc
```

### Installing ros2_control
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### Build the Controller
1. Put the `Mdog` file in the `src` file in your ros2_ws.
2. Install dependency and compile the package
* rosdep
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Compile the package
    ```bash
    colcon build --packages-up-to rl_quadruped_controller mdog_description keyboard_input hardware_unitree_mujoco --symlink-install
    ```

### Install Mujuco
1. please refer to [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) for C++ version installation details.
2. Copy [mdog](xml/mdog) file to the `unitree_mujoco/unitree_robots"` file.
3. Change to `robot:"mdog"` type in `unitree_mujoco/simulate/config.yaml`

### Start Simulation
1. Start Mujuco first.
2. Launch the rl_controller
   ```bash
   cd ~/ros2_ws
   source install/setup.bash 
   ros2 launch rl_quadruped_controller mujoco.launch.py
   ```
3. Open another terminal and run the `keyboard_control` node
   ```bash
   cd ~/ros2_ws
   source install/setup.bash 
   ros2 run keyboard_input keyboard_input
   ```

### Instructions
#### 1.1 Control Mode
* Passive Mode: Keyboard 1
* Fixed Stand: Keyboard 2
    * RL mode: Keyboard 3
#### 1.2 Control Input
* WASD IJKL: Move robot
* Space: Reset Speed Input