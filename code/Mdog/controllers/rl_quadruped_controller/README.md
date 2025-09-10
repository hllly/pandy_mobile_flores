# RL Quadruped Controller

This repository contains the reinforcement learning based controllers for the quadruped robot.

Tested environment:
* Ubuntu 22.04
  * ROS2 Humble

## 2. Build
### 2.1 Installing libtorch
```bash
cd ~/CLionProjects/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
```
```bash
cd ~/CLionProjects/
rm -rf libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
echo 'export Torch_DIR=~/CLionProjects/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/CLionProjects/libtorch/lib' >> ~/.bashrc
```

### 2.2 Build Controller
```bash
cd ~/ros2_ws
colcon build --packages-up-to rl_quadruped_controller
```

## 3. Launch

### 3.1 Mujoco Simulation

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller mujoco.launch.py
```