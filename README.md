# pandy_mobile 项目说明

pandy_mobile是一款基于一些开源实现二次开发的轮足式机器人，通过机械结构调整和引入强化学习控制，提高复杂地形下的机动性、避障与转向能力。本仓库聚焦于 ROS 2 控制栈、强化学习控制器以及仿真/实物对接方案，配合机械与硬件资料共同支撑研发。

## 项目定位与现状
- 机械侧提供可折叠轮足、12 个关节 + 4 个轮端的自由度设计，并给出 CNC 可制造零件 (`Mechanical/`) 与硬件 BOM（`fig/Hardware_Device_List.png`）。
- 软件侧围绕 ROS 2 控制框架搭建：包含控制器插件、命令输入节点、硬件接口以及 MuJoCo 模型。
- 控制策略采用两阶段强化学习（Isaac Gym → MuJoCo → 实机），形成可用于仿真与实物的推理管线。
- 当前重点在于 `rl_quadruped_controller`，借助 TorchScript 模型在实时控制线程中输出关节力矩、位置与阻尼指令。

## 系统架构
### 机械与硬件
- `Mechanical/`：包含全部需要外协加工的 STL 文件。
- 控制电脑（Onboard PC）经由 CAN/以太网与驱动器通信，IMU、力传感器等数据通过 ROS 2 话题进入控制层（详见硬件列表图）。

### 软件栈
1. **命令层（Commands）**  
   - `code/Mdog/commands/control_input_msgs` 定义 `Inputs` ROS 2 消息格式。  
   - `code/Mdog/commands/keyboard_input` / `joystick_input` 将键盘或手柄事件映射为速度、姿态及模式切换命令。
2. **控制层（Controllers）**  
   - `rl_quadruped_controller`：主控制器插件，内含有限状态机 `FSM`（被动、固定支撑、固定站立、RL 模式）。  
   - `leg_pd_controller`、`path_tracking_controller`、`unitree_guide_controller` 为其他控制策略或辅助控制器。  
   - 每个控制器作为 `ros2_control` 插件向控制管理器注册，订阅 `/control_input` 与 `/HighCmd` 消息，发布关节指令。
3. **硬件层（Hardwares）**  
   - `mdog_hardware_interface`：实现 `ros2_control` SystemInterface，通过 `LowCmd/LowState/IMU_data` 话题与底层驱动通信。  
   - `hardware_unitree_mujoco`：适配在线仿真的硬件抽象。  
   - `hipnuc_imu`、`motor`：封装 IMU 与电机的 ROS 2 接入。
4. **模型与描述（Descriptions & XML）**  
   - `code/Mdog/descriptions`：机器人 URDF 与可视化资源。  
   - `code/xml/mdog`：MuJoCo 机器人模型，供仿真加载。

### 强化学习控制器工作流
1. 启动时读取 `ros2_control` 参数，关联全部关节/传感器接口并创建 FSM 状态对象。  
2. `StateRL` 加载 TorchScript 模型（`config/<model_folder>/<model_name>`），维护历史观测并在后台线程中以 `frequency / decimation` 频率推理。  
3. `computeObservation()` 将 IMU、关节状态、命令等拼接为张量，经策略输出期望力矩/位置，随后写回至硬件接口。  
4. 当检测到错误或用户切换模式时，FSM 会跳转至被动/固定站立状态，保证安全性。

## 目录速览
- `Mechanical/`：整机加工件 STL。  
- `code/Mdog/commands/`：命令输入节点与自定义消息。  
- `code/Mdog/controllers/`：各类 ros2_control 控制器（RL、PD、路径跟踪等）。  
- `code/Mdog/descriptions/`：URDF 和模型配置。  
- `code/Mdog/hardwares/`：实机及仿真硬件接口。  
- `code/xml/`：MuJoCo 机器人模型。  
- `fig/`：系统框架、硬件清单以及实验 GIF。  
- `Mechanical/`：整机结构件文件。

## 环境准备
- 操作系统：Ubuntu 22.04  
- ROS 发行版：ROS 2 Humble  
- 强化学习推理：libtorch（C++11 ABI，对应 CPU 或 CUDA 版本）  
- 依赖工具：`ros2_control`、`ros2_controllers`、MuJoCo（Unitree 版本）  
- 建议通过 `rosdep` 自动安装其余 ROS 依赖。

### 安装 libtorch（示例）
```bash
cd ~/CLionProjects
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
echo 'export Torch_DIR=~/CLionProjects/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/CLionProjects/libtorch/lib' >> ~/.bashrc
```

### 安装 ros2_control
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## 快速开始
1. 将 `code/Mdog` 拷贝/链接至 `~/ros2_ws/src`。  
2. 解决依赖并编译：
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-up-to \
     rl_quadruped_controller mdog_description keyboard_input hardware_unitree_mujoco \
     --symlink-install
   ```
3. 安装并配置 MuJoCo（参考 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)）：  
   - 拷贝 `code/xml/mdog` 至 `unitree_mujoco/unitree_robots/`。  
   - 在 `unitree_mujoco/simulate/config.yaml` 中设置 `robot: "mdog"`。
4. 启动仿真：
   ```bash
   # 终端 A：运行 MuJoCo
   cd ~/unitree_mujoco/simulate/build
   ./unitree_mujoco

   # 终端 B：启动 RL 控制器
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch rl_quadruped_controller mujoco.launch.py

   # 终端 C：启动键盘输入
   cd ~/ros2_ws
   source install/setup.bash
   ros2 run keyboard_input keyboard_input
   ```
5. 控制方式：  
   - 模式切换：`1` 被动模式、`2` 固定站立、`3` RL 模式、`0` 切换高层命令来源。  
   - 行走控制：`W/S` 前后、`A/D` 横向、`I/K` 俯仰、`J/L` 偏航；空格键归零速度。  
   - 控制节点默认以 10 kHz 以内轮询键盘，无输入时保持上一指令并自动衰减。

## 后续迭代与优化方向
1. **模型/配置管理**：梳理 `model_folder`、YAML 配置与 ROS 参数，补充示例模型及其来源说明，便于复现实验。  
2. **仿真-实机一致性**：统一 `hardware_unitree_mujoco` 与 `mdog_hardware_interface` 的主题命名与单位，构建集成测试确保策略可无缝切换。  
3. **安全与故障监控**：扩展 FSM，对驱动错误、IMU 失联等异常提供自动降级与日志上报。  
4. **命令接口扩展**：提供更高层路径跟踪/导航接口，或整合 ROS Nav2/行为树以支持自主任务。  
5. **CI 与代码质量**：添加格式化、静态检查与单元测试，尤其覆盖控制器关键模块（观测归一化、线程安全）。  
6. **文档与可视化**：补充流程图、参数表及仿真演示脚本，使新成员能够快速搭建和调试。

## 参考资源
- `fig/system_framework.png`：训练与部署流程概览。  
- `fig/Hardware_Device_List.png`：整机硬件清单。  
- `Mechanical/`：零件 CAD/STL，便于快速迭代结构设计。  
