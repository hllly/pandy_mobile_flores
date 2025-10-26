// 版权所有 (c) 2022，Stogl Robotics Consulting UG (haftungsbeschränkt)（模板）
//
// 根据 Apache License 2.0 版（下称“许可证”）授权；
// 未遵循许可证条款前不得使用本文件。
// 可在以下地址获取许可证副本：
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 除非适用法律要求或书面同意，否则依据许可证分发的软件
// 均以“按原样”方式提供，不附带任何明示或暗示的担保或条件。
// 更多权限与限制条款请参阅许可证文本。

#include <limits>
#include <vector>

#include "mdog_hardware_interface/mdog_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"


namespace mdog_hardware_interface
{
hardware_interface::CallbackReturn MdogHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(
  "controller_manager.resource_manager.hardware_component.system.MdogHardwareInterface"));
  // TODO(任意贡献者)：读取参数并初始化硬件
  // hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_torque_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_position_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_kp_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_kd_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_error_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  imu_states_.assign(10, 0);

  for (const auto &joint: info_.joints) {
      for (const auto &interface: joint.state_interfaces) {
          joint_interfaces[interface.name].push_back(joint.name);
      }
  }
  
  for (int i=0; i<10; i++){
    imu_states_buffer_[i] = 0;
  }

  for (int i=0; i<16; i++){
    joint_position_temp_[i] = 0;
    joint_velocities_temp_[i] = 0;
    joint_effort_temp_[i] = 0;
    joint_kp_temp_[i] = 0;
    joint_kd_temp_[i] = 0;
    joint_error_temp_[i] = 0;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MdogHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*上一个状态*/)
{
  // TODO(任意贡献者)：准备机器人，使部分接口能够执行读写
  joints_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  this->node_ =  std::make_shared<rclcpp::Node>("hardware_node");
  cmd_publisher_ = this->node_->create_publisher<pd_interfaces::msg::LowCmd>("LowCmd", 1);
  imu_subscriber_ = this->node_->create_subscription<sensor_msgs::msg::Imu>(
    "IMU_data", 1,std::bind(&MdogHardwareInterface::imu_callback, this, std::placeholders::_1));
  state_subscriber_ = this->node_->create_subscription<pd_interfaces::msg::LowState>(
    "LowState", 1,std::bind(&MdogHardwareInterface::low_state_callback, this, std::placeholders::_1));

  // 使用 SingleThreadedExecutor 管理节点
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(this->node_);
  RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "Hardware interface configured successfully.");


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MdogHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  int ind = 0;
    for (const auto &joint_name: joint_interfaces["position"]) {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["velocity"]) {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["effort"]) {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["error"]) {
        state_interfaces.emplace_back(joint_name, "error", &joint_error_[ind++]);
    }

    // 导出 IMU 传感器状态接口
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // // 导出足端力传感器状态接口
    // for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++) {
    //     state_interfaces.emplace_back(
    //         info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
    // }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MdogHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  int ind = 0;
  for (const auto &joint_name: joint_interfaces["position"]) {
      command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name: joint_interfaces["velocity"]) {
      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name: joint_interfaces["effort"]) {
      command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
      command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
      command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
      ind++;
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MdogHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*上一个状态*/)
{
  // TODO(任意贡献者)：让机器人准备好接收指令
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "Hardware interface activated successfully.");
  // joint_position_command_ = joint_position_;
  // joint_velocities_command_ = joint_velocities_;
  // joint_torque_command_ = joint_effort_;

  // joints_states = joint_position_;

  std::thread([this]() {
    // rclcpp::spin(this->node_);
    RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "Spinning executor...");
    executor_->spin();
  }).detach();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MdogHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*上一个状态*/)
{
  // TODO(任意贡献者)：让机器人停止接收指令

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MdogHardwareInterface::read(
  const rclcpp::Time & /*时间*/, const rclcpp::Duration & /*周期*/)
{
  // TODO(任意贡献者)：读取机器人状态
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "--------------------------------------------------");
    // for (int i=0; i<16; i++){
      // joint_position_[i] = joint_position_[i] - joint_offset[i];
      // joint_position_[i] = joint_position_[i] - joint_offset[i];
      // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f",i, joint_position_temp_[i]);
    // }

    // joint_position_[0] = joint_position_temp_[0];
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[4]: %f", joint_position_temp_[4]);
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[14]: %f", joint_position_temp_[14]);

    // 左前（FL）
    int j = 0;
    for (int i=0; i<4; i++){
      joint_position_[j] = joint_position_temp_[i];
      joint_velocities_[j] = joint_velocities_temp_[i];
      joint_effort_[j] = joint_effort_temp_[i];
      joint_error_[j] = joint_error_temp_[i];
      j++;
    }
    joint_position_[3] = (joint_position_temp_[3]-wheel_init_pos_[0]);

    // 右前（FR）
    j = 4;
    for (int i=8; i<12; i++){
      joint_position_[j] = joint_position_temp_[i];
      joint_velocities_[j] = joint_velocities_temp_[i];
      joint_effort_[j] = joint_effort_temp_[i];
      joint_error_[j] = joint_error_temp_[i];
      j++;
    }
    joint_position_[7] = (joint_position_temp_[11]-wheel_init_pos_[1]);

    // 左后（RL）
    j = 8;
    for (int i=12; i<16; i++){
      joint_position_[j] = joint_position_temp_[i];
      joint_velocities_[j] = joint_velocities_temp_[i];
      joint_effort_[j] = joint_effort_temp_[i];
      joint_error_[j] = joint_error_temp_[i];
      j++;
    }
    joint_position_[11] = (joint_position_temp_[15]-wheel_init_pos_[2]);

    // 右后（RR）
    j = 12;
    for (int i=4; i<8; i++){
      joint_position_[j] = joint_position_temp_[i];
      joint_velocities_[j] = joint_velocities_temp_[i];
      joint_effort_[j] = joint_effort_temp_[i];
      joint_error_[j] = joint_error_temp_[i];
      j++;
    }
    joint_position_[15] = (joint_position_temp_[7]-wheel_init_pos_[3]);

    std::array<double,16> direction = { 1.0, 1.0, 1.0, 1.0,  // 左前（FL）
                                        1.0, 1.0, 1.0,-1.0,  // 右前（FR）
                                        1.0, 1.0, 1.0, 1.0,  // 左后（RL）
                                        1.0, 1.0, 1.0,-1.0}; // 右后（RR）

    std::array<double,16> init_offset = { 3.861508,3.218195,0.872212,0.00,    // 左前（FL） 
                                          1.366537,0.924127,2.849465,0.00,    // 右前（FR）
                                          2.975827,6.062500,0.729456,0.00,    // 左后（RL）
                                          1.336193,4.610954,3.986959,0.00,};  // 右后（RR）

    std::array<double,16> torque_const = {0.11577,0.11577,0.11577,0.1162,     // 左前（FL） 
                                          0.11577,0.11577,0.11577,0.1162,     // 右前（FR）
                                          0.11577,0.11577,0.11577,0.1162,     // 左后（RL）
                                          0.11577,0.11577,0.11577,0.1162, };  // 右后（RR）
    for (int i=0; i<16; i++){
      joint_position_[i] = (joint_position_[i] - init_offset[i])*0.1;
      joint_velocities_[i] *= 0.1;
      joint_effort_[i] *= torque_const[i]*40.0*10.0/1000.0;
    }

    for (size_t i = 0; i < joint_position_.size(); ++i) {
      joint_position_[i] *= direction[i];
      joint_velocities_[i] *= direction[i];
      joint_effort_[i] *= direction[i];
    }

    std::array<double,16> joint_offset = {0.0,0.580027778,-1.64361556,0.0,// 左前（FL）
                                          0.0,-0.580027778,1.64361556,0.0,// 右前（FR）
                                          0.0,0.580027778,-1.64361556,0.0,// 左后（RL）
                                          0.0,-0.580027778,1.64361556,0.0,// 右后（RR）
                                          };


    for (int i=0; i<16; i++){
      joint_position_[i] = (joint_position_[i] + joint_offset[i]);
    }

    // 打印所有关节位置
    // for (int i = 0; i < 16; ++i){
    //   RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f", i, joint_position_[i]);
    // }
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f", 4, joint_position_[4]);
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f", 6, joint_position_[6]);
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f", 10, joint_position_[10]);
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f", 14, joint_position_[14]);

    // 打印所有关节力矩
    // for (int i = 0; i < 16; ++i){
    //   RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_effort_[%d]: %f", i, joint_effort_[i]);
    // }
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_effort_[%d]: %f", 5, joint_effort_[5]);
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_effort_[%d]: %f", 9, joint_effort_[9]);
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_effort_[%d]: %f", 13, joint_effort_[13]);

  for(int i=0; i<10; i++){
    imu_states_[i] = imu_states_buffer_[i];
    // imu_states_[i] = 0;
    // printf("Read imu states: %f\n", imu_states_[i]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MdogHardwareInterface::write(
  const rclcpp::Time & /*时间*/, const rclcpp::Duration & /*周期*/)
{
  // TODO(任意贡献者)：下发机器人的控制指令


  // 顺序：右前（FR）髋-大腿-膝-轮 -> 左前（FL） -> 右后（RR） -> 左后（RL）
  std::array<double,16> init_offset = { 3.861508,3.218195,0.872212,0.00,    // 左前（FL） 
                                        1.366537,0.924127,2.849465,0.00,    // 右前（FR）
                                        2.975827,6.062500,0.729456,0.00,    // 左后（RL）
                                        1.336193,4.610954,3.986959,0.00,};  // 右后（RR）

  std::array<double,16> joint_offset = {0.0,0.580027778,-1.64361556,0.0,// 左前（FL）
                                        0.0,-0.580027778,1.64361556,0.0,// 右前（FR）
                                        0.0,0.580027778,-1.64361556,0.0,// 左后（RL）
                                        0.0,-0.580027778,1.64361556,0.0,// 右后（RR）
                                          };

  std::array<double,16> direction = { 1.0, 1.0, 1.0, 1.0,  // 左前（FL）
                                      1.0, 1.0, 1.0,-1.0,  // 右前（FR）
                                      1.0, 1.0, 1.0, 1.0,  // 左后（RL）
                                      1.0, 1.0, 1.0,-1.0}; // 右后（RR）
  
  // for(int i=0;i<16;i++){
  //   joint_position_command_[i]=0;
  //   joint_velocities_command_[i]=0;
  //   joint_torque_command_[i]=0;
  //   joint_kp_command_[i]=0;
  //   joint_kd_command_[i]=0;
  // }

  // joint_kd_command_[2] = 0; //0 1 2 3
  // joint_kd_command_[7] = 0; //4 5 6 7
  // joint_kd_command_[8] = 0; //8 9 10 11
  // joint_kd_command_[13] = 0; //12 13 14 15

  // 打印所有关节位置指令
  // for (int i = 0; i < 16; ++i){
  //   RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_cmd_[%d]: %f", i, joint_position_command_[i]);
  // }                                       

  // 左前（FL）
  int j = 0;
  for(int i=0;i<4;i++){
    motorCMD_msg.motor_cmd[j].mode = 1;
    motorCMD_msg.motor_cmd[j].q = (joint_position_command_[i] - joint_offset[i])*10*direction[i]+ init_offset[i];
    motorCMD_msg.motor_cmd[j].dq = joint_velocities_command_[i] * 10 *direction[i];
    motorCMD_msg.motor_cmd[j].tau = joint_torque_command_[i] *direction[i];
    motorCMD_msg.motor_cmd[j].kp = joint_kp_command_[i];
    motorCMD_msg.motor_cmd[j].kd = joint_kd_command_[i];
    motorCMD_msg.motor_cmd[j].tlimit = 400;
    j++;
  }
  motorCMD_msg.motor_cmd[3].q = 0;
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_raw_cmd_[1]: %f", joint_position_command_[1]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_cmd_[1]: %f", motorCMD_msg.motor_cmd[1].q);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_kp_cmd_[1]: %f", motorCMD_msg.motor_cmd[1].kp);
  
  // 右前（FR）
  j = 8;
  for(int i=4;i<8;i++){
    motorCMD_msg.motor_cmd[j].mode = 1;
    motorCMD_msg.motor_cmd[j].q = (joint_position_command_[i] - joint_offset[i])*10*direction[i]+ init_offset[i];
    motorCMD_msg.motor_cmd[j].dq = joint_velocities_command_[i] * 10 *direction[i];
    motorCMD_msg.motor_cmd[j].tau = joint_torque_command_[i] *direction[i];
    motorCMD_msg.motor_cmd[j].kp = joint_kp_command_[i];
    motorCMD_msg.motor_cmd[j].kd = joint_kd_command_[i];
    motorCMD_msg.motor_cmd[j].tlimit = 400;
    j++;
  }
  motorCMD_msg.motor_cmd[11].q = 0;

  // 左后（RL）
  j=12;
  for(int i=8;i<12;i++){
    motorCMD_msg.motor_cmd[j].mode = 1;
    motorCMD_msg.motor_cmd[j].q = (joint_position_command_[i] - joint_offset[i])*10*direction[i]+ init_offset[i];
    motorCMD_msg.motor_cmd[j].dq = joint_velocities_command_[i] * 10 *direction[i];
    motorCMD_msg.motor_cmd[j].tau = joint_torque_command_[i] *direction[i];
    motorCMD_msg.motor_cmd[j].kp = joint_kp_command_[i];
    motorCMD_msg.motor_cmd[j].kd = joint_kd_command_[i];
    motorCMD_msg.motor_cmd[j].tlimit = 400;
    j++;
  }
  motorCMD_msg.motor_cmd[15].q = 0;

  // 右后（RR）
  j=4;
  for(int i=12;i<16;i++){
    motorCMD_msg.motor_cmd[j].mode = 1;
    motorCMD_msg.motor_cmd[j].q = (joint_position_command_[i] - joint_offset[i])*10*direction[i]+ init_offset[i];
    motorCMD_msg.motor_cmd[j].dq = joint_velocities_command_[i] * 10 *direction[i];
    motorCMD_msg.motor_cmd[j].tau = joint_torque_command_[i] *direction[i];
    motorCMD_msg.motor_cmd[j].kp = joint_kp_command_[i];
    motorCMD_msg.motor_cmd[j].kd = joint_kd_command_[i];
    motorCMD_msg.motor_cmd[j].tlimit = 400;
    j++;
  }
  motorCMD_msg.motor_cmd[7].q = 0;

  for(int i=0;i<16;i++){
    motorCMD_msg.motor_cmd[i].tlimit = 1000;
  }

  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[1].q: %f", joint_position_command_[1] * 10 + init_offset[1] + joint_offset[1] * 10);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[1].dq: %f", joint_velocities_command_[1]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[1].tqu: %f", joint_torque_command_[1]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[1].kp: %f", joint_kp_command_[1]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[1].kd: %f", joint_kd_command_[1]);

  // 打印所有关节的 kd
  // for (int i = 0; i < 16; ++i){
  //   RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[%d].kd: %f", i, motorCMD_msg.motor_cmd[i].kd);
  // }
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[%d].q: %f", 1, joint_position_command_[1]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[%d].q: %f", 5, joint_position_command_[5]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[%d].q: %f",  9, joint_position_command_[9]);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[%d].q: %f", 16, joint_position_command_[13]);

  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[6].q: %f", motorCMD_msg.motor_cmd[6].q);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[6].dq: %f", motorCMD_msg.motor_cmd[6].dq);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[14].q: %f", motorCMD_msg.motor_cmd[14].q);
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "motor_cmd[14].dq: %f", motorCMD_msg.motor_cmd[14].dq);

  cmd_publisher_->publish(motorCMD_msg);
  return hardware_interface::return_type::OK;
}

void MdogHardwareInterface::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
  imu_states_buffer_[0] = msg->orientation.w;
  imu_states_buffer_[1] = msg->orientation.x;
  imu_states_buffer_[2] = msg->orientation.y;
  imu_states_buffer_[3] = msg->orientation.z;
  imu_states_buffer_[4] = msg->angular_velocity.x;
  imu_states_buffer_[5] = msg->angular_velocity.y;
  imu_states_buffer_[6] = msg->angular_velocity.z;
  imu_states_buffer_[7] = msg->linear_acceleration.x;
  imu_states_buffer_[8] = msg->linear_acceleration.y;
  imu_states_buffer_[9] = msg->linear_acceleration.z;
  // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "获取 IMU 数据...");
}

void MdogHardwareInterface::low_state_callback(const pd_interfaces::msg::LowState::SharedPtr msg){
    // RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "get data...");
    for (int i=0; i<16; i++){
      joint_position_temp_[i] = msg->motor_state[i].q;
      joint_velocities_temp_[i] = msg->motor_state[i].dq;
      joint_effort_temp_[i] = msg->motor_state[i].tau;
      joint_kp_temp_[i] = msg->motor_state[i].kp;
      joint_kd_temp_[i] = msg->motor_state[i].kd;
      joint_error_temp_[i] = msg->motor_state[i].error;
    }

    if (init_flag == 0){
        wheel_init_pos_[0] = joint_position_temp_[3];// 左前（FL）
        wheel_init_pos_[1] = joint_position_temp_[11];// 右前（FR）
        wheel_init_pos_[2] = joint_position_temp_[15];// 左后（RL）
        wheel_init_pos_[3] = joint_position_temp_[7];// 右后（RR）
        init_flag = 1;
    }
    // for (int i=0; i<16; i++){
    //   RCLCPP_INFO(rclcpp::get_logger("MdogHardwareInterface"), "joint_position_[%d]: %f", i, joint_position_temp_[i]);
    // }
}

}  // namespace mdog_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mdog_hardware_interface::MdogHardwareInterface, hardware_interface::SystemInterface)
