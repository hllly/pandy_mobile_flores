//
// 由 pj 于 24-10-4 创建。
//

#include "RlQuadrupedController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rl_quadruped_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration LeggedGymController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: command_interface_types_) {
                if (!command_prefix_.empty()) {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                } else {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LeggedGymController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto &interface_type: imu_interface_types_) {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        for (const auto &interface_type: foot_force_interface_types_) {
            conf.names.push_back(foot_force_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type LeggedGymController::
    update(const rclcpp::Time & /*时间*/, const rclcpp::Duration & /*周期*/) {
        if (ctrl_comp_.enable_estimator_) {
            if (ctrl_comp_.robot_model_ == nullptr) {
                return controller_interface::return_type::OK;
            }

            ctrl_comp_.robot_model_->update();
            ctrl_comp_.estimator_->update();
        }

        if (mode_ == FSMMode::NORMAL) {
            current_state_->run();
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name) {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        } else if (mode_ == FSMMode::CHANGE) {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn LeggedGymController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            feet_names_ = auto_declare<std::vector<std::string> >("feet_names", feet_names_);
            command_interface_types_ =
                    auto_declare<std::vector<std::string> >("command_interfaces", command_interface_types_);
            state_interface_types_ =
                    auto_declare<std::vector<std::string> >("state_interfaces", state_interface_types_);

            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);

            // IMU 传感器
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string> >("imu_interfaces", state_interface_types_);

            // 足端力传感器
            foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
            foot_force_interface_types_ =
                    auto_declare<std::vector<std::string> >("foot_force_interfaces", foot_force_interface_types_);

            // 强化学习配置目录
            robot_pkg_ = auto_declare<std::string>("robot_pkg", robot_pkg_);
            model_folder_ = auto_declare<std::string>("model_folder", model_folder_);

            // 姿态参数
            down_pos_ = auto_declare<std::vector<double> >("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double> >("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

            get_node()->get_parameter("update_rate", ctrl_comp_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Update Rate: %d Hz", ctrl_comp_.frequency_);

            if (foot_force_interface_types_.size() == 4) {
                ctrl_comp_.enable_estimator_ = true;
                ctrl_comp_.estimator_ = std::make_shared<Estimator>(ctrl_comp_);
            }
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_configure(
        const rclcpp_lifecycle::State & /*上一个状态*/) {
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (ctrl_comp_.enable_estimator_) {
                    ctrl_comp_.robot_model_ = std::make_shared<QuadrupedRobot>(
                        ctrl_comp_, msg->data, feet_names_, base_name_);
                }
            });


        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
                // 处理消息
                ctrl_comp_.control_inputs_.command = msg->command;
                if (ctrl_comp_.control_inputs_.command ==10){
                    use_high_cmd_ = !use_high_cmd_;
                }
                ctrl_comp_.control_inputs_.lx = msg->lx;
                ctrl_comp_.control_inputs_.ly = msg->ly;
                ctrl_comp_.control_inputs_.rx = msg->rx;
                ctrl_comp_.control_inputs_.ry = msg->ry;
            });

        high_input_subscription_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
            "/HighCmd", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // 处理消息
                if (use_high_cmd_){
                    ctrl_comp_.control_inputs_.lx = msg->twist.twist.linear.y;
                    ctrl_comp_.control_inputs_.ly = msg->twist.twist.linear.x;
                    ctrl_comp_.control_inputs_.rx = msg->twist.twist.angular.z;
                    ctrl_comp_.control_inputs_.ry = 0;
                }
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_activate(
        const rclcpp_lifecycle::State & /*上一个状态*/) {
        // 避免重启时残留数据，先清空缓存向量
        ctrl_comp_.clear();

        // 分配控制指令接口
        for (auto &interface: command_interfaces_) {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos) {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            } else {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // 分配状态接口
        for (auto &interface: state_interfaces_) {
            if (interface.get_prefix_name() == imu_name_) {
                ctrl_comp_.imu_state_interface_.emplace_back(interface);
            } else if (interface.get_prefix_name() == foot_force_name_) {
                ctrl_comp_.foot_force_state_interface_.emplace_back(interface);
            } else {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // 创建有限状态机状态列表
        state_list_.passive = std::make_shared<StatePassive>(ctrl_comp_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_comp_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_comp_, stand_pos_, stand_kp_, stand_kd_);


        RCLCPP_INFO(get_node()->get_logger(), "Using robot model from %s", robot_pkg_.c_str());
        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        std::string model_path = package_share_directory + "/config/" + model_folder_;
        state_list_.rl = std::make_shared<StateRL>(ctrl_comp_, model_path, stand_pos_);

        // 初始化有限状态机
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_deactivate(
        const rclcpp_lifecycle::State & /*上一个状态*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        return ControllerInterface::on_cleanup(previous_state);
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        return ControllerInterface::on_shutdown(previous_state);
    }

    controller_interface::CallbackReturn LeggedGymController::on_error(const rclcpp_lifecycle::State &previous_state) {
        return ControllerInterface::on_error(previous_state);
    }

    std::shared_ptr<FSMState> LeggedGymController::getNextState(const FSMStateName stateName) const {
        switch (stateName) {
            case FSMStateName::INVALID:
                return state_list_.invalid;
            case FSMStateName::PASSIVE:
                return state_list_.passive;
            case FSMStateName::FIXEDDOWN:
                return state_list_.fixedDown;
            case FSMStateName::FIXEDSTAND:
                return state_list_.fixedStand;
            case FSMStateName::RL:
                return state_list_.rl;
            default:
                return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller::LeggedGymController, controller_interface::ControllerInterface);
