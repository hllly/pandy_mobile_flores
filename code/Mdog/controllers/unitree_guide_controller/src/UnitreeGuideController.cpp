//
// 由 pj 于 24-9-6 创建。
//

#include "unitree_guide_controller/UnitreeGuideController.h"
#include "unitree_guide_controller/FSM/StatePassive.h"
#include "unitree_guide_controller/robot/QuadrupedRobot.h"

namespace unitree_guide_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration UnitreeGuideController::command_interface_configuration() const {
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

    controller_interface::InterfaceConfiguration UnitreeGuideController::state_interface_configuration() const {
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

        return conf;
    }

    controller_interface::return_type UnitreeGuideController::
    update(const rclcpp::Time & /*时间*/, const rclcpp::Duration & /*周期*/) {
        if (ctrl_comp_.robot_model_ == nullptr) {
            return controller_interface::return_type::OK;
        }

        ctrl_comp_.robot_model_->update();
        ctrl_comp_.wave_generator_->update();
        ctrl_comp_.estimator_->update();

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

    controller_interface::CallbackReturn UnitreeGuideController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            command_interface_types_ =
                    auto_declare<std::vector<std::string> >("command_interfaces", command_interface_types_);
            state_interface_types_ =
                    auto_declare<std::vector<std::string> >("state_interfaces", state_interface_types_);

            // IMU 传感器
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string> >("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                    auto_declare<std::vector<std::string> >("feet_names", feet_names_);

            // 姿态参数
            down_pos_ = auto_declare<std::vector<double> >("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double> >("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

            get_node()->get_parameter("update_rate", ctrl_comp_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_comp_.frequency_);

            ctrl_comp_.estimator_ = std::make_shared<Estimator>(ctrl_comp_);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_configure(
        const rclcpp_lifecycle::State & /*上一个状态*/) {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
                // 处理消息
                ctrl_comp_.control_inputs_.command = msg->command;
                ctrl_comp_.control_inputs_.lx = msg->lx;
                ctrl_comp_.control_inputs_.ly = msg->ly;
                ctrl_comp_.control_inputs_.rx = msg->rx;
                ctrl_comp_.control_inputs_.ry = msg->ry;
            });

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                ctrl_comp_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_comp_, msg->data, feet_names_, base_name_);
                ctrl_comp_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_comp_.robot_model_);
            });

        ctrl_comp_.wave_generator_ = std::make_shared<WaveGenerator>(0.45, 0.5, Vec4(0, 0.5, 0.5, 0));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
UnitreeGuideController::on_activate(const rclcpp_lifecycle::State & /*上一个状态*/) {
        // 防止重启后残留数据，先清空缓存向量
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
            } else {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // 创建有限状态机状态列表
        state_list_.passive = std::make_shared<StatePassive>(ctrl_comp_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_comp_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_comp_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_comp_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_comp_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_comp_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_comp_);

        // 初始化有限状态机
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_deactivate(
        const rclcpp_lifecycle::State & /*上一个状态*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
UnitreeGuideController::on_cleanup(const rclcpp_lifecycle::State & /*上一个状态*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
UnitreeGuideController::on_error(const rclcpp_lifecycle::State & /*上一个状态*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
UnitreeGuideController::on_shutdown(const rclcpp_lifecycle::State & /*上一个状态*/) {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> UnitreeGuideController::getNextState(const FSMStateName stateName) const {
        switch (stateName) {
            case FSMStateName::INVALID:
                return state_list_.invalid;
            case FSMStateName::PASSIVE:
                return state_list_.passive;
            case FSMStateName::FIXEDDOWN:
                return state_list_.fixedDown;
            case FSMStateName::FIXEDSTAND:
                return state_list_.fixedStand;
            case FSMStateName::FREESTAND:
                return state_list_.freeStand;
            case FSMStateName::TROTTING:
                return state_list_.trotting;
            case FSMStateName::SWINGTEST:
                return state_list_.swingTest;
            case FSMStateName::BALANCETEST:
                return state_list_.balanceTest;
            default:
                return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_guide_controller::UnitreeGuideController, controller_interface::ControllerInterface);
