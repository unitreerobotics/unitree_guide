#include "unitree_joint_controller/joint_controller.h"
#include "rclcpp/parameter_client.hpp"



namespace unitree_joint_controller {

    UnitreeJointController::UnitreeJointController(){
        memset(&lastCmd_, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorCmd));
        memset(&lastState_, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorState));
        memset(&servoCmd_, 0, sizeof(servoCmd_));
    }

    UnitreeJointController::~UnitreeJointController(){
        sub_ft_ = nullptr; // This will also drop the reference count and clean up if it's the last reference.
        sub_cmd_ = nullptr; // Same for this subscription.
    }


    controller_interface::return_type UnitreeJointController::update(const rclcpp::Time & time, const rclcpp::Duration & period)  {
        // Initialize the node 
        
        double currentPos, currentVel, calcTorque;
        lastCmd_ = *(command_.readFromRT());
        try {
            if(lastCmd_.mode == PMSM) {
                servoCmd_.pos = lastCmd_.q;
                positionLimits(servoCmd_.pos);
                servoCmd_.posStiffness = lastCmd_.kp;
                if(fabs(lastCmd_.q - PosStopF) < 0.00001){
                    servoCmd_.posStiffness = 0;
                }
                servoCmd_.vel = lastCmd_.dq;
                velocityLimits(servoCmd_.vel);
                servoCmd_.velStiffness = lastCmd_.kd;
                if(fabs(lastCmd_.dq - VelStopF) < 0.00001){
                    servoCmd_.velStiffness = 0;
                }
                servoCmd_.torque = lastCmd_.tau;
                effortLimits(servoCmd_.torque);
            }
            if(lastCmd_.mode == BRAKE) {
                servoCmd_.posStiffness = 0;
                servoCmd_.vel = 0;
                servoCmd_.velStiffness = 20;
                servoCmd_.torque = 0;
                effortLimits(servoCmd_.torque);
            }

        #ifdef rqtTune
                double i, i_max, i_min;
                getGains(servoCmd.posStiffness,i,servoCmd.velStiffness,i_max,i_min);
        #endif
            for (auto& state_interface : state_interfaces_) {
                if (state_interface.get_interface_name() == "position") {
                    currentPos = state_interface.get_value(); 
                    break;
                }
            }
            currentVel = computeVel(currentPos, (double)lastState_.q, (double)lastState_.dq, period.seconds());
            calcTorque = computeTorque(currentPos, currentVel, servoCmd_);      
            effortLimits(calcTorque);
            for (auto& command_interface : joint_cmd_interfaces_) {
                if (command_interface.get_interface_name() == "position") {
                    command_interface.set_value(calcTorque);
                    break;
                }
            }
            lastState_.q = currentPos;
            lastState_.dq = currentVel;
            for (auto& state_interface : state_interfaces_) {
                if (state_interface.get_interface_name() == "effort") {
                    lastState_.tau_est = state_interface.get_value(); 
                    break;
                }
            }

            // ros is ttypically not needed to lock and unlock since it is thread safe
            ros2_unitree_legged_msgs::msg::MotorState motor_state_msg;
            motor_state_msg.q = lastState_.q;
            motor_state_msg.dq = lastState_.dq;
            motor_state_msg.tau_est = lastState_.tau_est;
            if (controller_state_publisher_) {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;33mPublishing the motor state\033[0m");
                controller_state_publisher_->publish(motor_state_msg);
            }
        } catch (const std::exception & e){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception thrown during init stage with message: %s \n", e.what()); 
        }
        

        return controller_interface::return_type::OK;
    }

    //Second
    controller_interface::InterfaceConfiguration UnitreeJointController::command_interface_configuration() const
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> command_interface_configuration()\033[0m");
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        // conf.names.reserve(dof_ * params_.command_interfaces.size());
        for (const auto & interface_type : params_.command_interfaces)
        {
            conf.names.push_back(joint_name_ + "/" + interface_type);
        }
        return conf;
    }

    controller_interface::InterfaceConfiguration UnitreeJointController::state_interface_configuration() const
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> state_interface_configuration()\033[0m");
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        // conf.names.reserve(dof_ * params_.state_interfaces.size());
        for (const auto & interface_type : params_.state_interfaces)
        {
            conf.names.push_back(joint_name_ + "/" + interface_type);
        }
        return conf;
    }

    //First 
    controller_interface::CallbackReturn UnitreeJointController::on_init()
    {
        isHip_ = false;
        isThigh_ = false;
        isCalf_ = false;
        sensor_torque_ = 0;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> on_init()\033[0m");
        try
        {
        #ifdef rqtTune
                    // Load PID Controller using gains set on parameter server
                    if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
                        return false;
        #endif
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
            if (params_.joints.size() > 1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "\033[1;31m Curretnly each controller should have one joint\033[0m");

            } 

            std::string robot_description;
            if (waitForParameter("robot_description", robot_description, 5 /* timeout in seconds */)) {
                RCLCPP_INFO(get_node()->get_logger(), "Robot description retrieved successfully.");
            } else {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to get 'robot_description' parameter after waiting.");
            }

            // Parse the robot description to get the URDF model
            urdf::Model urdf_model;
            if (!urdf_model.initString(robot_description)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF contained in 'robot_description' parameter");
                return controller_interface::CallbackReturn::ERROR;
            }

            joint_name_ = params_.joints[0];
            joint_urdf_ = urdf_model.getJoint(joint_name_);
            if(joint_name_ == "FR_hip_joint" || joint_name_ == "FL_hip_joint" || joint_name_ == "RR_hip_joint" || joint_name_ == "RL_hip_joint"){
                isHip_ = true;
            }
            if(joint_name_ == "FR_calf_joint" || joint_name_ == "FL_calf_joint" || joint_name_ == "RR_calf_joint" || joint_name_ == "RL_calf_joint"){
                isCalf_ = true;
            }    
            setConfig();
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        // TODO(christophfroehlich): remove deprecation warning
        if (params_.allow_nonzero_velocity_at_trajectory_end)
        {
            RCLCPP_WARN(
            get_node()->get_logger(),
            "[Deprecated]: \"allow_nonzero_velocity_at_trajectory_end\" is set to "
            "true. The default behavior will change to false.");
        }

        return CallbackReturn::SUCCESS;
    }

    bool UnitreeJointController::waitForParameter( const std::string& param_name, std::string& param_value, int timeout_seconds) {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "robot_state_publisher");

        // Wait for the service to be available.
        while (!parameters_client->wait_for_service(std::chrono::seconds(timeout_seconds))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return 1;
            }
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for the robot_state_publisher service...");
        }

        // Get the robot_description parameter.
        auto parameters = parameters_client->get_parameters({"robot_description"});

        // Check if the parameter was retrieved successfully.
        if (!parameters.empty()) {
            auto& param = parameters.front();
            param_value = param.value_to_string();
            RCLCPP_INFO(get_node()->get_logger(), "Got robot_description");
            return true;
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter not found.");
            return false;
        }
    }

    void UnitreeJointController::setConfig() {
        std::string ft_topic = joint_name_ + "/joint_wrench";
        std::string cmd_topic = joint_name_ + "/command";
        sub_ft_ =get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
            ft_topic, 1, std::bind(&UnitreeJointController::setTorqueCB, this, std::placeholders::_1));
        sub_cmd_ =get_node()->create_subscription<ros2_unitree_legged_msgs::msg::MotorCmd>(
           cmd_topic, 20, std::bind(&UnitreeJointController::setCommandCB, this, std::placeholders::_1));
        controller_state_publisher_ = get_node()->create_publisher<ros2_unitree_legged_msgs::msg::MotorState>(joint_name_ + "/state", 10);
    }




    controller_interface::CallbackReturn UnitreeJointController::on_configure(const rclcpp_lifecycle::State& previous_state)  {
        // Set up the controller based on parameters, etc.
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;30m----> on_confirgure()\033[0m");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_activate(const rclcpp_lifecycle::State& previous_state)  {
        // lastCmd.Kp = 0;
        // lastCmd.Kd = 0;
        // auto result = hardware_interface::ResourceManager::get_state_interface_handle(joint_name_);
        double init_pos = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;31m----> on_activate()\033[0m");
        if (ControllerInterface::on_activate(previous_state) != controller_interface::CallbackReturn::SUCCESS) {
            return controller_interface::CallbackReturn::ERROR;
        }
            for (auto& state_interface : state_interfaces_) {
                if (state_interface.get_interface_name() == "position") {
                    init_pos = state_interface.get_value();
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;33m----> get the position for %s is  %lf\033[0m", joint_name_.c_str(), init_pos);
                    // Now you have the current position of the joint
                    break;
            }
        }

        lastCmd_.q = init_pos;
        lastState_.q = init_pos;
        lastCmd_.dq = 0;
        lastState_.dq = 0;
        lastCmd_.tau = 0;
        lastState_.tau_est = 0;
        command_.initRT(lastCmd_);
        controller_state_publisher_->on_activate();
        pid_controller_.reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void UnitreeJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }
    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::positionLimits(double &position)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf_->limits->lower, joint_urdf_->limits->upper);
    }

    void UnitreeJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf_->limits->velocity, joint_urdf_->limits->velocity);
    }

    void UnitreeJointController::effortLimits(double &effort)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf_->limits->effort, joint_urdf_->limits->effort);
    }


    controller_interface::CallbackReturn UnitreeJointController::on_deactivate(const rclcpp_lifecycle::State& previous_state)  {
        // Deactivate publishers, etc.
        controller_state_publisher_->on_deactivate();
        sub_ft_.reset();
        sub_cmd_.reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_cleanup(const rclcpp_lifecycle::State& previous_state)  {
        // Clean up any resources
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_error(const rclcpp_lifecycle::State& previous_state)  {
        // Handle errors
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_shutdown(const rclcpp_lifecycle::State& previous_state)  {
        // Handle shutdown
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // And so on with other methods that need porting

    // ...
    void UnitreeJointController::setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        if(isHip_) sensor_torque_ = msg->wrench.torque.x;
        else sensor_torque_ = msg->wrench.torque.y;
    }

    void UnitreeJointController::setCommandCB(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;30mReceived command\033[0m");
        lastCmd_.mode = msg->mode;
        lastCmd_.q = msg->q;
        lastCmd_.kp = msg->kp;
        lastCmd_.dq = msg->dq;
        lastCmd_.kd = msg->kd;
        lastCmd_.tau = msg->tau;
        command_.writeFromNonRT(lastCmd_);
    }

    // Helper functions for controlling the joint
    // ...

}

#include "pluginlib/class_list_macros.hpp"
// Export controller as a plugin
PLUGINLIB_EXPORT_CLASS(
  unitree_joint_controller::UnitreeJointController, controller_interface::ControllerInterface)
