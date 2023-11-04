#ifndef _UNITREE_ROS2_JOINT_CONTROLLER_H_
#define _UNITREE_ROS2_JOINT_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <control_toolbox/pid.hpp>
#include <memory>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include <std_msgs/msg/float64.hpp>
#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "unitree_joint_control_tool.h"
#include <urdf/model.h>
#include "unitree_joint_controller_parameters.hpp"
#include <memory>

#define PMSM      (0x0A)
#define BRAKE     (0x00)
#define PosStopF  (2.146E+9f)
#define VelStopF  (16000.0f)

namespace rclcpp_action
{
template <typename ActionT>
class ServerGoalHandle;
}  // namespace rclcpp_action
namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace unitree_joint_controller
{
    class UnitreeJointController: public controller_interface::ControllerInterface
    {
    private:
        std::vector<hardware_interface::LoanedCommandInterface> joint_cmd_interfaces_;
        std::vector<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_;
        rclcpp::Subscription<ros2_unitree_legged_msgs::msg::MotorCmd>::SharedPtr sub_cmd_;
        control_toolbox::Pid pid_controller_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_unitree_legged_msgs::msg::MotorState>> controller_state_publisher_;

    public:
        std::string name_space_;
        std::string joint_name_;
        float sensor_torque_;
        bool isHip_, isThigh_, isCalf_, rqtTune_;
        urdf::JointConstSharedPtr joint_urdf_;
        realtime_tools::RealtimeBuffer<ros2_unitree_legged_msgs::msg::MotorCmd> command_;
        ros2_unitree_legged_msgs::msg::MotorCmd lastCmd_;
        ros2_unitree_legged_msgs::msg::MotorState lastState_;
        ServoCmd servoCmd_;

        UnitreeJointController();
        ~UnitreeJointController();

        CallbackReturn on_init() override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
        
        void setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void setCommandCB(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg);
        void setConfig();

        // ... other public methods as needed ...

    private:

            // Preallocate variables used in the realtime update() function
        trajectory_msgs::msg::JointTrajectoryPoint state_current_;
        trajectory_msgs::msg::JointTrajectoryPoint command_current_;
        trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
        trajectory_msgs::msg::JointTrajectoryPoint state_error_;

        // Degrees of freedom
        size_t dof_;

        // Storing command joint names for interfaces
        std::vector<std::string> command_joint_names_;

        // Parameters from ROS for joint_trajectory_controller
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;


        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);
        void setGains(const double &, const double &, const double &, const double &, const double &, const bool &);
        void getGains(double &, double &, double &, double &, double &, bool &);
        void getGains(double &, double &, double &, double &, double &);
        bool waitForParameter(const std::string& , std::string& , int ); 

        // ... other private methods and member variables as needed ...

    };
}

#endif // _UNITREE_ROS2_JOINT_CONTROLLER_H_
