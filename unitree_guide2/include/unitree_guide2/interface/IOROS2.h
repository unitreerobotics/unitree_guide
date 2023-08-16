/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifdef COMPILE_WITH_ROS

#ifndef IOROS2_H
#define IOROS2_H

#include "rclcpp/rclcpp.hpp"
#include "interface/IOInterface.h"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <string>

class IOROS2 : public IOInterface, public rclcpp::Node
{
public:
    IOROS2(std::shared_ptr<rclcpp::Node> node);
    ~IOROS2();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    void initRecv();
    void initSend();

private:
    std::shared_ptr<rclcpp::Node> node_;
    void sendCmd(const LowlevelCmd *cmd);
    void recvState(LowlevelState *state);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::MotorState>::SharedPtr _servo_sub[12];
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::MotorCmd>::SharedPtr _servo_pub[12];

    ros2_unitree_legged_msgs::msg::LowCmd _lowCmd;
    ros2_unitree_legged_msgs::msg::LowState _lowState;
    std::string _robot_name;

    // Callback functions for ROS
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void FRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void FRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void FRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

    void FLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void FLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void FLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

    void RRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void RRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void RRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

    void RLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void RLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
    void RLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS
