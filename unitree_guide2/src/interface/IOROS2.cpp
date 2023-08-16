/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS2.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
    RCLCPP_INFO(rclcpp::get_logger("ros_interface"), "ROS2 interface shutting down!");
    rclcpp::shutdown();
}

IOROS2::IOROS2(std::shared_ptr<rclcpp::Node> node) : IOInterface(),
        Node("IOROS2_node"), 
         node_(node) {
        RCLCPP_INFO(node_->get_logger(), "The control interface for ROS2 Gazebo simulation");

        _robot_name = node_->declare_parameter<std::string>("robot_name", "");
        RCLCPP_INFO(node_->get_logger(), "robot_name: %s", _robot_name.c_str());

        // start subscriber
        initRecv();

        // initialize publisher
        initSend(); 

        signal(SIGINT, RosShutDown);

        cmdPanel = new KeyBoard();
    }


IOROS2::~IOROS2(){
    delete cmdPanel;
    rclcpp::shutdown();
}

void IOROS2::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}


void IOROS2::sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < 12; ++i){
        _lowCmd.motor_cmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motor_cmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motor_cmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motor_cmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motor_cmd[i].kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motor_cmd[i].kp = lowCmd->motorCmd[i].Kp;
    }
    for(int m(0); m < 12; ++m){
        _servo_pub[m]->publish(_lowCmd.motor_cmd[m]);
    }
    rclcpp::spin_some(node_);
}

void IOROS2::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motor_state[i].q;
        state->motorState[i].dq = _lowState.motor_state[i].dq;
        state->motorState[i].ddq = _lowState.motor_state[i].ddq;
        state->motorState[i].tauEst = _lowState.motor_state[i].tau_est;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS2::initSend(){
    _servo_pub[0] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 10);
    _servo_pub[1] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 10);
    _servo_pub[2] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 10);
    _servo_pub[3] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 10);
    _servo_pub[4] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 10);
    _servo_pub[5] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 10);
    _servo_pub[6] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 10);
    _servo_pub[7] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 10);
    _servo_pub[8] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 10);
    _servo_pub[9] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 10);
    _servo_pub[10] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 10);
    _servo_pub[11] = node_->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 10);
}

void IOROS2::initRecv(){
    _imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>("/trunk_imu", 10, std::bind(&IOROS2::imuCallback, this, std::placeholders::_1));
    _servo_sub[0] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FR_hip_controller/state", 10, std::bind(&IOROS2::FRhipCallback, this, std::placeholders::_1));
    _servo_sub[1] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 10, std::bind(&IOROS2::FRthighCallback, this, std::placeholders::_1));
    _servo_sub[2] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FR_calf_controller/state", 10, std::bind(&IOROS2::FRcalfCallback, this, std::placeholders::_1));
    _servo_sub[3] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FL_hip_controller/state", 10, std::bind(&IOROS2::FLhipCallback, this, std::placeholders::_1));
    _servo_sub[4] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 10, std::bind(&IOROS2::FLthighCallback, this, std::placeholders::_1));
    _servo_sub[5] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FL_calf_controller/state", 10, std::bind(&IOROS2::FLcalfCallback, this, std::placeholders::_1));
    _servo_sub[6] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RR_hip_controller/state", 10, std::bind(&IOROS2::RRhipCallback, this, std::placeholders::_1));
    _servo_sub[7] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 10, std::bind(&IOROS2::RRthighCallback, this, std::placeholders::_1));
    _servo_sub[8] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RR_calf_controller/state", 10, std::bind(&IOROS2::RRcalfCallback, this, std::placeholders::_1));
    _servo_sub[9] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RL_hip_controller/state", 10, std::bind(&IOROS2::RLhipCallback, this, std::placeholders::_1));
    _servo_sub[10] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 10, std::bind(&IOROS2::RLthighCallback, this, std::placeholders::_1));
    _servo_sub[11] = node_->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RL_calf_controller/state", 10, std::bind(&IOROS2::RLcalfCallback, this, std::placeholders::_1));
}



void IOROS2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{ 
    _lowState.imu.quaternion[0] = msg->orientation.w;
    _lowState.imu.quaternion[1] = msg->orientation.x;
    _lowState.imu.quaternion[2] = msg->orientation.y;
    _lowState.imu.quaternion[3] = msg->orientation.z;

    _lowState.imu.gyroscope[0] = msg->angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg->angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg->angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
}

void IOROS2::FRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[0].mode = msg->mode;
    _lowState.motor_state[0].q = msg->q;
    _lowState.motor_state[0].dq = msg->dq;
    _lowState.motor_state[0].tau_est = msg->tau_est;
}

void IOROS2::FRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[1].mode = msg->mode;
    _lowState.motor_state[1].q = msg->q;
    _lowState.motor_state[1].dq = msg->dq;
    _lowState.motor_state[1].tau_est = msg->tau_est;
}

void IOROS2::FRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[2].mode = msg->mode;
    _lowState.motor_state[2].q = msg->q;
    _lowState.motor_state[2].dq = msg->dq;
    _lowState.motor_state[2].tau_est = msg->tau_est;
}

void IOROS2::FLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[3].mode = msg->mode;
    _lowState.motor_state[3].q = msg->q;
    _lowState.motor_state[3].dq = msg->dq;
    _lowState.motor_state[3].tau_est = msg->tau_est;
}

void IOROS2::FLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[4].mode = msg->mode;
    _lowState.motor_state[4].q = msg->q;
    _lowState.motor_state[4].dq = msg->dq;
    _lowState.motor_state[4].tau_est = msg->tau_est;
}

void IOROS2::FLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[5].mode = msg->mode;
    _lowState.motor_state[5].q = msg->q;
    _lowState.motor_state[5].dq = msg->dq;
    _lowState.motor_state[5].tau_est = msg->tau_est;
}

void IOROS2::RRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[6].mode = msg->mode;
    _lowState.motor_state[6].q = msg->q;
    _lowState.motor_state[6].dq = msg->dq;
    _lowState.motor_state[6].tau_est = msg->tau_est;
}

void IOROS2::RRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[7].mode = msg->mode;
    _lowState.motor_state[7].q = msg->q;
    _lowState.motor_state[7].dq = msg->dq;
    _lowState.motor_state[7].tau_est = msg->tau_est;
}

void IOROS2::RRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[8].mode = msg->mode;
    _lowState.motor_state[8].q = msg->q;
    _lowState.motor_state[8].dq = msg->dq;
    _lowState.motor_state[8].tau_est = msg->tau_est;
}

void IOROS2::RLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[9].mode = msg->mode;
    _lowState.motor_state[9].q = msg->q;
    _lowState.motor_state[9].dq = msg->dq;
    _lowState.motor_state[9].tau_est = msg->tau_est;
}

void IOROS2::RLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[10].mode = msg->mode;
    _lowState.motor_state[10].q = msg->q;
    _lowState.motor_state[10].dq = msg->dq;
    _lowState.motor_state[10].tau_est = msg->tau_est;
}

void IOROS2::RLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[11].mode = msg->mode;
    _lowState.motor_state[11].q = msg->q;
    _lowState.motor_state[11].dq = msg->dq;
    _lowState.motor_state[11].tau_est = msg->tau_est;
}

#endif  // COMPILE_WITH_ROS