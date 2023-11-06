/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"
#include <thread>
#include <memory>



State_move_base::State_move_base(CtrlComponents *ctrlComp)
: State_Trotting(ctrlComp), Node("state_move_base"){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    initRecv();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
}

void State_move_base::highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Got a high cmd.\033[0m");

    _vx = msg->velocity[0];
    _vy = msg->velocity[1];
    _wz = msg->yaw_speed;

}

void State_move_base::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Got a cmd vel.\033[0m");
    _vx = msg->linear.x;
    _vy = msg->linear.y;
    _wz = msg->angular.z;
    
}

void State_move_base::initRecv() {
    _cmdSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&State_move_base::twistCallback, this, std::placeholders::_1));
        // Spin up a new thread to handle callbacks for this node.
    _cmdhigh = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
        "high_cmd", 1, std::bind(&State_move_base::highCmdCallback, this, std::placeholders::_1));

    std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->get_node_base_interface());
        executor.spin();
    }).detach();
}

#endif  // COMPILE_WITH_MOVE_BASE