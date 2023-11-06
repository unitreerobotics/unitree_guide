/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#ifndef STATE_MOVE_BASE_H
#define STATE_MOVE_BASE_H

#include "FSM/State_Trotting.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"


class State_move_base : public State_Trotting, public rclcpp::Node {
public:
    State_move_base(CtrlComponents *ctrlComp);
    ~State_move_base() override {}
    FSMStateName checkChange() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
    void getUserCmd();
    void initRecv();
    // void run();
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdSub;
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr _cmdhigh;

    double _vx, _vy;
    double _wz;
};

#endif  // STATE_MOVE_BASE_H

#endif  // COMPILE_WITH_MOVE_BASE