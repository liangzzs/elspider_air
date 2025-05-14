/**
 * @file basic_interface.cpp
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-12-24
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "basic_interface.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
//#include "common/utilities/basic_func.hpp"
#include "basic_math.hpp"
#include <ros/ros.h>

/**
 * @brief Construct a new Basic Interface:: Basic Interface object
 * @param[in] r_nh the reference to ros node handle
 * @param[in] r_robot_state the reference to robot state
 */
BasicInterface::BasicInterface(ros::NodeHandle &r_nh, RobotState &r_robot_state)
    : nh_(r_nh), robot_state_(r_robot_state), timer_(TimerSourceType::kRos),
      state_estimator_(robot_state_.param.leg_num), joystick_(robot_state_.joystick),
      robot_param_(robot_state_.param), comm_board_state_(robot_state_.comm_board_state),
      feedback_ready_flag_(robot_state_.feedback_ready_flag)
{
    // 默认 interface_type 为 "hardware"
    const std::string interface_type = "hardware";

    // 初始化 VRPN 功能（默认启用）
    robot_state_.use_vrpn = true;
    vrpn_z_offset_ = 0.0; // 默认 Z 轴偏移量为 0
    //sub_vrpn_msg_ = nh_.subscribe("/vrpn_client_node/" + robot_state_.param.robot_name + "/pose", 1,
     //                             &BasicInterface::vrpnCallback, this);

    // 初始化 VRPN 滤波器
    IirFilter vrpn_body_linear_vel_filter(2, {0.0675, 0.1349, 0.0675}, {1, -1.1430, 0.4128});
    std::vector<IirFilter> temp_filter_vec_{3, vrpn_body_linear_vel_filter};
    vrpn_body_linear_vel_filter_vec_ = temp_filter_vec_;
    
    const int kLegNum = robot_param_.leg_num;
    const int kLegDof = robot_param_.leg_dof;

    motor_receive_.resize(kLegDof);
    motor_send_.resize(kLegDof);

    for (int i = 0; i < kLegDof; ++i)
    {
        motor_receive_.at(i).resize(kLegNum);
        motor_send_.at(i).resize(kLegNum);
    }

    // initialize joint torque filter
    // second order butterworth filter, cut-off freq: 150Hz, sampling freq: 500Hz
    IirFilter joint_torque_filter(2, {0.3913, 0.7827, 0.3913}, {1, 0.3695, 0.1958});
    std::vector<IirFilter> joint_torque_filter_row_vec{6, joint_torque_filter};
    joint_torque_cmd_filter_mat_.assign(3, joint_torque_filter_row_vec);

    // temp cut-off freq
    joint_pos_fdb_filter_mat_ = joint_torque_cmd_filter_mat_;
    joint_vel_fdb_filter_mat_ = joint_torque_cmd_filter_mat_;
}

/**
 * @brief configuration for interface thread
 */
void BasicInterface::interfaceThread(void)
{
    static bool one_time_flag{};
    if (!one_time_flag)
    {
        threadConfig();
        one_time_flag = true;
    }

    // some devices don't send feedback until data is received, so write commands first
    limitCommand();
    updateCommand();
    write();
    read();
    updateFeedback();
    updateRobotState();
}

/**
 * @brief thread configuration
 */
void BasicInterface::threadConfig(void)
{
    bool enable_debug = false;
    //GET_ROS_PARAM(nh_, "/hexapod/enable_debug", enable_debug);
    nh_.getParam("/hexapod/enable_debug", enable_debug);
    if (enable_debug)
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();
}

/**
 * @brief update commands from robot_state_ to motor_send_
 */
void BasicInterface::updateCommand(void)
{
    robot_state_.cmd_mutex.lock_shared();
    for (int i = 0; i < robot_state_.param.leg_num; ++i)
    {
        for (int j = 0; j < robot_state_.param.leg_dof; ++j)
        {
            // prevent nan and inf data
            if (std::isfinite(robot_state_.joint.kd(j, i)))
                motor_send_.at(j).at(i).kd = robot_state_.joint.kd(j, i);
            if (std::isfinite(robot_state_.joint.kp(j, i)))
                motor_send_.at(j).at(i).kp = robot_state_.joint.kp(j, i);
            if (std::isfinite(robot_state_.joint.pos_cmd(j, i)))
                motor_send_.at(j).at(i).pos = robot_state_.joint.pos_cmd(j, i) -
                                              robot_state_.param.joint_calibration_pos(j, i) +
                                              robot_state_.joint.pos_zero(j, i);
            if (std::isfinite(robot_state_.joint.vel_cmd(j, i)))
                motor_send_.at(j).at(i).vel = robot_state_.joint.vel_cmd(j, i);
            if (std::isfinite(robot_state_.joint.torque_cmd(j, i)))
                motor_send_.at(j).at(i).torque = robot_state_.joint.torque_cmd(j, i);
        }
    }
    robot_state_.cmd_mutex.unlock_shared();
}

/**
 * @brief reset commands to zero
 */
void BasicInterface::resetCommand(void)
{
    robot_state_.joint.kd.setZero();
    robot_state_.joint.kp.setZero();
    robot_state_.joint.pos_cmd.setZero();
    robot_state_.joint.vel_cmd.setZero();
    robot_state_.joint.torque_cmd.setZero();
    updateCommand();
    write();
}