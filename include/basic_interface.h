/**
 * @file basic_interface.h
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-12-03
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once
/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <thread>
#include <vector>

/* external project header files */
//#include "geometry_msgs/PoseStamped.h"
#include "MotorState.h"
#include <Eigen/Dense>
#include <ros/ros.h>

/* internal project header files */
#include "state_estimator.h"
// #include "common/models/leg_kinematic.h"
#include "iir_filter.h"
#include "timer.h"
#include "robot_param.h"
#include "robot_state.h"

namespace basic_interface
{
using namespace robot_state;
struct MotorSend
{
    double kp{}, kd{}, pos{}, vel{}, torque{};
};
struct MotorReceive
{
    [[deprecated]] MotorReceive &operator=(const unitree_legged_msgs::MotorState &motor_state)
    {
        pos = motor_state.q;
        vel = motor_state.dq;
        acc = motor_state.ddq;
        torque = motor_state.tauEst;
        temp = motor_state.temperature;
        return *this;
    }
    double pos{}, vel{}, acc{}, torque{}, temp{};
};
} // namespace basic_interface

class BasicInterface
{
  public:
    BasicInterface(ros::NodeHandle &r_nh, RobotState &r_robot_state);
    virtual ~BasicInterface() = default;
    void interfaceThread(void);
    void resetCommand(void);

  protected:
    ros::NodeHandle &nh_;
    /**
     * @brief multi-threading is not safe, so set the buffer variables
     */
    basic_interface::ImuSensor imu_{};
    basic_interface::Odometry odom_{};
    //basic_interface::FootForceSensor foot_force_sensor_;
    std::vector<std::vector<basic_interface::MotorReceive>> motor_receive_;
    std::vector<std::vector<basic_interface::MotorSend>> motor_send_;

  private:
    virtual void read(void) = 0;
    virtual void write(void) = 0;
    void threadConfig(void);
    void updateCommand(void);
    void limitCommand(void);
    void updateFeedback(void);
    void updateRobotState(void);
    //void vrpnCallback(const geometry_msgs::PoseStamped &vrpn_pose);

    RobotState &robot_state_;
    //Kinematic kinematic_;
    Timer timer_;
    StateEstimator state_estimator_;
    std::vector<std::vector<IirFilter>> joint_torque_cmd_filter_mat_{}, joint_pos_fdb_filter_mat_{},
        joint_vel_fdb_filter_mat_{};
    ros::Subscriber sub_vrpn_msg_;
    //geometry_msgs::PoseStamped vrpn_msg_;
    double vrpn_z_offset_{};
    std::vector<IirFilter> vrpn_body_linear_vel_filter_vec_{};

  protected:
    /**
    * @brief robot_param is multi-thread safe, other variables are atomic, so use reference
    */
    basic_interface::Joystick &joystick_;
    RobotParam &robot_param_;
    std::atomic<CommBoardState> &comm_board_state_;
    std::atomic<bool> &feedback_ready_flag_;
    std::thread fdb_monitor_thread_;
    // LowPassFilter lpf_[3]{};
};
