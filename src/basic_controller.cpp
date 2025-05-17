/**
 * @file basic_controller.cpp
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-11-30
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "basic_controller.h"

/* c system header files */

/* c++ standard library header files */
#include <memory>
#include <mutex>
#include <shared_mutex>

/* external project header files */

/* internal project header files */
//#include "common/utilities/basic_func.hpp"
#include "basic_math.hpp"

/**
 * @brief Construct a new Basic Controller:: Basic Controller object
 * @param[in] r_nh the reference to ros node handle
 * @param[in] r_robot_state the reference to robot state
 * @param[in] r_control_state the reference to control state
 */
BasicController::BasicController(ros::NodeHandle &r_nh, RobotState &r_robot_state,
    ControlState &r_control_state)
: nh_(r_nh), robot_state_(r_robot_state), control_state_(r_control_state),
timer_(TimerSourceType::kRos) 
{
    std::cout << "BasicController initialized with RobotState and ControlState." << std::endl;

    // 检查 RobotState 参数的有效性
    if (robot_state_.param.leg_num <= 0 || robot_state_.param.leg_dof <= 0)
    {
        throw std::invalid_argument("Invalid RobotState parameters: leg_num or leg_dof is not valid.");
    }

    // 初始化控制参数
    control_state_.motor_init_torque.setZero();
    control_state_.joint.kp.setZero();
    control_state_.joint.kd.setZero();
}

void BasicController::commandHook(void)
{
    // 默认实现为空
}

void BasicController::updateCommand(CommandType command_type, double dt, uint8_t leg_num,
                                    bool use_joint_vel_cmd)
{
    ROS_ASSERT(dt > 0);

    // zero-initialization
    static Eigen::MatrixXd pre_joint_pos_cmd(robot_state_.param.leg_dof,
                                             robot_state_.param.leg_num);

    std::unique_lock<std::shared_mutex> lock(robot_state_.cmd_mutex);

    // just copy, for debug
    robot_state_.foot.pos_cmd = control_state_.foot.pos_cmd;
    robot_state_.foot.vel_cmd = control_state_.foot.vel_cmd;
    robot_state_.foot.acc_cmd = control_state_.foot.acc_cmd;
    robot_state_.foot.force_cmd = control_state_.foot.force_cmd;

    if (command_type == CommandType::kParam) // update parameters
    {
        for (int i = 0; i < robot_state_.param.leg_num; ++i)
        {
            if (leg_num & LEG(i))
            {
                robot_state_.joint.kp.col(i) = control_state_.joint.kp.col(i);
                robot_state_.joint.kd.col(i) = control_state_.joint.kd.col(i);
            }
        }
    }
    // else if (command_type == CommandType::kFootForce) // foot force commands
    // {
    //     // F = kp * pos_err + kd * vel_err + torque_cmd
    //     // clear joint kp and kd
    //     for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //     {
    //         if (leg_num & LEG(i))
    //         {
    //             robot_state_.joint.kp.col(i).setZero();
    //             robot_state_.joint.kd.col(i).setZero();
    //         }
    //     }

    //     // calculate torque command use jacobian
    //     robot_state_.fdb_mutex.lock_shared();
    //     for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //     {
    //         if (leg_num & (1 << i))
    //         {
    //             // \tau = J^T * F
    //             robot_state_.joint.torque_cmd.col(i) =
    //                 kinematic_.getJacobian(robot_state_.joint.pos_fdb.col(i), i).transpose() *
    //                 control_state_.foot.force_cmd.col(i);
    //         }
    //     }
    //     robot_state_.fdb_mutex.unlock_shared();
    // }
    // else if (command_type ==
    //          CommandType::
    //              kFootPV) // foot position velocity hybrid & joint torque feedforward commands
    // {
    //     // set joint kp and kd
    //     for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //     {
    //         if (leg_num & LEG(i))
    //         {
    //             robot_state_.joint.kp.col(i) = control_state_.joint.kp.col(i);
    //             robot_state_.joint.kd.col(i) = control_state_.joint.kd.col(i);
    //         }
    //     }
    //     for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //     {
    //         if (leg_num & LEG(i))
    //         {
    //             robot_state_.joint.pos_cmd.col(i) =
    //                 kinematic_.inverseKinematic(control_state_.foot.pos_cmd.col(i), i);
    //             if (use_joint_vel_cmd)
    //             {
    //                 /* differential method */
    //                 robot_state_.joint.vel_cmd.col(i) =
    //                     (robot_state_.joint.pos_cmd.col(i) - pre_joint_pos_cmd.col(i)) / dt;
    //                 /* analytical method */
    //                 // robot_state_.fdb_mutex.lock_shared();
    //                 // // dot{q} = J^{-1} * dot{p}
    //                 // Eigen::MatrixXd jacobian = kinematic_.getJacobian(robot_state_.joint.pos_fdb.col(i), i);
    //                 // // Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    //                 // Eigen::MatrixXd jacobian_pinv = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
    //                 // robot_state_.joint.vel_cmd.col(i) = jacobian_pinv * control_state_.foot.vel_cmd.col(i);
    //                 // robot_state_.fdb_mutex.unlock_shared();
    //             }
    //             else
    //             {
    //                 robot_state_.joint.vel_cmd.setZero();
    //             }
    //             // joint torque feedforward
    //             robot_state_.joint.torque_cmd.col(i) = control_state_.joint.torque_cmd.col(i);
    //             // robot_state_.joint.torque_cmd.col(i).setZero();
    //         }
    //     }
    // }
    // else if (command_type ==
    //          CommandType::kFoot) // foot position velocity hybrid & foot torque feedforward commands
    // {
    //     // set joint kp and kd
    //     for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //     {
    //         if (leg_num & LEG(i))
    //         {
    //             robot_state_.joint.kp.col(i) = control_state_.joint.kp.col(i);
    //             robot_state_.joint.kd.col(i) = control_state_.joint.kd.col(i);
    //         }
    //     }
    //     for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //     {
    //         if (leg_num & LEG(i))
    //         {
    //             robot_state_.joint.pos_cmd.col(i) =
    //                 kinematic_.inverseKinematic(control_state_.foot.pos_cmd.col(i), i);
    //             if (use_joint_vel_cmd)
    //             {
    //                 /* differential method */
    //                 robot_state_.joint.vel_cmd.col(i) =
    //                     (robot_state_.joint.pos_cmd.col(i) - pre_joint_pos_cmd.col(i)) / dt;
    //                 /* analytical method */
    //                 // robot_state_.fdb_mutex.lock_shared();
    //                 // // dot{q} = J^{-1} * dot{p}
    //                 // Eigen::MatrixXd jacobian = kinematic_.getJacobian(robot_state_.joint.pos_fdb.col(i), i);
    //                 // // Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    //                 // Eigen::MatrixXd jacobian_pinv = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
    //                 // robot_state_.joint.vel_cmd.col(i) = jacobian_pinv * control_state_.foot.vel_cmd.col(i);
    //                 // robot_state_.fdb_mutex.unlock_shared();
    //             }
    //             else
    //             {
    //                 robot_state_.joint.vel_cmd.setZero();
    //             }
    //             // foot torque feedforward
    //             // calculate torque command use jacobian
    //             robot_state_.fdb_mutex.lock_shared();
    //             for (int i = 0; i < robot_state_.param.leg_num; ++i)
    //             {
    //                 if (leg_num & (1 << i))
    //                 {
    //                     // \tau = J^T * F
    //                     robot_state_.joint.torque_cmd.col(i) =
    //                         kinematic_.getJacobian(robot_state_.joint.pos_fdb.col(i), i)
    //                             .transpose() *
    //                         control_state_.foot.force_cmd.col(i);
    //                 }
    //             }
    //             robot_state_.fdb_mutex.unlock_shared();
    //             // robot_state_.joint.torque_cmd.col(0,i).setZero();
    //         }
    //     }
    // }
    else if (command_type == CommandType::kJoint) // joint commands
    {
        for (int i = 0; i < robot_state_.param.leg_num; ++i)
        {
            if (leg_num & LEG(i))
            {
                robot_state_.joint.kp.col(i) = control_state_.joint.kp.col(i);
                robot_state_.joint.kd.col(i) = control_state_.joint.kd.col(i);
                robot_state_.joint.pos_cmd.col(i) = control_state_.joint.pos_cmd.col(i);
                robot_state_.joint.torque_cmd.col(i) = control_state_.joint.torque_cmd.col(i);

                if (use_joint_vel_cmd)
                    robot_state_.joint.vel_cmd.col(i) = control_state_.joint.vel_cmd.col(i);
                else
                    robot_state_.joint.vel_cmd.col(i).setZero();
            }
        }
    }

    pre_joint_pos_cmd = robot_state_.joint.pos_cmd;

    //commandHook();
}
/**
 * @brief initialize motor
 */
void BasicController::initMotor(void)
{
    const int kLegNum = robot_state_.param.leg_num;
    const int kLegDof = robot_state_.param.leg_dof;

    static int joint_num = 0;
    // TODO: add a first-in flag to solve this problem completely?
    // no motor in hardware or in gazebo, motor feedback equal to zero
    // we need to ensure that last_pos not equal to now_pos at the beginning
    static Eigen::MatrixXd last_pos = Eigen::MatrixXd::Random(kLegDof, kLegNum);
    Eigen::MatrixXd current_pos(kLegDof, kLegNum);
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> init_finished_flag(kLegDof, kLegNum);

    double time_compensate = 0;
    int sum_flag = 0;

    switch (robot_state_.param.motor_init_type)
    {
    case MotorInitType::kAbsoluteEncoder:
        robot_state_.joint.pos_zero.setZero();
        robot_state_.motor_init_finished_flag = true;
        break;
    case MotorInitType::kManualCalibration:
        robot_state_.fdb_mutex.lock_shared();
        current_pos = robot_state_.joint.pos_raw_fdb;
        robot_state_.fdb_mutex.unlock_shared();
        robot_state_.joint.pos_zero = current_pos;
        robot_state_.motor_init_finished_flag = true;
        break;
    // TODO: rewrite this part(Hexapod specific)
    case MotorInitType::kLyingAutoCalibration:
        timer_.start();

        robot_state_.fdb_mutex.lock_shared();
        current_pos = robot_state_.joint.pos_raw_fdb;
        robot_state_.fdb_mutex.unlock_shared();

        init_finished_flag.setZero();

        // determine whether the joint has reached the mechanical limit
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.02)
            {
                init_finished_flag(joint_num, j) = true;
            }
        }

        //  determine whether all six joints in the same position reached the mechanical limit
        sum_flag = 0;
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            sum_flag += init_finished_flag(joint_num, j);
        }

        if (sum_flag == 6)
        {
            // save motor zero position
            robot_state_.cmd_mutex.lock();
            robot_state_.joint.pos_zero.row(joint_num) = current_pos.row(joint_num);
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                // record the zero position
                robot_state_.joint.pos_cmd(joint_num, j) =
                    robot_state_.param.joint_calibration_pos(joint_num, j);
                // clear torque command and set kp, use PD to keep position
                robot_state_.joint.torque_cmd(joint_num, j) = 0;
                control_state_.joint.kp(joint_num, j) = control_state_.joint.kp_default(joint_num);
                robot_state_.joint.kp(joint_num, j) = control_state_.joint.kp(joint_num, j);
            }
            robot_state_.cmd_mutex.unlock();

            ROS_DEBUG_STREAM("joint_pos_zero: " << (current_pos.row(joint_num)));
            // turn to the next joints
            ++joint_num;
        }

        if (joint_num == 3)
        {
            robot_state_.motor_init_finished_flag = true;
        }
        else
        {
            robot_state_.cmd_mutex.lock();
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                if (joint_num > 0)
                    robot_state_.joint.torque_cmd(joint_num, j) =
                        control_state_.motor_init_torque(joint_num);
            }
            robot_state_.cmd_mutex.unlock();
        }

        last_pos = current_pos;

        timer_.stop();

        // maintain a fixed control frequency
        time_compensate =
            1000. / (double)control_state_.motor_init_frequency - timer_.elapsedMilliseconds();
        if (!isfinite(time_compensate))
        {
            ROS_FATAL("error frequency");
            exit(1);
        }
        if (time_compensate > 0)
        {
            // use ros interface to support simulated time
            ros::Duration(time_compensate / 1000.).sleep();
        }
        break;
    case MotorInitType::kAutoCalibration:
        timer_.start();

        robot_state_.fdb_mutex.lock_shared();
        current_pos = robot_state_.joint.pos_raw_fdb;
        robot_state_.fdb_mutex.unlock_shared();

        init_finished_flag.setZero();

        // determine whether the joint has reached the mechanical limit
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            if (std::abs(current_pos(joint_num, j) - last_pos(joint_num, j)) < 0.01)
            {
                init_finished_flag(joint_num, j) = true;
            }
        }

        //  determine whether all six joints in the same position reached the mechanical limit
        sum_flag = 0;
        for (int j = 0; j < robot_state_.param.leg_num; ++j)
        {
            sum_flag += init_finished_flag(joint_num, j);
        }

        if (sum_flag == 6)
        {
            // save motor zero position
            robot_state_.cmd_mutex.lock();
            robot_state_.joint.pos_zero.row(joint_num) = current_pos.row(joint_num);
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                // record the zero position
                robot_state_.joint.pos_cmd(joint_num, j) =
                    robot_state_.param.joint_calibration_pos(joint_num, j);
                // clear torque command and set kp, use PD to keep position
                robot_state_.joint.torque_cmd(joint_num, j) = 0;
                control_state_.joint.kp(joint_num, j) = control_state_.joint.kp_default(joint_num);
                robot_state_.joint.kp(joint_num, j) = control_state_.joint.kp(joint_num, j);
            }
            robot_state_.cmd_mutex.unlock();

            ROS_DEBUG_STREAM("joint_pos_zero: " << (current_pos.row(joint_num)));
            // turn to the next joints
            ++joint_num;
        }

        if (joint_num == 3)
        {
            robot_state_.motor_init_finished_flag = true;
        }
        else
        {
            // set the next joints' commands, then delay and wait for motor running
            robot_state_.cmd_mutex.lock();
            for (int j = 0; j < robot_state_.param.leg_num; ++j)
            {
                robot_state_.joint.torque_cmd(joint_num, j) =
                    control_state_.motor_init_torque(joint_num);
            }
            robot_state_.cmd_mutex.unlock();
        }

        last_pos = current_pos;

        timer_.stop();

        // maintain a fixed control frequency
        time_compensate =
            1000. / (double)control_state_.motor_init_frequency - timer_.elapsedMilliseconds();
        if (!isfinite(time_compensate))
        {
            ROS_FATAL("error frequency");
            exit(1);
        }
        if (time_compensate > 0)
        {
            // use ros interface to support simulated time
            ros::Duration(time_compensate / 1000.).sleep();
            // std::this_thread::sleep_for(std::chrono::microseconds(int(1000 * time_compensate)));
        }
        break;
    }
}