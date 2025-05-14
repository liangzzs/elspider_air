/**
 * @file elspider_air_interface.cpp
 * @author Master Yip (2205929492@qq.com), Haoyu Wang (qrpucp@qq.com)
 * @brief elspider air hardware interface, use UDP to communicate with HexapodCommBoard
 * @version 0.1
 * @date 2024-01-10
 *
 * @copyright Copyright (c) 2024
 *
 */

/* related header files */
#include "elspider_air_interface.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
//#include "common/utilities/basic_func.hpp"
#include "basic_math.hpp"

/**
 * @brief UDP config in CommBoard
 */
const int kLocalPort = 7;
const int kTargetPort = 10;
const std::string kTargetIp = "192.168.5.10";

// STM32 order {5, 1, 4, 2, 3, 0}
const int stm32LegRemap[6] = {5, 1, 4, 2, 3, 0};

// clang-format off
// Redirect before remapping: Fixed order(5, 1, 3, 4, 2, 0)
const int stm32MotorRedirect[18] = {-1, 1, 1,   -1, -1, -1,   -1, -1, -1,
                                    -1, -1, -1,   -1, 1, 1,   -1, -1, -1};
// Redirect after remapping
// const int stm32MotorRedirect[18] = {1, 1, 1, -1, -1, -1, 1, -1, -1,
//                                     1, 1, 1, -1, -1, -1, 1, -1, -1};
// clang-format on

/**
 * @brief Construct a new Elspider Mini Interface:: Elspider Mini Interface object
 * @param[in] r_nh the reference to ros node handle
 * @param[in] r_robot_state the reference to robot state
 * @param[in] r_joy_msg the reference to joy message
 */
ElspiderAirInterface::ElspiderAirInterface(ros::NodeHandle &r_nh, RobotState &r_robot_state)
    : BasicInterface(r_nh, r_robot_state), udp_comm_(kLocalPort, kTargetIp, kTargetPort)
{
    if (!udp_comm_.init())
        exit(1);

    //sub_imu_msg_ = nh_.subscribe("/wheeltec/imu", 1, &ElspiderAirInterface::imuCallback, this);

    fdb_monitor_thread_ = std::thread([&]() {
        while (!feedback_ready_flag_ && ros::ok()) // 100Hz
        {
            std::string input_str{};
            std::cin >> input_str;
            if (feedback_ready_flag_)
                break;
            if (input_str == "debug")
            {
                std::cout << "udp_ready_flag_: " << udp_ready_flag_ << std::endl;
            }
            else if (input_str == "force")
            {
                feedback_ready_flag_ = true;
                break;
            }
            usleep(10000);
        }
        std::cout << "feedback monitor thread destroyed" << std::endl;
    });
    fdb_monitor_thread_.detach();
}

/**
 * @brief read feedback in elspider mini interface
 */
void ElspiderAirInterface::read(void)
{
    // if (!feedback_ready_flag_ && joy_ready_flag_ && udp_ready_flag_ && imu_ready_flag_)
    if (!feedback_ready_flag_ && udp_ready_flag_)
        feedback_ready_flag_ = true;

    updateJoystick();
    updateImu();
    receiveMotorFeedback();
    updateMotorFeedback();
}

/**
 * @brief write commands in elspider mini interface
 */
void ElspiderAirInterface::write(void)
{
    updateMotorCommand();
    sendMotorCommand();
}

/**
 * @brief receive motor feedback from HexapodCommBoard through UDP
 */
void ElspiderAirInterface::receiveMotorFeedback(void)
{
    // non-blocking
    // udp_comm_.send();
    if (udp_comm_.receive(10)) // 10ms
    {
        if (!udp_ready_flag_)
        {    udp_ready_flag_ = true;
            std::cout<< "udp ready" << std::endl;
        }
        udp_receive_data_ = udp_comm_.getReceiveData();
    }
    // blocking
    // udp_comm_.receive();
}

/**
 * @brief send motor command to HexapodCommBoard through UDP
 */
void ElspiderAirInterface::sendMotorCommand(void)
{
    udp_comm_.setSendData(udp_send_data_);
    udp_comm_.send();
}

/**
 * @brief transform raw feedback data to robot state
 */
void ElspiderAirInterface::updateMotorFeedback(void)
{
    for (int k = 0; k < robot_param_.leg_num; ++k)
    {
        int i = stm32LegRemap[k];
        for (int j = 0; j < robot_param_.leg_dof; ++j)
        {
            int id = robot_param_.leg_dof * i + j;
            // int id2 = robot_param_.leg_dof * k + j;
            motor_receive_.at(j).at(k).pos =
                udp_receive_data_.udp_motor_receive[id].pos /
                robot_param_.motor_reduction_ratio(j) / robot_param_.external_reduction_ratio(j) *
                robot_param_.leg_mirror_coe(k) * stm32MotorRedirect[id];
            motor_receive_.at(j).at(k).vel =
                udp_receive_data_.udp_motor_receive[id].vel /
                robot_param_.motor_reduction_ratio(j) / robot_param_.external_reduction_ratio(j) *
                robot_param_.leg_mirror_coe(k) * stm32MotorRedirect[id];
            motor_receive_.at(j).at(k).acc =
                udp_receive_data_.udp_motor_receive[id].acc /
                robot_param_.motor_reduction_ratio(j) / robot_param_.external_reduction_ratio(j) *
                robot_param_.leg_mirror_coe(k) * stm32MotorRedirect[id];
            motor_receive_.at(j).at(k).torque =
                udp_receive_data_.udp_motor_receive[id].torque *
                robot_param_.motor_reduction_ratio(j) * robot_param_.external_reduction_ratio(j) *
                robot_param_.leg_mirror_coe(k) * stm32MotorRedirect[id];
            motor_receive_.at(j).at(k).temp = udp_receive_data_.udp_motor_receive[id].temp;
        }
    }
}

/**
 * @brief transform motor command to [udp_send_data] format
 */
[[deprecated]] void ElspiderAirInterface::updateMotorCommand(void)
{
    // set CommBoard state
    // udp_send_data_.state = (uint8_t)(CommBoardState)robot_state_.comm_board_state;
    udp_send_data_.state = static_cast<uint8_t>(static_cast<CommBoardState>(comm_board_state_));

    // set motor command
    for (int k = 0; k < robot_param_.leg_num; ++k)
    // for (int k = 4; k <5; ++k)
    {
        int i = stm32LegRemap[k];
        for (int j = 0; j < robot_param_.leg_dof; ++j)
        {
            int id = robot_param_.leg_dof * i + j;
            udp_send_data_.udp_motor_send[id].kp = motor_send_.at(j).at(k).kp;
            udp_send_data_.udp_motor_send[id].kd = motor_send_.at(j).at(k).kd;
            udp_send_data_.udp_motor_send[id].pos =
                motor_send_.at(j).at(k).pos * robot_param_.motor_reduction_ratio(j) *
                robot_param_.external_reduction_ratio(j) * robot_param_.leg_mirror_coe(k) *
                stm32MotorRedirect[id];
            udp_send_data_.udp_motor_send[id].vel =
                motor_send_.at(j).at(k).vel * robot_param_.motor_reduction_ratio(j) *
                robot_param_.external_reduction_ratio(j) * robot_param_.leg_mirror_coe(k) *
                stm32MotorRedirect[id];
            udp_send_data_.udp_motor_send[id].torque =
                motor_send_.at(j).at(k).torque / robot_param_.motor_reduction_ratio(j) /
                robot_param_.external_reduction_ratio(j) * robot_param_.leg_mirror_coe(k) *
                stm32MotorRedirect[id];
        }
    }
}

