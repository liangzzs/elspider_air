/**
 * @file test_udp_non_blocking.cpp
 * @brief non-blocking UDP communication test
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-09-19
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "udp_comm.h"
#include "basic_controller.h"
#include "elspider_air.h"
#include "robot_state.h"
#include "robot_param.h"
#include "control_state.h"
#include "robot_param.h"

/* c++ standard library header files */
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

UdpComm udp_comm(7, "192.168.1.10", 10);
udp::ReceiveData udp_receive_data = {};
udp::SendData udp_send_data = {};

// 设置非阻塞模式读取键盘输入
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_udp_node");
    ros::NodeHandle nh;
    RobotParam robot_param = buildElspiderAir();
    RobotState robot_state(robot_param);
    ControlState control_state(robot_param);
    BasicController basic_controller(nh, robot_state, control_state);

    // 将robot_state设为全局可访问
    RobotState& robot_state_ = robot_state;

    // udp initialization
    if (!udp_comm.init())
    {
        printf("udp init failed\n");
        exit(1);
    }
    std::cout << "leg_dof: " << robot_param.leg_dof << ", leg_num: " << robot_param.leg_num << std::endl;
    while (true)
    {
        //UDP通信
        udp_send_data.state = static_cast<uint8_t>(CommBoardState::kIdle);
        udp_comm.setSendData(udp_send_data);
        udp_comm.send();
        if (udp_comm.receive(1000))
        {
            udp_receive_data = udp_comm.getReceiveData();
            std::cout << "UDP receive successfully" << std::endl;
        }
        else
        {
            std::cout << "UDP receive timeout" << std::endl;
        }

        // 检查键盘输入
        if (kbhit())
        {
            char c = getchar();
            if (c == 'm') // 按下 'm' 键
            {
                robot_state_.motor_init_request = true;
                std::cout << "Motor initialization request triggered." << std::endl;
            }
        }
        //motor init
        if (robot_state_.motor_init_request)
        {
            static bool first_in_flag = true;
            if (first_in_flag)
            {
                std::thread thread([&robot_state_, &basic_controller]() {
                    while (!robot_state_.motor_init_finished_flag) {
                        basic_controller.initMotor(); // 调用initMotor函数
                    }
                });
                thread.detach();
                first_in_flag = false;
            }
            if (robot_state_.motor_init_finished_flag)
            {
                robot_state_.motor_init_request = false;
                std::cout << "Motor initialization completed successfully." << std::endl;
            }
            else
            {
                std::cout << "Waiting for motor initialization..." << std::endl;
            }
        }

    }
    return 0;
}
