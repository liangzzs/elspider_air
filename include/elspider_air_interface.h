/**
 * @file elspider_air_interface.h
 * @author Master Yip (2205929492@qq.com), Haoyu Wang (qrpucp@qq.com)
 * @brief elspider air hardware interface, use UDP to communicate with HexapodCommBoard
 * @version 0.1
 * @date 2024-01-10
 *
 * @copyright Copyright (c) 2024
 *
 */

 #pragma once
 //#include "common/utilities/pin_header.h"
 /* related header files */
 
 /* c system header files */
 
 /* c++ standard library header files */
 
 /* external project header files */
 //#include <sensor_msgs/Imu.h>
 //#include <sensor_msgs/Joy.h>
 
 /* internal project header files */
 #include "basic_interface.h"
 #include "udp_comm.h"
 //#include "interface/ros_joystick/joystick_manager.h"
 
 class ElspiderAirInterface : public BasicInterface
 {
   public:
     ElspiderAirInterface(ros::NodeHandle &r_nh, RobotState &r_robot_state);
     ~ElspiderAirInterface() override = default;
     void read(void) override; // 将 read 方法设置为 public
     void write(void) override;
     bool udp_ready_flag_;
     
 
   private:
     //void read(void) override;
     //void write(void) override;
 
     //void updateJoystick(void);
     //void updateImu(void);
     void receiveMotorFeedback(void);
     void sendMotorCommand(void);
     void updateMotorFeedback(void);
     void updateMotorCommand(void);
     //void imuCallback(const sensor_msgs::Imu &imu);
 
     // ros joystick
     //JoystickManager joystick_manager_;
     //sensor_msgs::Joy joy_msg_;
     //bool joy_ready_flag_{};
 
     // udp
     UdpComm udp_comm_;
     udp::ReceiveData udp_receive_data_{};
     udp::SendData udp_send_data_{};
     //bool udp_ready_flag_{};
 
     // imu
    //  ros::Subscriber sub_imu_msg_;
    //  sensor_msgs::Imu imu_msg_;
    //  bool imu_ready_flag_{};
 };
 