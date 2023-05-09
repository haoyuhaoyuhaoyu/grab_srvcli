#include "rclcpp/rclcpp.hpp"
#include "grab_interface/srv/grab_srv_data.hpp"

#include <memory>
#include "../include/rm_base.h"

#include <thread>
#include "time.h"
#include <chrono>
#include <iostream>

// 手动维护句柄
SOCKHANDLE m_sockhand_left = -1;
SOCKHANDLE m_sockhand_right = -1;

// 休眠(毫秒)
void sleep_cp(int milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#endif

#ifdef __linux
    usleep(milliseconds * 1000);
#endif
}

void MCallback(CallbackData data)
{
    // 判断接口类型
    switch(data.errCode)
    {
        case MOVEJ_CANFD_CB: // 角度透传
            printf("MOVEJ_CANFD 透传结果: %d\r\n", data.errCode);
        break;
        case MOVEP_CANFD_CB: // 位姿透传
            printf("MOVEP_CANFD  透传结果: %d\r\n", data.errCode);
        break;
        case FORCE_POSITION_MOVE_CB: // 力位混合透传
            printf("FORCE_POSITION_MOVE  透传结果: %d\r\n", data.errCode);
        break;
    }

}
void reset_joint_pos(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    float joint[7] = {0};
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("reset_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void reset_hand_pos(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    int joint[6] = {1000, 1000, 1000, 1000, 1000, 1000};
    ret = Set_Hand_Angle(m_sockhand, joint, 1);
    if(ret != 0)
    {
        printf("reset_hand_pos Set_Hand_Angle:%d\r\n",ret);
        return;
    }
}

void hand_grab(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    int joint[6] = {500, 500, 500, 500, 500, -1};
    ret = Set_Hand_Speed(m_sockhand, 500, 1);
    if(ret != 0)
    {
        printf("hand_grab Set_Hand_Speed:%d\r\n",ret);
        return;
    }
    ret = Set_Hand_Force(m_sockhand, 500, 1);
    if(ret != 0)
    {
        printf("hand_grab Set_Hand_Force:%d\r\n",ret);
        return;
    }
    ret = Set_Hand_Angle(m_sockhand, joint, 1);
    if(ret != 0)
    {
        printf("hand_grab Set_Hand_Angle:%d\r\n",ret);
        return;
    }
}

void grab(const std::shared_ptr<grab_interface::srv::GrabSrvData::Request> request,
          std::shared_ptr<grab_interface::srv::GrabSrvData::Response> response)
{
  
  if (request->grab_type == 'a')
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "grab type: %c", request->grab_type);
    response->if_success = 1;
    hand_grab(m_sockhand_left);
    hand_grab(m_sockhand_right);
  }
  else if (request->grab_type == 'b')
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "grab type: %c", request->grab_type);
    response->if_success = 1;
    reset_hand_pos(m_sockhand_left);
    reset_hand_pos(m_sockhand_right);
  }
  else if (request->grab_type == 'c')
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "grab type: %c", request->grab_type);
  }
  else if (request->grab_type == 'q')
  {
    rclcpp::shutdown();
    return;
  }
 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: [%ld]", (long int)response->if_success);
}

void Getpos(int a)
{
  std::cout << "111";
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "a = %d", a);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
   // for rm init
//  RM_API_Init(MCallback);
   // 初始化API, 注册回调函数
//  RM_API_Init(MCallback);
   // 连接服务器
//  m_sockhand_right = Arm_Socket_Start((char*)"192.168.1.17", 8080, ARM_75, 5000);
//  m_sockhand_left = Arm_Socket_Start((char*)"192.168.1.18", 8080, ARM_75, 5000);
//   // reset pos
//   reset_joint_pos(m_sockhand_left);
//   reset_joint_pos(m_sockhand_right);
//  Set_Tool_Voltage(m_sockhand_right, 3, 1);
//  Set_Tool_Voltage(m_sockhand_left, 3, 1);
//  reset_hand_pos(m_sockhand_right);
//  reset_hand_pos(m_sockhand_left);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grab_server");

  rclcpp::Service<grab_interface::srv::GrabSrvData>::SharedPtr service =
    node->create_service<grab_interface::srv::GrabSrvData>("grab_srvcli", &grab);

//trac_ik
  std::string urdf_xml;
  node->declare_parameters<std::string>(
            std::string(),       // parameters are not namespaced
            std::map<std::string, std::string>{
                    {"robot_description", std::string()},
            });
  node->get_parameter("robot_description", urdf_xml);



  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to grab");

  int a = 10;
  std::thread th1(Getpos, a); // for camera

  th1.join();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
