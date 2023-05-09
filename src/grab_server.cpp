#include "rclcpp/rclcpp.hpp"
#include "grab_interface/srv/grab_srv_data.hpp"

#include <memory>
#include "../include/rm_base.h"

#include <thread>
#include "time.h"
#include <chrono>
#include <iostream>

#include "trac_ik/trac_ik.hpp"

#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"


// 手动维护句柄
SOCKHANDLE m_sockhand_left = -1;
SOCKHANDLE m_sockhand_right = -1;

std::shared_ptr<TRAC_IK::TRAC_IK> Tracik_solver_r;
std::shared_ptr<TRAC_IK::TRAC_IK> Tracik_solver_l;

float Joint[7];

KDL::Chain Chain_R;
KDL::JntArray Result;

KDL::JntArray Nominal_r;
KDL::JntArray Nominal_l;

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

void set_joint_pos_r(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    double roll, pitch, yaw;
    KDL::Frame end_effector_pose;

    end_effector_pose.M(0, 0) = -0.985;
    end_effector_pose.M(0, 1) = -0.174;
    end_effector_pose.M(0, 2) = 0.0;
    end_effector_pose.M(1, 0) = 0.0;
    end_effector_pose.M(1, 1) = 0.0;
    end_effector_pose.M(1, 2) = -1.000;
    end_effector_pose.M(2, 0) = 0.174;
    end_effector_pose.M(2, 1) = -0.985;
    end_effector_pose.M(2, 2) = 0;

    end_effector_pose.p[0] = 0.334;
    end_effector_pose.p[1] = -0.577;
    end_effector_pose.p[2] = 0.489;

    KDL::JntArray result;
    int rc = Tracik_solver_r->CartToJnt(Nominal_r, end_effector_pose, result);

    if(rc >= 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%lf %lf %lf %lf %lf %lf %lf", result(0), result(1), result(2),
                    result(3), result(4), result(5), result(6));
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error !");
    }
    float joint[7] = {0};
    for (int j = 0; j < 7; j++) {
        joint[j] = result(j)*57.3;
    }   
    //ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("reset_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void set_joint_pos_l(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    double roll, pitch, yaw;
    KDL::Frame end_effector_pose;

    end_effector_pose.M(0, 0) = -0.985;
    end_effector_pose.M(0, 1) = 0.174;
    end_effector_pose.M(0, 2) = 0.0;
    end_effector_pose.M(1, 0) = 0.0;
    end_effector_pose.M(1, 1) = 0.0;
    end_effector_pose.M(1, 2) = 1.000;
    end_effector_pose.M(2, 0) = 0.174;
    end_effector_pose.M(2, 1) = -0.985;
    end_effector_pose.M(2, 2) = 0;

    end_effector_pose.p[0] = -0.059;
    end_effector_pose.p[1] = 0.861;
    end_effector_pose.p[2] = 0.403;

    KDL::JntArray result;

    int rc = Tracik_solver_l->CartToJnt(Nominal_l, end_effector_pose, result);

    if(rc >= 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "l %lf %lf %lf %lf %lf %lf %lf", result(0), result(1), result(2),
                    result(3), result(4), result(5), result(6));
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "l error !");
    }
    float joint[7] = {0};
    for (int j = 0; j < 7; j++) {
        joint[j] = result(j)*57.3;
    }   
    //ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("reset_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void get_matrix_r(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    double roll, pitch, yaw;
    KDL::Frame end_effector_pose;

    end_effector_pose.M(0, 0) = -0.985;
    end_effector_pose.M(0, 1) = -0.174;
    end_effector_pose.M(0, 2) = 0.0;
    end_effector_pose.M(1, 0) = 0.0;
    end_effector_pose.M(1, 1) = 0.0;
    end_effector_pose.M(1, 2) = -1.000;
    end_effector_pose.M(2, 0) = 0.174;
    end_effector_pose.M(2, 1) = -0.985;
    end_effector_pose.M(2, 2) = 0;

    end_effector_pose.p[0] = 0.334;
    end_effector_pose.p[1] = -0.577;
    end_effector_pose.p[2] = 0.489;

    KDL::JntArray result;
    int rc = Tracik_solver_r->CartToJnt(Nominal_r, end_effector_pose, result);

    if(rc >= 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%lf %lf %lf %lf %lf %lf %lf", result(0), result(1), result(2),
                    result(3), result(4), result(5), result(6));
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error !");
    }
    float joint[7] = {0};
    for (int j = 0; j < 7; j++) {
        joint[j] = result(j)*57.3;
    }   
    //ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
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
    set_joint_pos_r(m_sockhand_right);
    set_joint_pos_l(m_sockhand_left);
  }
  else if (request->grab_type == 't')
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "grab type: %c", request->grab_type);
    get_matrix_r(m_sockhand_right);
    get_matrix_l(m_sockhand_left);
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

   TRAC_IK::TRAC_IK tracik_solver("pelvis", "r_link7", urdf_xml, 0.05, 1e-3);
   Tracik_solver_r  = std::make_shared<TRAC_IK::TRAC_IK>("pelvis", "r_link7", urdf_xml, 0.05, 1e-3);

    KDL::Chain chain;
    KDL::JntArray ll, ul;  // lower joint limits, upper joint limits

    bool valid = tracik_solver.getKDLChain(chain);

    if (!valid) {
        RCLCPP_ERROR(node->get_logger(), "There was no valid KDL chain found");
        return -1;
    }

    valid = tracik_solver.getKDLLimits(ll, ul);

    if (!valid) {
        RCLCPP_ERROR(node->get_logger(), "There were no valid KDL joint limits found");
        return -1;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    RCLCPP_INFO(node->get_logger(), "Using %d joints", chain.getNrOfJoints());

 // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver(chain);  // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver(chain);  // PseudoInverse vel solver
    // Joint Limit Solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, 1e-3);
    // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j = 0; j < nominal.data.size(); j++) {
        nominal(j) = (ll(j) + ul(j)) / 2.0;
    }
    Nominal_r = nominal;
    KDL::Frame end_effector_pose;
    float jointtt[7];
    Get_Joint_Degree (m_sockhand_right, jointtt);
    for (uint j = 0; j < nominal.data.size(); j++) {
        nominal(j) = jointtt[j]/57.3;
    }
    fk_solver.JntToCart(nominal, end_effector_pose);
    std::cout<<end_effector_pose.

    TRAC_IK::TRAC_IK tracik_solver_2("pelvis", "l_link7", urdf_xml, 0.05, 1e-3);
    Tracik_solver_l = std::make_shared<TRAC_IK::TRAC_IK>("pelvis", "l_link7", urdf_xml, 0.05, 1e-3);
    KDL::Chain chain_2;
    KDL::JntArray ll_2, ul_2;

    bool valid_2 = tracik_solver_2.getKDLChain(chain_2);

    if (!valid_2) {
        RCLCPP_ERROR(node->get_logger(), "There was no valid KDL chain found");
        return -1;
    }

    valid_2 = tracik_solver_2.getKDLLimits(ll_2, ul_2);

    if (!valid_2) {
        RCLCPP_ERROR(node->get_logger(), "There were no valid KDL joint limits found");
        return -1;
    }

    assert(chain_2.getNrOfJoints() == ll_2.data.size());
    assert(chain_2.getNrOfJoints() == ul_2.data.size());

    RCLCPP_INFO(node->get_logger(), "Using %d joints", chain_2.getNrOfJoints());

    // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver_2(chain_2);  // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver_2(chain_2);  // PseudoInverse vel solver
    // Joint Limit Solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver_2(chain_2, ll_2, ul_2, fk_solver_2, vik_solver_2, 1, 1e-3);
    // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

    KDL::JntArray nominal_2(chain_2.getNrOfJoints());

    for (uint j = 0; j < nominal_2.data.size(); j++) {
        nominal_2(j) = (ll_2(j) + ul_2(j)) / 2.0;
    }

    Nominal_l = nominal_2;

    //fk_solver_2.JntToCart(nominal_2, end_effector_pose_2);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to grab");

  int a = 10;
  std::thread th1(Getpos, a); // for camera

  th1.join();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
