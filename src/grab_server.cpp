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

#include <Eigen/Dense>


#include "udp.h"

using Eigen::MatrixXd;

// 手动维护句柄
SOCKHANDLE m_sockhand_left = -1;
SOCKHANDLE m_sockhand_right = -1;

bool IF_QUIT = false;
//udp
union robot_state_u{
    double id_d_xyz[5];
    uint8_t buffer[40];
};

struct robot_state_req{
    uint16_t cmd_no;
    uint16_t length;
    uint32_t counter;
};

union robot_state_req_u{
    double id_d_xyz[5];
    uint8_t buffer[40];
};

robot_state_u rs_u;

//tracik
std::shared_ptr<TRAC_IK::TRAC_IK> Tracik_solver_r;
std::shared_ptr<TRAC_IK::TRAC_IK> Tracik_solver_l;

std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
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
void reset_joint_pos_walk_r(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    float joint[7] = {-88, -64, 86, -24, -1.7, -57, -77};  //for walk
    //float joint[7] = {-89.9, -61.4, 91.8, -41, 2.52, -50, -90}; //for grab
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("reset_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}
void reset_joint_pos_r(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    float joint[7] = {-89.9, -61.4, 91.8, -41, 2.52, -50, -90}; //for grab
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("reset_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}
void reset_joint_pos_l(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    float joint[7] = {103.344, 64.864, -89.756, 21.254, 10.989, 65.736, -112.714};
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

    end_effector_pose.M(0, 0) = 0.0918854;
    end_effector_pose.M(0, 1) = -0.316273;
    end_effector_pose.M(0, 2) = 0.944208;
    end_effector_pose.M(1, 0) = -0.969049;
    end_effector_pose.M(1, 1) = -0.24659;
    end_effector_pose.M(1, 2) = 0.0117049;
    end_effector_pose.M(2, 0) = 0.22913;
    end_effector_pose.M(2, 1) = -0.91606;
    end_effector_pose.M(2, 2) = -0.329142;

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

    end_effector_pose.M(0, 0) = -0.156173;
    end_effector_pose.M(0, 1) = -0.169;
    end_effector_pose.M(0, 2) = -0.973164;
    end_effector_pose.M(1, 0) = -0.982825;
    end_effector_pose.M(1, 1) = 0.124654;
    end_effector_pose.M(1, 2) = 0.136076;
    end_effector_pose.M(2, 0) = 0.0983117;
    end_effector_pose.M(2, 1) = 0.977702;
    end_effector_pose.M(2, 2) = -0.185565;

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



void set_end_pos_r(SOCKHANDLE m_sockhand)
{
    int ret = -1;
    // 回零位
    double roll, pitch, yaw;
    KDL::Frame end_effector_pose;

    end_effector_pose.M(0, 0) = 0.0139876;
    end_effector_pose.M(0, 1) = -0.133365;
    end_effector_pose.M(0, 2) = 0.990968;
    end_effector_pose.M(1, 0) = -0.889726;
    end_effector_pose.M(1, 1) = -0.453908;
    end_effector_pose.M(1, 2) = -0.0485285;
    end_effector_pose.M(2, 0) = 0.45628;
    end_effector_pose.M(2, 1) = -0.881012;
    end_effector_pose.M(2, 2) = -0.125007;

    // init pos
    //  end_effector_pose.p[0] = 0.136268 + 0.02;
    //  end_effector_pose.p[1] = -0.438657 - 0.01;
    //  end_effector_pose.p[2] = -0.05049 + 0.05;
    
    // end_effector_pose.p[0] = 0.46;
    //  end_effector_pose.p[1] = -0.27;
    //  end_effector_pose.p[2] = 0.27049;
    end_effector_pose.p[0] = rs_u.id_d_xyz[2]-0.15;
    end_effector_pose.p[1] = rs_u.id_d_xyz[3]-0.08;
    end_effector_pose.p[2] = rs_u.id_d_xyz[4]-0.09;


    KDL::JntArray result;
    float temp_joint[7] = {0};
    Get_Joint_Degree (m_sockhand, temp_joint);
    for(int i=0; i<7; i++)
    {
        Nominal_r(i) = temp_joint[i]/57.3;
    }

    int rc = Tracik_solver_r->CartToJnt(Nominal_r, end_effector_pose, result);

    if(rc >= 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r %lf %lf %lf %lf %lf %lf %lf", result(0), result(1), result(2),
                    result(3), result(4), result(5), result(6));
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r error !");
        return;
    }
    float joint[7] = {0};
    for (int j = 0; j < 7; j++) {
        joint[j] = result(j)*57.3;
    }   
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("set_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void rot_end_pos_r(SOCKHANDLE m_sockhand, float degree)
{
    int ret = -1;
    // 回零位
    double roll, pitch, yaw;
    KDL::Frame end_effector_pose;

    MatrixXd rot_matrix(3,3);
    rot_matrix<<1,0,0,0,cos(degree/57.3),sin(degree/57.3),0,-sin(degree/57.3),cos(degree/57.3);

    // current pos
    float temp_joint[7] = {0};
    Get_Joint_Degree (m_sockhand, temp_joint);
    for(int i=0; i<7; i++)
    {
        Nominal_r(i) = temp_joint[i]/57.3;
    }

    fk_solver->JntToCart(Nominal_r, end_effector_pose);

    MatrixXd end_matrix(3,3);
    end_matrix<<end_effector_pose.M(0, 0),end_effector_pose.M(0, 1),end_effector_pose.M(0, 2),
    end_effector_pose.M(1, 0),end_effector_pose.M(1, 1),end_effector_pose.M(1, 2),
    end_effector_pose.M(2, 0),end_effector_pose.M(2, 1),end_effector_pose.M(2, 2);

    end_matrix = rot_matrix*end_matrix;
    end_effector_pose.M(0, 0) = end_matrix(0,0);
    end_effector_pose.M(0, 1) = end_matrix(0,1);
    end_effector_pose.M(0, 2) = end_matrix(0,2);
    end_effector_pose.M(1, 0) = end_matrix(1,0);
    end_effector_pose.M(1, 1) = end_matrix(1,1);
    end_effector_pose.M(1, 2) = end_matrix(1,2);
    end_effector_pose.M(2, 0) = end_matrix(2,0);
    end_effector_pose.M(2, 1) = end_matrix(2,1);
    end_effector_pose.M(2, 2) = end_matrix(2,2);

    // init pos
    //  end_effector_pose.p[0] = 0.136268 + 0.02;
    //  end_effector_pose.p[1] = -0.438657 - 0.01;
    //  end_effector_pose.p[2] = -0.05049 + 0.05;
    
    // end_effector_pose.p[0] = 0.46;
    //  end_effector_pose.p[1] = -0.27;
    //  end_effector_pose.p[2] = 0.27049;
    end_effector_pose.p[0] = rs_u.id_d_xyz[2];
    end_effector_pose.p[1] = rs_u.id_d_xyz[3];
    end_effector_pose.p[2] = rs_u.id_d_xyz[4]-0.03;


    KDL::JntArray result;
    Get_Joint_Degree (m_sockhand, temp_joint);
    for(int i=0; i<7; i++)
    {
        Nominal_r(i) = temp_joint[i]/57.3;
    }

    int rc = Tracik_solver_r->CartToJnt(Nominal_r, end_effector_pose, result);

    if(rc >= 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r %lf %lf %lf %lf %lf %lf %lf", result(0), result(1), result(2),
                    result(3), result(4), result(5), result(6));
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r error !");
        return;
    }
    float joint[7] = {0};
    for (int j = 0; j < 7; j++) {
        joint[j] = result(j)*57.3;
    }   
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("set_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void set_end_relative_pos_r(SOCKHANDLE m_sockhand, float *pos)
{
    int ret = -1;
    // 回零位
    double roll, pitch, yaw;
    KDL::Frame end_effector_pose;
    // current pos
    float temp_joint[7] = {0};
    Get_Joint_Degree (m_sockhand, temp_joint);
    for(int i=0; i<7; i++)
    {
        Nominal_r(i) = temp_joint[i]/57.3;
    }

    fk_solver->JntToCart(Nominal_r, end_effector_pose);

    end_effector_pose.p[0] = end_effector_pose.p[0]+pos[0];
    end_effector_pose.p[1] = end_effector_pose.p[1]+pos[1];
    end_effector_pose.p[2] = end_effector_pose.p[2]+pos[2];

    KDL::JntArray result;

    int rc = Tracik_solver_r->CartToJnt(Nominal_r, end_effector_pose, result);

    if(rc >= 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r %lf %lf %lf %lf %lf %lf %lf", result(0), result(1), result(2),
                    result(3), result(4), result(5), result(6));
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r error !");
        return;
    }
    float joint[7] = {0};
    for (int j = 0; j < 7; j++) {
        joint[j] = result(j)*57.3;
    }   
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("set_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void set_end_relative_rot_r(SOCKHANDLE m_sockhand, float degree, int speed=20)
{
    int ret = -1;
    // current pos
    float temp_joint[7] = {0};
    Get_Joint_Degree (m_sockhand, temp_joint);

    temp_joint[6] = temp_joint[6] + degree;

    ret = Movej_Cmd(m_sockhand, temp_joint, speed, 0, 1);
    if(ret != 0)
    {
        printf("set_joint_pos Movej_Cmd 1:%d\r\n",ret);
        return;
    }
}

void set_joint_relative_rot_r(SOCKHANDLE m_sockhand, int id, float degree, int speed=20)
{
    int ret = -1;
    // current pos
    float temp_joint[7] = {0};
    Get_Joint_Degree (m_sockhand, temp_joint);

    temp_joint[id] = temp_joint[id] + degree;

    ret = Movej_Cmd(m_sockhand, temp_joint, speed, 0, 1);
    if(ret != 0)
    {
        printf("set_arm_relative_rot_r Movej_Cmd r:%d\r\n",ret);
        return;
    }
}

void set_thumb_pos(SOCKHANDLE m_sockhand, int degree)
{
    int ret = -1;
    // 回零位
    int joint[6] = {-1, -1, -1, -1, -1, degree};
    ret = Set_Hand_Angle(m_sockhand, joint, 1);
    if(ret != 0)
    {
        printf("reset_hand_pos Set_Hand_Angle:%d\r\n",ret);
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
    int joint[6] = {500, 500, 500, 500, 1000, 500};
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

void hand_grab_process(SOCKHANDLE m_sockhand)
{
    //3
    int hand_speed, hand_force;
    int ret = -1;
    int hand_angle[6];
    hand_speed = 300;
    hand_force = 200;
    hand_angle[0] = 800; hand_angle[1] = 800; hand_angle[2] = 800;
    hand_angle[3] = 800; hand_angle[4] = 900;  hand_angle[5] = -1;
    ret = Set_Hand_Speed(m_sockhand, hand_speed,1);
    ret = Set_Hand_Force (m_sockhand, hand_force, 1);
    ret = Set_Hand_Angle (m_sockhand, hand_angle, 1);
    sleep_cp(500);
    //4
    for(int i = 0;i<=2;i++)
    {
        hand_speed = 500;
        hand_force = 450;
        hand_angle[0] = 500; hand_angle[1] = 500; hand_angle[2] = 500;
        hand_angle[3] = 500; hand_angle[4] = 500;  hand_angle[5] = -1;
        ret = Set_Hand_Speed(m_sockhand, hand_speed,1);
        ret = Set_Hand_Force (m_sockhand, hand_force, 1);
        ret = Set_Hand_Angle (m_sockhand, hand_angle, 1);
        sleep_cp(500);
    }
}

void hand_release_process(SOCKHANDLE m_sockhand)
{
    int hand_speed, hand_force;
    int ret = -1;
    int hand_angle[6];
    hand_speed = 100;
    hand_force = 200;
    hand_angle[0] = 1000; hand_angle[1] = 1000; hand_angle[2] = 1000;
    hand_angle[3] = 1000; hand_angle[4] = 1000;  hand_angle[5] = -1;
    ret = Set_Hand_Speed(m_sockhand, hand_speed,1);
    ret = Set_Hand_Force (m_sockhand, hand_force, 1);
    ret = Set_Hand_Angle (m_sockhand, hand_angle, 1);
    sleep_cp(500);

}



void grab(const std::shared_ptr<grab_interface::srv::GrabSrvData::Request> request,
          std::shared_ptr<grab_interface::srv::GrabSrvData::Response> response)
{
  
  if (request->grab_type == 'a')
  {
    set_end_relative_rot_r(m_sockhand_right, 5);
  }
//   else if (request->grab_type == 'b')
//   {
//     set_end_relative_rot_r(m_sockhand_right, -5);
//   }
//   else if (request->grab_type == 'z')
//   {
//     set_joint_relative_rot_r(m_sockhand_right, 5, 5);
//   }
//   else if (request->grab_type == 'x')
//   {
//     set_joint_relative_rot_r(m_sockhand_right, 5, -5);
//   }
//   else if (request->grab_type == 'y')
//   {
//     set_end_pos_r(m_sockhand_right);
//     //rot_end_pos_r(m_sockhand_right, -10);
//   }
//   else if (request->grab_type == 'i')
//   {
//     float pos[3] = {0, 0, 0.01};
//     set_end_relative_pos_r(m_sockhand_right, pos);
//   }
//   else if (request->grab_type == 'k')
//   {
//     float pos[3] = {0, 0, -0.01};
//     set_end_relative_pos_r(m_sockhand_right, pos);
//   }
//   else if (request->grab_type == 'j')
//   {
//     float pos[3] = {0, 0.01, 0};
//     set_end_relative_pos_r(m_sockhand_right, pos);
//   }
//   else if (request->grab_type == 'l')
//   {
//     float pos[3] = {0, -0.01, 0};
//     set_end_relative_pos_r(m_sockhand_right, pos);
//   }
//   else if (request->grab_type == 'u')
//   {
//     float pos[3] = {0.01, 0, 0};
//     set_end_relative_pos_r(m_sockhand_right, pos);
//   }
//   else if (request->grab_type == 'o')
//   {
//     float pos[3] = {-0.01, 0, 0};
//     set_end_relative_pos_r(m_sockhand_right, pos);
//   }
  else if (request->grab_type == 'c')
  {
    reset_joint_pos_walk_r(m_sockhand_right); // init pos for walk
  }
  else if (request->grab_type == 'v')
  {
    reset_joint_pos_r(m_sockhand_right); // init pos for grab
  }
  else if (request->grab_type == 't') // pour water
  {
    //set_joint_relative_rot_r(m_sockhand_right, 5, -15);
    //sleep(1);
    
    set_end_pos_r(m_sockhand_right);
    sleep_cp(1000);

    set_thumb_pos(m_sockhand_right, 100);
    sleep_cp(1000);


    float pos[3] = {0.04, 0.04, 0};
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(1000);


    hand_grab_process(m_sockhand_right);
    sleep_cp(1000);


    pos[0]=0; pos[1]=0; pos[2]=0.1;
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(1000);


    set_end_relative_rot_r(m_sockhand_right, -45);
    sleep_cp(100);

    pos[0]=0; pos[1]=0.04; pos[2]=0.02;
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(100);

    set_end_relative_rot_r(m_sockhand_right, -58, 15);
    sleep_cp(1000);


    set_end_relative_rot_r(m_sockhand_right, 58, 10);
    sleep_cp(500);

    pos[0]=0.04; pos[1]=-0.06; pos[2]=-0.02;
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(500);

    set_end_relative_rot_r(m_sockhand_right, 45);
    sleep_cp(100);

    set_joint_relative_rot_r(m_sockhand_right, 5, -10);
    sleep_cp(100);
    
    pos[0]=0; pos[1]=0; pos[2]=-0.13;
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(300);

    hand_release_process(m_sockhand_right);
    sleep_cp(1000);


    pos[0]=-0.02; pos[1]=0; pos[2]=0;
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(100);
    pos[0]=-0.08; pos[1]=-0.08; pos[2]=0;
    set_end_relative_pos_r(m_sockhand_right, pos);
    sleep_cp(1000);


    reset_hand_pos(m_sockhand_right);
    sleep_cp(100);
    reset_joint_pos_r(m_sockhand_right);
    sleep_cp(100);


    //sleep(1);
   //rot_end_pos_r(m_sockhand_right, 10);
  }

  else if (request->grab_type == 'w')
  {
    reset_joint_pos_r(m_sockhand_right);
  }
  
  else if (request->grab_type == 'e') //shake hand
  {
    
    // float pos[3] = {0, 0, 0};
    // set_end_relative_pos_r(m_sockhand_right, pos);
    // sleep(2);
    // hand_grab(m_sockhand_right);
    // for(int i =0; i<3; i++)
    // {
    // pos[2] = 0.04;
    // set_end_relative_pos_r(m_sockhand_right, pos);
    // sleep(1);
    // pos[2] = -0.04;
    // set_end_relative_pos_r(m_sockhand_right, pos);
    // sleep(1);
    // }
    // pos[0] = 0;
    // set_end_relative_pos_r(m_sockhand_right, pos);
    // reset_hand_pos(m_sockhand_right);
    
    hand_grab(m_sockhand_right);

    set_joint_relative_rot_r(m_sockhand_right, 5, -10);
    sleep_cp(10);
    for(int i=0; i<3; i++)
    {
        set_joint_relative_rot_r(m_sockhand_right, 5, 30, 50);
        sleep_cp(10);
        set_joint_relative_rot_r(m_sockhand_right, 5, -30, 50);
        sleep_cp(10);
    }
    
    reset_hand_pos(m_sockhand_right);
    reset_joint_pos_r(m_sockhand_right);

  }
  else if (request->grab_type == 'q')
  {
    IF_QUIT = true;
    rclcpp::shutdown();
    return;
  }
 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: [%ld]", (long int)response->if_success);
}

void Get_pos()
{
  struct sockaddr_storage src_addr = {0};
    socklen_t addrlen = sizeof src_addr;

    char *iface_addr_str = "0.0.0.0";
    char *iface_port_str = "10001";

    int sock = udp_init_host(iface_addr_str, iface_port_str);
    //std::cout<<"server:"<<std::endl;

    while(!IF_QUIT) 
    {

        ssize_t nbytes;
        nbytes = wait_for_packet(sock, rs_u.buffer, 40,
                                 (struct sockaddr *) &src_addr, &addrlen);
        std::cout<<static_cast<double>(rs_u.id_d_xyz[0])<<' '<<static_cast<double>(rs_u.id_d_xyz[1])<<' '<<static_cast<double>(rs_u.id_d_xyz[2])<<' '<<static_cast<double>(rs_u.id_d_xyz[3])
        <<' '<<static_cast<double>(rs_u.id_d_xyz[4])<<std::endl;
       // if(nbytes == 124){
        //    for (int i = 0; i < 8; ++i) {
       //         std::cout<<rs_u.rs.task_space_position[2]<<" "<<rs_u.rs.task_space_position[1]<<" "<<rs_u.rs.task_space_position[0]<<std::endl;
       //     }
       // }
        sleep(1);
    }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
   // for rm init
  RM_API_Init(MCallback);
   // 初始化API, 注册回调函数
  RM_API_Init(MCallback);
   // 连接服务器
  m_sockhand_right = Arm_Socket_Start((char*)"192.168.1.17", 8080, ARM_75, 5000);
  m_sockhand_left = Arm_Socket_Start((char*)"192.168.1.18", 8080, ARM_75, 5000);
   // reset pos
  reset_joint_pos_l(m_sockhand_left);
  reset_joint_pos_r(m_sockhand_right);
  //reset_joint_pos_walk_r(m_sockhand_right);
  Set_Tool_Voltage(m_sockhand_right, 3, 1);
  Set_Tool_Voltage(m_sockhand_left, 3, 1);
  reset_hand_pos(m_sockhand_right);
  reset_hand_pos(m_sockhand_left);

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

   //TRAC_IK::TRAC_IK tracik_solver("pelvis", "r_link7", urdf_xml, 0.05, 1e-3);
   Tracik_solver_r  = std::make_shared<TRAC_IK::TRAC_IK>("pelvis", "r_link7", urdf_xml, 0.05, 1e-3);

    KDL::Chain chain;
    KDL::JntArray ll, ul;  // lower joint limits, upper joint limits

    bool valid = Tracik_solver_r->getKDLChain(chain);

    if (!valid) {
        RCLCPP_ERROR(node->get_logger(), "There was no valid KDL chain found");
        return -1;
    }

    valid = Tracik_solver_r->getKDLLimits(ll, ul);

    if (!valid) {
        RCLCPP_ERROR(node->get_logger(), "There were no valid KDL joint limits found");
        return -1;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    RCLCPP_INFO(node->get_logger(), "Using %d joints", chain.getNrOfJoints());

 // Set up KDL IK
    //KDL::ChainFkSolverPos_recursive fk_solver(chain);  // Forward kin. solver
    fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    KDL::ChainIkSolverVel_pinv vik_solver(chain);  // PseudoInverse vel solver
    // Joint Limit Solver
    
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, *fk_solver, vik_solver, 1, 1e-3);
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
    fk_solver->JntToCart(nominal, end_effector_pose);
    std::cout<<"right matrix"<<std::endl;
    std::cout<<end_effector_pose.M(0, 0)<<"  "<<end_effector_pose.M(0, 1)<<"  "<<end_effector_pose.M(0, 2)<<std::endl;
    std::cout<<end_effector_pose.M(1, 0)<<"  "<<end_effector_pose.M(1, 1)<<"  "<<end_effector_pose.M(1, 2)<<std::endl;
    std::cout<<end_effector_pose.M(2, 0)<<"  "<<end_effector_pose.M(2, 1)<<"  "<<end_effector_pose.M(2, 2)<<std::endl;
    std::cout<<"right pos"<<std::endl;
    std::cout<<end_effector_pose.p(0)<<"  "<<end_effector_pose.p(1)<<"  "<<end_effector_pose.p(2)<<std::endl;

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

    Get_Joint_Degree (m_sockhand_left, jointtt);
    for (uint j = 0; j < nominal.data.size(); j++) {
        nominal(j) = jointtt[j]/57.3;
    }
    fk_solver_2.JntToCart(nominal, end_effector_pose);
    std::cout<<"left matrix"<<std::endl;
    std::cout<<end_effector_pose.M(0, 0)<<end_effector_pose.M(0, 1)<<end_effector_pose.M(0, 2)<<std::endl;
    std::cout<<end_effector_pose.M(1, 0)<<end_effector_pose.M(1, 1)<<end_effector_pose.M(1, 2)<<std::endl;
    std::cout<<end_effector_pose.M(2, 0)<<end_effector_pose.M(2, 1)<<end_effector_pose.M(2, 2)<<std::endl;

    //fk_solver_2.JntToCart(nominal_2, end_effector_pose_2);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to grab");

  std::thread th1(Get_pos); // for camera

  th1.detach();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
