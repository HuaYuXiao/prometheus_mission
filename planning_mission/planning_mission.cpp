//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
//topic 头文件
#include <mission_utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "message_utils.h"

using namespace std;

#define MIN_DIS 0.1
# define NODE_NAME "planning_mission"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
prometheus_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;

geometry_msgs::PoseStamped final_goal;                              // goal
prometheus_msgs::PositionReference planner_cmd;          // fast planner cmd

bool sim_mode;
bool control_yaw_flag;
bool flag_get_cmd = false;
bool flag_get_goal = false;
float desired_yaw = 0;  //[rad]
float distance_to_goal = 0;
double last_angle;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void planner();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void planner_cmd_cb(const prometheus_msgs::PositionReference::ConstPtr& msg){
    flag_get_cmd = true;

    planner_cmd = *msg;
}

void quadrotor_planner_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    flag_get_cmd = true;

    planner_cmd.header.stamp = ros::Time::now();
    planner_cmd.header.frame_id = "map";

    planner_cmd.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;  //TRAJECTORY
    planner_cmd.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME; //ENU_FRAME
    planner_cmd.time_from_start = 0.0;

    planner_cmd.position_ref[0] = msg->position.x;
    planner_cmd.position_ref[1] = msg->position.y;
    planner_cmd.position_ref[2] = msg->position.z;

    planner_cmd.velocity_ref[0] = msg->velocity.x;
    planner_cmd.velocity_ref[1] = msg->velocity.y;
    planner_cmd.velocity_ref[2] = msg->velocity.z;

    planner_cmd.acceleration_ref[0] = msg->acceleration.x;
    planner_cmd.acceleration_ref[1] = msg->acceleration.y;
    planner_cmd.acceleration_ref[2] = msg->acceleration.z;

    planner_cmd.yaw_ref = msg->yaw;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg){
    _DroneState = *msg;

    distance_to_goal = sqrt(pow(_DroneState.position[0] - final_goal.pose.position.x, 2) +
                            pow(_DroneState.position[1] - final_goal.pose.position.y, 2) +
                            pow(_DroneState.position[2] - final_goal.pose.position.z, 2));
}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    flag_get_goal = true;

    final_goal = *msg;
    final_goal.pose.position.z = _DroneState.position[2];

    cout << "[mission] Get a new goal: " <<
    final_goal.pose.position.x << ", " <<
    final_goal.pose.position.y << ", " <<
    final_goal.pose.position.z << endl;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "planning_mission");
    ros::NodeHandle nh("~");

    nh.param<bool>("planning_mission/control_yaw_flag", control_yaw_flag, true);
    // 是否为仿真模式
    nh.param<bool>("planning_mission/sim_mode", sim_mode, true);
    
    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);
    //【订阅】来自planning的指令
    ros::Subscriber planner_sub = nh.subscribe<prometheus_msgs::PositionReference>("/prometheus/position_cmd", 10, planner_cmd_cb);
    //【订阅】来自EGO-Planner的指令
    ros::Subscriber quadrotor_planner_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/prometheus/quadrotor_position_cmd", 10, quadrotor_planner_cmd_cb);
    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 1, goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    cout << "[mission] prometheus_mission initialized!" << endl;

    while (ros::ok()){
        static int exec_num=0;
        exec_num++;

        //回调
        ros::spinOnce();

        if(flag_get_cmd){
                if (distance_to_goal < MIN_DIS){
                    // 抵达目标附近，则停止速度控制，改为位置控制
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
                    Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                    Command_Now.Reference_State.position_ref[0] = final_goal.pose.position.x;
                    Command_Now.Reference_State.position_ref[1] = final_goal.pose.position.y;
                    Command_Now.Reference_State.position_ref[2] = final_goal.pose.position.z;
                    Command_Now.Reference_State.yaw_ref = desired_yaw;
                    command_pub.publish(Command_Now);

                    if (exec_num == 10) {
                        cout << "[mission] Goal arrived, waiting for a new goal" << endl;
                        exec_num = 0;
                    }

                    flag_get_goal = true;
                    while (!flag_get_goal) {
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                    }
                }else{
                    planner();
                    ros::Duration(0.05).sleep();
                }
        }
    }

    return 0;
}

void planner(){
    if (control_yaw_flag){
        // 根据速度大小决定是否更新期望偏航角， 更新采用平滑滤波的方式，系数可调
        // TODO: fastplanner航向策略仍然可以进一步优化
        if(sqrt(planner_cmd.velocity_ref[1]* planner_cmd.velocity_ref[1] + planner_cmd.velocity_ref[0]* planner_cmd.velocity_ref[0]) > 0.05){
            float next_desired_yaw_vel = atan2(planner_cmd.velocity_ref[1] ,planner_cmd.velocity_ref[0]);
            float next_desired_yaw_pos = atan2(planner_cmd.position_ref[1] - _DroneState.position[1], planner_cmd.position_ref[0] - _DroneState.position[0]);

            if(next_desired_yaw_pos > 0.8){
                next_desired_yaw_pos = 0.8;
            }if(next_desired_yaw_pos < -0.8){
                next_desired_yaw_pos = -0.8;
            }

            desired_yaw = (0.92 * desired_yaw + 0.04 * next_desired_yaw_pos + 0.04 * next_desired_yaw_vel);
        }
    }else{
        desired_yaw = 0.0;
    }

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    // TODO: Command_ID may exceed range of int
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State = planner_cmd;
    Command_Now.Reference_State.yaw_ref = planner_cmd.yaw_ref;
    command_pub.publish(Command_Now);
}
