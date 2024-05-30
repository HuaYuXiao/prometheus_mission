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
#include <easondrone_msgs/ControlCommand.h>
#include <easondrone_msgs/DroneState.h>
#include <easondrone_msgs/DetectionInfo.h>
#include <easondrone_msgs/PositionReference.h>
#include <easondrone_msgs/AttitudeReference.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>

using namespace std;

# define NODE_NAME "planning_mission"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
easondrone_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
easondrone_msgs::DroneState _DroneState;                                   //无人机状态量
ros::Publisher command_pub;

geometry_msgs::PoseStamped final_goal;                              // goal
easondrone_msgs::PositionReference planner_cmd;          // fast planner cmd

float thresh_no_replan_ = 0.2;
bool control_yaw_flag = false;
bool flag_get_cmd = false;
bool flag_get_goal = false;
float desired_yaw = 0.0;  //[rad]
float distance_to_goal = 0.0;
double last_angle = 0.0;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void planner();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void planner_cmd_cb(const easondrone_msgs::PositionReference::ConstPtr& msg){
    flag_get_cmd = true;

    planner_cmd = *msg;
}

void quadrotor_planner_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    flag_get_cmd = true;

    planner_cmd.Move_mode = easondrone_msgs::PositionReference::POS_VEL_ACC;
    planner_cmd.Move_frame          = easondrone_msgs::PositionReference::ENU_FRAME;

    planner_cmd.position_ref[0] = msg->position.x;
    planner_cmd.position_ref[1] = msg->position.y;
    planner_cmd.position_ref[2] = msg->position.z;

    planner_cmd.velocity_ref[0] = msg->velocity.x;
    planner_cmd.velocity_ref[1] = msg->velocity.y;
    planner_cmd.velocity_ref[2] = msg->velocity.z;

    planner_cmd.acceleration_ref[0] = msg->acceleration.x;
    planner_cmd.acceleration_ref[1] = msg->acceleration.y;
    planner_cmd.acceleration_ref[2] = msg->acceleration.z;

    planner_cmd.yaw_ref             = atan2(msg->velocity.y, msg->velocity.x); // msg->yaw
    planner_cmd.yaw_rate_ref        = msg->yaw_dot;
}

void drone_state_cb(const easondrone_msgs::DroneState::ConstPtr& msg){
    _DroneState = *msg;
}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    flag_get_goal = true;

    final_goal = *msg;
    final_goal.pose.position.z = _DroneState.position[2];
    last_angle = 2 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);

    cout << "[mission] Get a new goal: " <<
    final_goal.pose.position.x << " " <<
    final_goal.pose.position.y << " " <<
    final_goal.pose.position.z << endl;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "planning_mission");
    ros::NodeHandle nh("~");

    nh.param<float>("planning_mission/thresh_no_replan", thresh_no_replan_, 0.2);
    nh.param<bool>("planning_mission/control_yaw_flag", control_yaw_flag, false);
    
    //【订阅】无人机当前状态
    ros::Subscriber drone_state_sub = nh.subscribe<easondrone_msgs::DroneState>("/easondrone/drone_state", 10, drone_state_cb);
    //【订阅】来自planning的指令
    ros::Subscriber planner_sub = nh.subscribe<easondrone_msgs::PositionReference>("/easondrone/position_cmd", 10, planner_cmd_cb);
    //【订阅】来自EGO-Planner的指令
    ros::Subscriber quadrotor_planner_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/easondrone/quadrotor_position_cmd", 10, quadrotor_planner_cmd_cb);
    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/easondrone/planning/goal", 1, goal_cb);
    
    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    command_pub = nh.advertise<easondrone_msgs::ControlCommand>("/easondrone/control_command", 10);

    // 设置cout的精度为小数点后两位
    std::cout << std::fixed << std::setprecision(2);

    cout << "[mission] easondrone_mission initialized!" << endl;

    while (ros::ok()){
        //回调
        ros::spinOnce();

        if(flag_get_goal){
            distance_to_goal = sqrt(pow(_DroneState.position[0] - final_goal.pose.position.x, 2) +
                                    pow(_DroneState.position[1] - final_goal.pose.position.y, 2) +
                                    pow(_DroneState.position[2] - final_goal.pose.position.z, 2));

            cout << "[mission] Distance to [" << final_goal.pose.position.x << " " << final_goal.pose.position.y << " " << final_goal.pose.position.z << "] is " << distance_to_goal << endl;

            // priority of mission is higher than planner
            if (distance_to_goal <= control_yaw_flag){
                // 抵达目标附近，则停止速度控制，改为位置控制
                planner_cmd.header.stamp = ros::Time::now();
                Command_Now.Mode = easondrone_msgs::ControlCommand::Move;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Reference_State.Move_mode = easondrone_msgs::PositionReference::XYZ_POS;
                Command_Now.Reference_State.Move_frame = easondrone_msgs::PositionReference::ENU_FRAME;

                Command_Now.Reference_State.position_ref[0] = final_goal.pose.position.x;
                Command_Now.Reference_State.position_ref[1] = final_goal.pose.position.y;
                Command_Now.Reference_State.position_ref[2] = final_goal.pose.position.z;

                Command_Now.Reference_State.velocity_ref[0] = 0.0;
                Command_Now.Reference_State.velocity_ref[1] = 0.0;
                Command_Now.Reference_State.velocity_ref[2] = 0.0;

                Command_Now.Reference_State.acceleration_ref[0] = 0.0;
                Command_Now.Reference_State.acceleration_ref[1] = 0.0;
                Command_Now.Reference_State.acceleration_ref[2] = 0.0;

                Command_Now.Reference_State.yaw_ref = last_angle;

                command_pub.publish(Command_Now);

                cout << "[mission] close to goal, change to POS control" << endl;

                flag_get_goal = false;

                ros::Duration(0.02).sleep();
            }
            else{
                if(flag_get_cmd){
                    planner();

                    ros::Duration(0.02).sleep();
                }
                else{
                    // TODO: two types of situation:
                    //  1. planner failed, directly publish move cmd
                    //  2. planner not initialized yet
//                    planner_cmd.header.stamp = ros::Time::now();
//                    Command_Now.Mode = easondrone_msgs::ControlCommand::Hold;
//                    Command_Now.Command_ID = Command_Now.Command_ID + 1;
//                    Command_Now.source = NODE_NAME;
//                    Command_Now.Reference_State.Move_mode = easondrone_msgs::PositionReference::XYZ_POS;
//                    Command_Now.Reference_State.Move_frame = easondrone_msgs::PositionReference::ENU_FRAME;
//                    Command_Now.Reference_State.position_ref[0] = final_goal.pose.position.x;
//                    Command_Now.Reference_State.position_ref[1] = final_goal.pose.position.y;
//                    Command_Now.Reference_State.position_ref[2] = final_goal.pose.position.z;
//                    Command_Now.Reference_State.yaw_ref = last_angle;
//
//                    command_pub.publish(Command_Now);

                    cout << "[mission] planner not initialized yet" << endl;

                    ros::Duration(0.02).sleep();
                }
            }
        }
        else{
            ros::Duration(0.02).sleep();
        }
    }

    return 0;
}

void planner(){
    if(control_yaw_flag){
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
    }

    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = easondrone_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    // easondrone_msgs::PositionReference is part of  easondrone_msgs::ControlCommand.Reference_State
    Command_Now.Reference_State = planner_cmd;

    command_pub.publish(Command_Now);
}
