#include "hero_chassis_controller/hero_chassis_controller.h"
#include <cmath>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {       
                                             
    if (!effort_joint_interface) {  
 
    ROS_ERROR("Effort joint interface is null.");
    return false;
}
     effort_joint_interface_ = effort_joint_interface;
    // Initialize joint handles
    front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

    ROS_INFO("Joint handles initialized: %s, %s, %s, %s", 
             front_left_joint_.getName().c_str(), front_right_joint_.getName().c_str(),
             back_left_joint_.getName().c_str(), back_right_joint_.getName().c_str());
    
    // Load wheel track and base from parameter server
    controller_nh.param("/wheel/track", wheel_track_, 0.4);
    controller_nh.param("/wheel/base", wheel_base_, 0.4);    
    ROS_INFO("wheel_track: %f, wheel_base: %f", wheel_track_, wheel_base_ ); 
    // Load PID gains from parameter server
    if (!updatePIDGains(controller_nh)) {
        ROS_ERROR("Failed to load PID gains from parameter server.");
        return false;
    }
   
    // Initialize odometry publisher
    odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 50);
    if (!odom_pub_) {
        ROS_ERROR("Failed to advertise odometry topic.");
        return false;
    }
     tf_broadcaster_Initialized = true;
     
     cmd_vel_sub_ = root_nh.subscribe("/cmd_vel" ,10, &HeroChassisController::cmdVelCallback, this);
    
    return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
    ros::spinOnce();
    
    double v_x = cmd_vel_.linear.x;
    double v_y = cmd_vel_.linear.y;
    double v_yaw = cmd_vel_.angular.z;
    
    
    double wheel_speeds[4];
    calculateWheelSpeeds(v_x, v_y, v_yaw, wheel_speeds);
    
    for (size_t i = 0; i < 4; ++i) {
        double error = wheel_speeds[i] - current_wheel_speeds_[i];
        double command = pid_controllers_[i].computeCommand(error, period);
        
        hardware_interface::JointHandle *joint_handles[] = {
            &front_left_joint_, &front_right_joint_, &back_left_joint_, &back_right_joint_
        };
        if (i >= sizeof(joint_handles) / sizeof(joint_handles[0])) {
            ROS_ERROR("Index out of range for joint handles.");
            return;
        }
        joint_handles[i]->setCommand(command);
        ROS_INFO("joint %ld, command %f", i , command); 																																																																				
    }
// 更新当前轮子速度
    current_wheel_speeds_[0] = wheel_speeds[0];
    current_wheel_speeds_[1] = wheel_speeds[1];
    current_wheel_speeds_[2] = wheel_speeds[2];
    current_wheel_speeds_[3] = wheel_speeds[3];
    
    updateOdometry(v_x, v_y, v_yaw);
    publishOdometry();
    publishTF();
}

bool HeroChassisController::updatePIDGains(ros::NodeHandle &controller_nh) {
    double p, i, d, i_max, i_min;
    std::string joint_names[4] = {"left_front_wheel_joint", "right_front_wheel_joint", "left_back_wheel_joint", "right_back_wheel_joint"};

    for (int a = 0; a < 4; ++a) {
        std::string param_name_p = "/pid_gains/" + joint_names[a] + "/p";
        std::string param_name_i = "/pid_gains/" + joint_names[a] + "/i";
        std::string param_name_d = "/pid_gains/" + joint_names[a] + "/d";
        std::string param_name_i_max = "/pid_gains/" + joint_names[a] + "/i_max";
        std::string param_name_i_min = "/pid_gains/" + joint_names[a] + "/i_min";

        // Ensure all parameters have default values
        if (!controller_nh.getParam(param_name_p, p) || !controller_nh.getParam(param_name_i, i) ||
            !controller_nh.getParam(param_name_d, d) || !controller_nh.getParam(param_name_i_max, i_max) ||
            !controller_nh.getParam(param_name_i_min, i_min)) {
            ROS_ERROR_STREAM("Failed to get parameter for joint " << joint_names[a]);
            return false;
        }

        // Check if any of the gains are not set (i.e., they are NaN) and handle accordingly
        if (std::isnan(p) || std::isnan(i) || std::isnan(d) || std::isnan(i_max) || std::isnan(i_min)) {
            ROS_ERROR_STREAM("PID gains for wheel " << a+1 << " are not set correctly.");
            return false;
        }

        // Set the gains for the corresponding PID controller
        pid_controllers_[a].setGains(p, i, d, i_max, i_min);
        
        // Print the PID gains using ROS_INFO
        ROS_INFO_STREAM("PID gains for joint " << joint_names[a] << ": "
                        << "p: " << p << ", i: " << i << ", d: " << d 
                        << ", i_max: " << i_max << ", i_min: " << i_min);
    }
    return true;
}

void HeroChassisController::calculateWheelSpeeds(double v_x, double v_y, double v_yaw, double (&wheel_speeds)[4]) {
    // 确保轮距和轴距参数已加载，否则使用默认值
    if (wheel_track_ <= 0 || wheel_base_ <= 0) {
        ROS_ERROR("Wheel track or base is not set correctly. Using default values.");
        wheel_track_ = 0.4;  // 默认轮距
        wheel_base_ = 0.4;   // 默认轴距
    }

    // 计算每个轮子的期望速度
    // 轮子半径
    double wheel_radius = 0.07625;  // 从mecanum_wheel.urdf.xacro中获取

    // 计算每个轮子的线速度
    wheel_speeds[0] = (v_x - v_yaw * (wheel_track_ / 2) - v_y * (wheel_base_ / 2)) / wheel_radius; // front_left
    wheel_speeds[1] = (v_x + v_yaw * (wheel_track_ / 2) + v_y * (wheel_base_ / 2)) / wheel_radius; // front_right
    wheel_speeds[2] = (v_x - v_yaw * (wheel_track_ / 2) + v_y * (wheel_base_ / 2)) / wheel_radius; // back_left
    wheel_speeds[3] = (v_x + v_yaw * (wheel_track_ / 2) - v_y * (wheel_base_ / 2)) / wheel_radius; // back_right
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received velocity command: linear x: %f, y: %f, z: %f, angular x: %f, y: %f, z: %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    cmd_vel_ = *msg;
}

void HeroChassisController::updateOdometry(double v_x, double v_y, double v_yaw) {
    if (first_update_) {
        first_update_ = false;
        odom_msg_.pose.pose.position.x = 0;
        odom_msg_.pose.pose.position.y = 0;
        odom_msg_.pose.pose.position.z = 0;  // Correct access method
        odom_orientation_ = Eigen::Quaterniond(1, 0, 0, 0);  // Identity quaternion
        last_update_time_ = ros::Time::now();
    }

    double dt = (ros::Time::now() - last_update_time_).toSec();
    last_update_time_ = ros::Time::now();

    // Calculate the odometry pose using Eigen
    Eigen::Vector3d velocity(v_x, v_y, v_yaw);
    Eigen::Vector3d delta_position = velocity * dt;

    // Update the position
    odom_msg_.pose.pose.position.x += delta_position[0];
    odom_msg_.pose.pose.position.y += delta_position[1];
    odom_msg_.pose.pose.position.z += delta_position[2];  // Using correct member access

    // Update the orientation
    Eigen::Vector3d angular_velocity(0, 0, v_yaw);
    Eigen::AngleAxisd delta_angle(angular_velocity.norm() * dt, angular_velocity.normalized());
    Eigen::Quaterniond delta_orientation = Eigen::Quaterniond(delta_angle);  // Correct conversion method
    odom_orientation_ = delta_orientation * odom_orientation_;
    odom_orientation_.normalize();  // Normalize to keep the quaternion valid

    // Convert Eigen quaternion to ROS quaternion
    odom_msg_.pose.pose.orientation.w = odom_orientation_.w();
    odom_msg_.pose.pose.orientation.x = odom_orientation_.x();
    odom_msg_.pose.pose.orientation.y = odom_orientation_.y();
    odom_msg_.pose.pose.orientation.z = odom_orientation_.z();
}

void HeroChassisController::publishOdometry() {
    if (!odom_pub_) {
        ROS_ERROR("Odometry publisher not initialized.");
        return;
    }
    odom_pub_.publish(odom_msg_);
}

void HeroChassisController::publishTF() {
    if (!tf_broadcaster_Initialized) {
        ROS_ERROR("TF broadcaster not initialized.");
        return;
    }
    tf::Transform transform;
    tf::poseMsgToTF(odom_msg_.pose.pose, transform);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, odom_msg_.header.stamp, "odom", "base_link"));
}

 }  // namespace hero_chassis_controller
