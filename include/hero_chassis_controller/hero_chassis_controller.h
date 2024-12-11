#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  
   void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
   geometry_msgs::Twist cmd_vel_;
   double wheel_track_;  // 轮距
  double wheel_base_;   // 轴距
 private:
  bool updatePIDGains(ros::NodeHandle &controller_nh);
  void calculateWheelSpeeds(double v_x, double v_y, double v_yaw, double (&wheel_speeds)[4]);
  void updateOdometry(double v_x, double v_y, double v_yaw);
  void publishOdometry();
  void publishTF();
  bool tf_broadcaster_Initialized;
  tf::TransformBroadcaster tf_broadcaster_;
  control_toolbox::Pid pid_controllers_[4];
  
  double wheel_offset_z_;  // 轮子Z轴偏移
  double current_wheel_speeds_[4] = {0, 0, 0, 0};
  ros::Publisher odom_pub_;
  nav_msgs::Odometry odom_msg_;
  Eigen::Quaterniond odom_orientation_;  // 使用Eigen存储四元数
  bool first_update_ = true;
  ros::Time last_update_time_;
  ros::Subscriber cmd_vel_sub_;
    // 存储接收到的cmd_vel
  hardware_interface::EffortJointInterface* effort_joint_interface_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
