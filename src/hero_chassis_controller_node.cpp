#include "hero_chassis_controller/hero_chassis_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <controller_manager/controller_manager.h>
#include <thread>
#include <chrono>

namespace hero_chassis_controller {

class HeroChassisControllerNode {
public:
    HeroChassisControllerNode() : root_nh_(ros::NodeHandle{}), controller_nh_(ros::NodeHandle("controller")) {
        // 初始化 ControllerManager
        controller_manager_ = std::make_unique<controller_manager::ControllerManager>(nullptr, root_nh_);

        // 加载控制器
        if (!controller_manager_->loadController("hero_chassis_controller")) {
            ROS_ERROR("Failed to load hero_chassis_controller.");
            return;
        }

        // 初始化 HeroChassisController
        hero_chassis_controller_ = std::make_unique<HeroChassisController>();
        if (!hero_chassis_controller_->init(nullptr, root_nh_, controller_nh_)) {
            ROS_ERROR("Failed to initialize HeroChassisController.");
            return;
        }

        // 初始化 cmd_vel 订阅器
        cmd_vel_sub_ = root_nh_.subscribe("/cmd_vel", 10, &HeroChassisControllerNode::cmdVelCallback, this);

        // 初始化 cmd_vel 发布器
        cmd_vel_pub_ = root_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 调用 HeroChassisController 的 cmdVelCallback 方法
        hero_chassis_controller_->cmdVelCallback(msg);
    }

    void publishCmdVel() {
        // 创建一个Twist消息
        geometry_msgs::Twist twist;
        twist.linear.x = 0.5;  // 线速度，根据需要调整
        twist.angular.z = 0.0;  // 角速度，根据需要调整

        // 循环发布消息
        ros::Rate rate(10);  // 设置循环频率为 10Hz
        while (ros::ok()) {
            cmd_vel_pub_.publish(twist);
            rate.sleep();
        }
    }

    void run() {
        // 进入 ROS 事件循环
        std::thread vel_thread(&HeroChassisControllerNode::publishCmdVel, this);  // 创建线程发布cmd_vel消息
        vel_thread.detach();  // 分离线程

        ros::Rate rate(10);  // 设置循环频率为 10Hz
        while (ros::ok()) {
            controller_manager_->update(ros::Time::now(), ros::Duration(1.0 / 10));
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle root_nh_;
    ros::NodeHandle controller_nh_;
    std::unique_ptr<controller_manager::ControllerManager> controller_manager_;
    std::unique_ptr<HeroChassisController> hero_chassis_controller_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher cmd_vel_pub_;
};

}  // namespace hero_chassis_controller

int main(int argc, char** argv) {
    ros::init(argc, argv, "hero_chassis_controller_node");

    hero_chassis_controller::HeroChassisControllerNode node;
    node.run();

    return 0;
}
