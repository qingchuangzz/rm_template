#include "hero_chassis_controller/hero_chassis_controller.h"
#include <ros/ros.h>

namespace hero_chassis_controller {

class HeroChassisControllerNode {
public:
    HeroChassisControllerNode(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
        : root_nh_(root_nh), controller_nh_(controller_nh), hero_chassis_controller_(std::make_unique<HeroChassisController>()) {
        // 初始化 cmd_vel 订阅器
        cmd_vel_sub_ = root_nh_.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, hero_chassis_controller_.get());
    }
   
private:
    ros::NodeHandle root_nh_;
    ros::NodeHandle controller_nh_;
    std::unique_ptr<HeroChassisController> hero_chassis_controller_;
    ros::Subscriber cmd_vel_sub_;
};

}  // namespace hero_chassis_controller

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "hero_chassis_controller_node");

    // 创建节点句柄
    ros::NodeHandle root_nh;
    ros::NodeHandle controller_nh;

    // 创建 HeroChassisControllerNode 实例
    hero_chassis_controller::HeroChassisControllerNode node(root_nh, controller_nh);

  

    // 进入 ROS 事件循环
    ros::Rate rate(20);  // 设置循环频率为 10Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
