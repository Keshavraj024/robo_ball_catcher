#include "ball_chaser/drive_bot.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ball_chaser
{
    DriveBot::DriveBot() : Node("drive_bot")
    {
        m_driveToTargerServer = this->create_service<ball_chaser_interface::srv::DriveToTarget>("ball_chaser/command_robot",
                                                                                                std::bind(&DriveBot::DriveToTargetCallback, this, _1, _2));
        m_cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    void DriveBot::DriveToTargetCallback(const ball_chaser_interface::srv::DriveToTarget::Request::SharedPtr &request, const ball_chaser_interface::srv::DriveToTarget::Response::SharedPtr &response)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = request->linear_x;
        msg.angular.z = request->angular_z;

        m_cmdVelPub->publish(msg);
        response->msg_feedback = "Command Velocity Published";
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ball_chaser::DriveBot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}