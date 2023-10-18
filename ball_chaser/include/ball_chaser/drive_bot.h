#include "rclcpp/rclcpp.hpp"
#include "ball_chaser_interface/srv/drive_to_target.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace ball_chaser
{
    class DriveBot : public rclcpp::Node
    {
    public:
        DriveBot();

    private:
        rclcpp::Service<ball_chaser_interface::srv::DriveToTarget>::SharedPtr m_driveToTargerServer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPub;
        void DriveToTargetCallback(const ball_chaser_interface::srv::DriveToTarget::Request::SharedPtr &request, const ball_chaser_interface::srv::DriveToTarget::Response::SharedPtr &response);
    };
}