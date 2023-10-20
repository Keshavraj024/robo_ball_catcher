#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ball_chaser_interface/srv/drive_to_target.hpp"

namespace ball_chaser
{
    class ProcessImage : public rclcpp::Node
    {
    public:
        ProcessImage();

    private:
        void processImageCallback(const sensor_msgs::msg::Image &cameraImage);
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_cameraSubscriber;
        
        void driveToTaget(const float &linearVel, const float &angularVel);
        double m_linearVelocityX;
        double m_angularVelocityZ;
    };
}