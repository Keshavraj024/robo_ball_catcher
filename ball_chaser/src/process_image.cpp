#include "ball_chaser/process_image.h"

using std::placeholders::_1;

namespace ball_chaser
{
    ProcessImage::ProcessImage() : Node("process_image")
    {
        m_cameraSubscriber = this->create_subscription<sensor_msgs::msg::Image>("/ball_chaser/camera/image_raw", 0, std::bind(&ProcessImage::processImageCallback, this, _1));
    }

    void ProcessImage::processImageCallback(const sensor_msgs::msg::Image &cameraImage)
    {
        bool is_ball_found{false};
        for (size_t idx = 0; idx < cameraImage.height * cameraImage.step; idx += 3)
        {
            if ((cameraImage.data.at(idx) > 100) && (cameraImage.data.at(idx + 1) == 0) && (cameraImage.data.at(idx + 2) == 0))
            {
                const auto idx_column = idx % cameraImage.step;

                if (idx_column < cameraImage.step / 3)
                    driveToTaget(0.5, -1);
                else if (idx_column < ((cameraImage.step / 3))* 2)
                    driveToTaget(0.5, 0);
                else
                    driveToTaget(0.5, 1);
                is_ball_found = true;
                break;
            }

        }
        if (!is_ball_found)
        {
            driveToTaget(0, 0);
        }
    }

    void ProcessImage::driveToTaget(const float &linearVel, const float &angularVel)
    {
        auto client = this->create_client<ball_chaser_interface::srv::DriveToTarget>("ball_chaser/command_robot");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for spwan service server to be up!!");
        }

        auto request = std::make_shared<ball_chaser_interface::srv::DriveToTarget::Request>();
        request->linear_x = linearVel;
        request->angular_z = angularVel;
        auto future = client->async_send_request(request);

        
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ball_chaser::ProcessImage>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}