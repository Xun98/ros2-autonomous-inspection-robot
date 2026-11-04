#include <chrono>
#include <memory>
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


class InitPosePublisher : public rclcpp::Node
{
public:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  InitPosePublisher()
  : Node("initpose_publisher")
  {
    publisher_ = this->create_publisher<PoseMsg>("/initialpose", 10);

    // 每隔 500ms 检查一次 AMCL 是否已经订阅
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            if (publisher_->get_subscription_count() == 0) {
                return;
            }
            pub_initpose();
            timer_->cancel();
            rclcpp::shutdown();
        });
  }

  void pub_initpose()
  {
            PoseMsg message;
        message.header.frame_id = "map";
        message.header.stamp = this->get_clock()->now();
        auto &position = message.pose.pose.position;
        auto &orientation = message.pose.pose.orientation;
        position.x = .0;
        position.y = .0;
        position.z = .0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        orientation.x = q.getX();
        orientation.y = q.getY();
        orientation.z = q.getZ();
        orientation.w = q.getW();

        message.pose.covariance[0] = 0.25;  // x
        message.pose.covariance[7] = 0.25;  // y
        message.pose.covariance[35] = 0.0685; // yaw

        RCLCPP_INFO(get_logger(), "Pub init pose: x = %.2f, y = %.2f, z = %.2f", position.x, position.y, position.z);
        this->publisher_->publish(message);
  }

  
private:

  rclcpp::Publisher<PoseMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}