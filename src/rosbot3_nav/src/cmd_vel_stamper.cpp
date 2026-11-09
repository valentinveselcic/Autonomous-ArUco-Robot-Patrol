#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelStamper : public rclcpp::Node
{
public:
  CmdVelStamper()
  : Node("cmd_vel_stamper")
  {
    // Nav2 will publish to /cmd_vel_nav as plain Twist
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav",
      10,
      std::bind(&CmdVelStamper::cmdVelCallback, this, std::placeholders::_1));

    // Drive controller expects TwistStamped on /cmd_vel
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel",
      10);

    RCLCPP_INFO(get_logger(),
      "cmd_vel_stamper started: /cmd_vel_nav (Twist) -> /cmd_vel (TwistStamped)");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = this->now();
    stamped.header.frame_id = "base_link";  // reasonable default
    stamped.twist = *msg;
    pub_->publish(stamped);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelStamper>());
  rclcpp::shutdown();
  return 0;
}