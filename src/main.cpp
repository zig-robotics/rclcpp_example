#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("example_node", rclcpp::NodeOptions().use_intra_process_comms(true));
  const auto publisher = rclcpp::create_publisher<builtin_interfaces::msg::Time>(node, "test", 1);
  const auto subscription = rclcpp::create_subscription<builtin_interfaces::msg::Time>(
    node,
    "test",
    1,
    [node](builtin_interfaces::msg::Time::ConstSharedPtr msg) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Time: " << msg->sec);
    });

  const auto timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration::from_seconds(1.0),
    [publisher, node](){
      const auto time_msg = builtin_interfaces::msg::Time(node->now());
      publisher->publish(time_msg);
    }
  );

  auto executor = rclcpp::experimental::executors::EventsExecutor();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
