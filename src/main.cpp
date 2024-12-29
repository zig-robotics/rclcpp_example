#include "rclcpp/rclcpp.hpp"
#include "zigros_example_interface/msg/example.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("example_node", rclcpp::NodeOptions().use_intra_process_comms(true));
  const auto publisher = rclcpp::create_publisher<zigros_example_interface::msg::Example>(node, "test", 1);
  const auto subscription = rclcpp::create_subscription<zigros_example_interface::msg::Example>(
    node,
    "test",
    1,
    [node](zigros_example_interface::msg::Example::ConstSharedPtr msg) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Time: " << msg->time.sec);
    });

  const auto timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration::from_seconds(1.0),
    [publisher, node](){
      auto msg = zigros_example_interface::msg::Example();
      msg.time = node->now();
      publisher->publish(msg);
    }
  );

  auto executor = rclcpp::experimental::executors::EventsExecutor();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
