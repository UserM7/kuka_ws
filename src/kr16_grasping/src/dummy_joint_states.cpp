
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("dummy_joint_states");

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  rclcpp::WallRate loop_rate(50);

  sensor_msgs::msg::JointState msg;
  
    msg.name.push_back("joint_a1");
    msg.name.push_back("joint_a2");
    msg.name.push_back("joint_a3");
    msg.name.push_back("joint_a4");
    msg.name.push_back("joint_a5");
    msg.name.push_back("joint_a6");
    

    msg.position.push_back(-1.5810079050729513);
    msg.position.push_back(0.8167376905451679);
    msg.position.push_back(-0.004375380628040148);
    msg.position.push_back(-1.5857193233230227);
    msg.position.push_back(-7.76584250674877e-06);
    msg.position.push_back(0.7069321699035735);
    

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  auto counter = 0.0;
  auto joint_value = 0.0;
  while (rclcpp::ok()) {
    counter += 0.1;
    joint_value = std::sin(counter);

    for (size_t i = 0; i < msg.name.size(); ++i) {
      msg.position[i] = joint_value;
    }

    msg.header.stamp = clock->now();

    joint_state_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
