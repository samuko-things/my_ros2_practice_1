// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TurtleMoveToGoal : public rclcpp::Node
{
public:
  TurtleMoveToGoal() : Node("turtle_move_to_goal"), count_(0)
  {
    pose_subscriber_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    auto pose_subscriber_opt = rclcpp::SubscriptionOptions();
    pose_subscriber_opt.callback_group = pose_subscriber_callback_group_;
    pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&TurtleMoveToGoal::update_pose, this, _1), pose_subscriber_opt);

    // timer_ = this->create_wall_timer(
    //     1000ms, std::bind(&TurtleMoveToGoal::timer_callback, this), timer_callback_group_);

    RCLCPP_INFO(this->get_logger(), "turtle_move_to_goal node has started");
    execute_task();
  }

  void execute_task() {
    double x_goal_pose, y_goal_pose;
    turtlesim::msg::Pose goal_pose;

    std::cout << "goal x position: ";
    std::cin >> x_goal_pose;

    std::cout << "goal y position: ";
    std::cin >> y_goal_pose;

    goal_pose.x = x_goal_pose;
    goal_pose.y = y_goal_pose;

    move_to_goal(goal_pose, 0.1);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::CallbackGroup::SharedPtr pose_subscriber_callback_group_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
  turtlesim::msg::Pose pose;
  size_t count_;

  void update_pose(const turtlesim::msg::Pose &pose_data)
  {
    pose.x = pose_data.x;
    pose.y = pose_data.y;
    pose.theta = pose_data.theta;

    RCLCPP_INFO(this->get_logger(), "'%f', '%f', '%f'", (double)pose.x, (double)pose.y, (double)pose.theta);
  }

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "cool messages");
  }

  double compute_euclidean_dist(turtlesim::msg::Pose goal_pose)
  {
    return sqrt((pow((goal_pose.x - pose.x), 2) + pow((goal_pose.y - pose.y), 2)));
  }

  double compute_steer_angle(turtlesim::msg::Pose goal_pose)
  {
    return atan2(goal_pose.y - pose.y, goal_pose.x - pose.x);
  }

  double command_linear_vel(turtlesim::msg::Pose goal_pose, double constant = 0.5)
  {
    return constant * compute_euclidean_dist(goal_pose);
  }

  double command_angular_velocity(turtlesim::msg::Pose goal_pose, double constant = 1.5)
  {
    return constant * compute_steer_angle(goal_pose);
  }

  void move_to_goal(turtlesim::msg::Pose goal_pose, double distance_tolerance)
  {
    geometry_msgs::msg::Twist vel_msg;
    rclcpp::Rate loop_rate(100000);

    double compute_distance = compute_euclidean_dist(goal_pose);
    while (compute_distance >= distance_tolerance)
    {
      // Porportional controller.
      // https://en.wikipedia.org/wiki/Proportional_control

      // Linear velocity in the x-axis.double x_goal_pose, y_goal_pose;
      vel_msg.linear.x = command_linear_vel(goal_pose);
      vel_msg.linear.y = 0;
      vel_msg.linear.z = 0;

      // Angular velocity in the z-axis.
      vel_msg.angular.x = 0;
      vel_msg.angular.y = 0;
      vel_msg.angular.z = command_angular_velocity(goal_pose);

      // Publishing our vel_msg
      velocity_publisher_->publish(vel_msg);

      // Publish at the desired rate.
      loop_rate.sleep();
      compute_distance = compute_euclidean_dist(goal_pose);
    }
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher_->publish(vel_msg);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::executors::MultiThreadedExecutor executor;

  // auto node = std::make_shared<TurtleMoveToGoal>();
  // executor.add_node(node);
  // executor.spin();

  rclcpp::spin(std::make_shared<TurtleMoveToGoal>());
  rclcpp::shutdown();
  return 0;
}
