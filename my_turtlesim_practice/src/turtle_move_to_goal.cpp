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
#include <ctime>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/srv/turtle_pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}
double rad_2_deg(double ang_in_rad)
{
  double ang_in_deg = ang_in_rad * 180.0 / M_PI;
  return ang_in_deg;
}
double deg_to_rad(double ang_in_deg)
{
  double ang_in_rad = ang_in_deg * M_PI / 180.0;
  return ang_in_rad;
}


class TurtleMoveToGoalNode : public rclcpp::Node
{
public:
  TurtleMoveToGoalNode() : Node("turtle_move_to_goal"), count_(0)
  {
    turtle_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    turtle_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&TurtleMoveToGoalNode::update_pose, this, _1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleMoveToGoalNode::move_to_goal, this));

    add_goal_server_ = this->create_service<my_robot_interfaces::srv::TurtlePose>(
        "add_new_goal_pose",
        std::bind(&TurtleMoveToGoalNode::callback_add_new_goal_pose, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "turtle_move_to_goal node has started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_velocity_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_subscriber_;

  size_t count_;
  bool turtlesim_up_ = false;
  turtlesim::msg::Pose pose;
  turtlesim::msg::Pose goal_pose;
  std::vector<std::thread> threads_;

  double kp_dist = 1.25;
  double kp_angle = 2.5;

  std::vector<double> pose_x_list;
  std::vector<double> pose_y_list;
  std::vector<std::string> turtle_name_list;
  rclcpp::Service<my_robot_interfaces::srv::TurtlePose>::SharedPtr add_goal_server_;

  void callback_add_new_goal_pose(const my_robot_interfaces::srv::TurtlePose::Request::SharedPtr req,
                             const my_robot_interfaces::srv::TurtlePose::Response::SharedPtr res)
  {
    turtle_name_list.push_back(req->name);
    pose_x_list.push_back(req->pose_x);
    pose_y_list.push_back(req->pose_y);

    res->success = true;

    RCLCPP_INFO(this->get_logger(), "pos: {%s, %f, %f}", req->name.c_str(), (double) req->pose_x, (double)req->pose_y);
  }

  void call_kill_service(std::string req_name)
  {
    auto client = this->create_client<turtlesim::srv::Kill>("kill");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the kill_server to be up...");
    }

    auto req = std::make_shared<turtlesim::srv::Kill::Request>();
    req->name = req_name;

    auto future = client->async_send_request(req);

    try
    {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "turtle_name_killed: {%s}", req->name.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while calling kill_service");
    }
  }

  void call_clear_service()
  {
    auto client = this->create_client<std_srvs::srv::Empty>("clear");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the clear_server to be up...");
    }

    auto req = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = client->async_send_request(req);

    try
    {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "clear line");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while calling clear_service");
    }
  }

  void update_pose(const turtlesim::msg::Pose &pose_data)
  {
    pose.x = pose_data.x;
    pose.y = pose_data.y;
    pose.theta = pose_data.theta;

    turtlesim_up_ = true;

    // RCLCPP_INFO(this->get_logger(), "'%f', '%f', '%f'", (double)pose.x, (double)pose.y, (double)pose.theta);
  }

  void move_to_goal()
  {
    if (!turtlesim_up_)
    {
      return;
    }

    if (pose_x_list.size() == 0)
    {
      return;
    }

    goal_pose.x = pose_x_list[0];
    goal_pose.y = pose_y_list[0];

    auto vel_msg = geometry_msgs::msg::Twist();
    double dist_x = goal_pose.x - pose.x;
    double dist_y = goal_pose.y - pose.y;
    double distance = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));

    if (distance > 0.25)
    {
      // position
      vel_msg.linear.x = kp_dist * distance;

      // orientation
      double steering_angle = std::atan2(dist_y, dist_x);
      double angle_diff = steering_angle - pose.theta;
      if (angle_diff > M_PI)
      {
        angle_diff -= 2 * M_PI;
      }
      else if (angle_diff < -M_PI)
      {
        angle_diff += 2 * M_PI;
      }
      vel_msg.angular.z = kp_angle * angle_diff;
    }
    else
    {
      // target reached!
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;

      threads_.push_back(std::thread(std::bind(&TurtleMoveToGoalNode::call_kill_service, this, turtle_name_list[0])));
      threads_.push_back(std::thread(std::bind(&TurtleMoveToGoalNode::call_clear_service, this)));
      turtle_name_list.erase(turtle_name_list.begin());
      pose_x_list.erase(pose_x_list.begin());
      pose_y_list.erase(pose_y_list.begin());
    }

    turtle_velocity_publisher_->publish(vel_msg);
  }
};

int main(int argc, char *argv[])
{
  srand(time(0));
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleMoveToGoalNode>());
  rclcpp::shutdown();
  return 0;
}
