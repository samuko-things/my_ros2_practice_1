#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/turtle_pose.hpp"
#include "turtlesim/srv/spawn.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

float randomFloat()
{
  return (float)(rand()) / (float)(RAND_MAX);
}
int randomInt(int a, int b)
{
  if (a > b)
    return randomInt(b, a);
  if (a == b)
    return a;
  return a + (rand() % (b - a));
}

float randomFloat(int a, int b)
{
  if (a > b)
    return randomFloat(b, a);
  if (a == b)
    return a;

  return (float)randomInt(a, b) + randomFloat();
}
double genRandNum()
{
  return randomFloat(1, 10);
}

class TurtleAddGoalPoseNode : public rclcpp::Node
{
public:
  TurtleAddGoalPoseNode() : Node("turtle_add_goal_pose")
  {
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&TurtleAddGoalPoseNode::callback_spawn_turtle, this));
  }

  void callback_spawn_turtle()
  {
    double x_pos = genRandNum();
    double y_pos = genRandNum();
    threads_.push_back(std::thread(std::bind(&TurtleAddGoalPoseNode::call_spawn_service, this, turtle_name, x_pos, y_pos)));
    threads_.push_back(std::thread(std::bind(&TurtleAddGoalPoseNode::call_add_new_goal_pose_service, this, turtle_name, x_pos, y_pos)));
    count += 1;
    turtle_name = "turtle" + std::to_string(count);
  }

  void call_add_new_goal_pose_service(std::string req_name, double req_pose_x, double req_pose_y)
  {
    auto client = this->create_client<my_robot_interfaces::srv::TurtlePose>("add_new_goal_pose");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the add_new_goal_pose_server to be up...");
    }

    auto req = std::make_shared<my_robot_interfaces::srv::TurtlePose::Request>();
    req->name = req_name;
    req->pose_x = req_pose_x;
    req->pose_y = req_pose_y;

    auto future = client->async_send_request(req);

    try
    {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "pos: {%s, %f, %f}", req->name.c_str(), (double)req->pose_x, (double)req->pose_y);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while calling add_new_goal_pose_service");
    }
  }

  void call_spawn_service(std::string req_name, double req_x, double req_y)
  {
    auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the spawn_server to be up...");
    }

    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    req->x = req_x;
    req->y = req_y;
    req->theta = 0.0;
    req->name = req_name;

    auto future = client->async_send_request(req);

    try
    {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "pos: {%s, %f, %f}", res->name.c_str(), (double)req->x, (double)req->y);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while calling spawn_service");
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::thread> threads_;
  std::string turtle_name = "turtle2";
  int count = 2;
};

int main(int argc, char **argv)
{
  srand(time(0));
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleAddGoalPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}