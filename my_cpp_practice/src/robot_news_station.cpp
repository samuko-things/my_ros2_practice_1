#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node // MODIFY NAME
{
public:
  RobotNewsStationNode() : Node("robot_news_station") // MODIFY NAME
  {
    this->declare_parameter<std::string>("robot_name", "Husky");

    robot_name = this->get_parameter("robot_name").as_string();

    publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                      std::bind(&RobotNewsStationNode::publish_news,
                                      this));

    RCLCPP_INFO(this->get_logger(), "robot_news_station node has started");
  }

  void publish_news(){
    example_interfaces::msg::String msg;
    msg.data = "Hi, this is " + robot_name +" from the Robot News Station !";
    publisher_->publish(msg);
  }

private:
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string robot_name;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewsStationNode>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}