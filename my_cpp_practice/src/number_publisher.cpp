#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
  NumberPublisherNode() : Node("number_publisher")
  {
    this->declare_parameter<int>("number_to_publish", 5);
    this->declare_parameter<float>("publish_frequency", 2.0);

    number_to_publish = this->get_parameter("number_to_publish").as_int();
    publish_frequency = this->get_parameter("publish_frequency").as_double();
    time_in_ms = (int)(1000.00 / publish_frequency);

    publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(time_in_ms),
                                     std::bind(&NumberPublisherNode::publish_number,this));

    RCLCPP_INFO(this->get_logger(), "number_publisher node has started");
  }

  void publish_number()
  {
    example_interfaces::msg::Int64 num;
    num.data = number_to_publish;
    publisher_->publish(num);
  }

private:
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int number_to_publish;
  double publish_frequency;
  int time_in_ms;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}