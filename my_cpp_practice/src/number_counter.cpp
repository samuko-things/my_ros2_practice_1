#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node // MODIFY NAME
{
public:
  NumberCounterNode() : Node("number_counter") // MODIFY NAME
  {
    publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
    subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        "number", 10,
        std::bind(&NumberCounterNode::callback_number, this, std::placeholders::_1));

    server_ = this->create_service<example_interfaces::srv::SetBool>(
        "reset_counter",
        std::bind(&NumberCounterNode::callback_reset_counter, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "number_counter node has started");
  }

private:
  void callback_number(const example_interfaces::msg::Int64::SharedPtr num)
  {
    counter += num->data;
    example_interfaces::msg::Int64 num_counter;
    num_counter.data = counter;
    publisher_->publish(num_counter);

    RCLCPP_INFO(this->get_logger(), "num_received: %ld, counter_sent: %ld", num->data, num_counter.data);
  }

  void callback_reset_counter(
    const example_interfaces::srv::SetBool::Request::SharedPtr req,
    const example_interfaces::srv::SetBool::Response::SharedPtr res
  ){
    if(req->data == true){
      counter = 0;
      res->success = true;
      res->message = "SUCCESS: counter reset successful";
    }
    else {
      res->success = false;
      res->message = "ERROR: counter reset failed";
    }
  }

  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
  int counter = 0;

  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}