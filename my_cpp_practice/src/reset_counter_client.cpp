#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

class ResetCounterClientNode : public rclcpp::Node
{
public:
  ResetCounterClientNode() : Node("reset_counter_client")
  {
    threads_.push_back(std::thread(std::bind(&ResetCounterClientNode::call_reset_counter_service, this, true)));
    delay_ms(5000);
    threads_.push_back(std::thread(std::bind(&ResetCounterClientNode::call_reset_counter_service, this, true)));
  }

  void call_reset_counter_service(bool req_data)
  {
    auto client = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the reset_counter_server to be up...");
    }

    auto req = std::make_shared<example_interfaces::srv::SetBool::Request>();
    req->data = req_data;

    auto future = client->async_send_request(req);

    try
    {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "data: %d, success: %d, message: '%s'", (int)req->data, (int)res->success, res->message.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while calling add_two_ints_service");
    }
  }

private:
  std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ResetCounterClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}