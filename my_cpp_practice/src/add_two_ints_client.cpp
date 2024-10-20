#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
  AddTwoIntsClientNode() : Node("add_two_ints_client")
  {
    // thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::call_add_two_ints_service, this, 1, 4));
    threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::call_add_two_ints_service, this, 1, 4)));
    threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::call_add_two_ints_service, this, 4, 5)));
  }

  void call_add_two_ints_service(int req_a, int req_b)
  {
    auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the add_two_int_server to be up...");
    }

    auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    req->a = req_a;
    req->b = req_b;

    auto future = client->async_send_request(req);

    try
    {
      auto res = future.get();
      RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)req->a, (int)req->b, (int)res->sum);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while calling add_two_ints_service");
    }
  }

private:
  std::vector<std::thread> threads_;
  // std::thread thread1_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}