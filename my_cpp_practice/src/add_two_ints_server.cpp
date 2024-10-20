#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
  AddTwoIntsServerNode() : Node("add_two_ints_server")
  {
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&AddTwoIntsServerNode::callback_add_two_ints, this, _1, _2));
        
    RCLCPP_INFO(this->get_logger(), "add_two_ints_server node has been started.");
  }

private:
  void callback_add_two_ints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr req,
                          const example_interfaces::srv::AddTwoInts::Response::SharedPtr res)
  {
    res->sum = req->a + req->b;
    RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)req->a, (int)req->b, (int)res->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}