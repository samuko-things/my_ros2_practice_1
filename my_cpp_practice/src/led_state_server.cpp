#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedStateServerNode : public rclcpp::Node
{
public:
  LedStateServerNode() : Node("led_state_server")
  {
    this->declare_parameter<std::vector<int64_t>>("led_panel_state", {0, 0, 1});

    led_panel_state = this->get_parameter("led_panel_state").as_integer_array();

    publisher_ = this->create_publisher<my_robot_interfaces::msg::LedState>("led_panel_state", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&LedStateServerNode::publish_led_states, this));

    server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
        "set_led",
        std::bind(&LedStateServerNode::callback_set_led, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "lead_state_server node has started successfully");
  }

private:
  void publish_led_states()
  {
    my_robot_interfaces::msg::LedState led_state;
    led_state.data = led_panel_state;
    publisher_->publish(led_state);
  }

  void callback_set_led(
      const my_robot_interfaces::srv::SetLed::Request::SharedPtr req,
      const my_robot_interfaces::srv::SetLed::Response::SharedPtr res)
  {
    int led_num = (int)req->led_number;
    int turn_on = (int)req->turn_on;
    led_panel_state.at(led_num) = (int64_t)turn_on;

    res->success = true;

    if (turn_on)
    {
      RCLCPP_INFO(this->get_logger(), "led number %d is on", led_num);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "led number %d is off", led_num);
    }
  }


  rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
  rclcpp::Publisher<my_robot_interfaces::msg::LedState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<int64_t> led_panel_state;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LedStateServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}













// #include "rclcpp/rclcpp.hpp"
// #include "my_robot_interfaces/msg/led_state_array.hpp"
// #include "my_robot_interfaces/srv/set_led.hpp"

// class LedPanelNode : public rclcpp::Node
// {
// public:
//   LedPanelNode() : Node("led_panel"), led_states_(3, 0)
//   {
//     led_states_publisher_ =
//         this->create_publisher<my_robot_interfaces::msg::LedStateArray>("led_states", 10);
//     led_states_timer_ =
//         this->create_wall_timer(std::chrono::seconds(4),
//                                 std::bind(&LedPanelNode::publishLedStates, this));
//     set_led_service_ = this->create_service<my_robot_interfaces::srv::SetLed>(
//         "set_led",
//         std::bind(&LedPanelNode::callbackSetLed, this, std::placeholders::_1, std::placeholders::_2));
//     RCLCPP_INFO(this->get_logger(), "Led panel node has been started");
//   }

// private:
//   void publishLedStates()
//   {
//     auto msg = my_robot_interfaces::msg::LedStateArray();
//     msg.led_states = led_states_;
//     led_states_publisher_->publish(msg);
//   }

//   void callbackSetLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
//                       const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
//   {
//     int64_t led_number = request->led_number;
//     int64_t state = request->state;

//     if (led_number > (int64_t)led_states_.size() || led_number <= 0)
//     {
//       response->success = false;
//       return;
//     }

//     if (state != 0 && state != 1)
//     {
//       response->success = false;
//       return;
//     }

//     led_states_.at(led_number - 1) = state;
//     response->success = true;
//     publishLedStates();
//   }

//   std::vector<int64_t> led_states_;

//   rclcpp::Publisher<my_robot_interfaces::msg::LedStateArray>::SharedPtr led_states_publisher_;
//   rclcpp::TimerBase::SharedPtr led_states_timer_;
//   rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_service_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<LedPanelNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }