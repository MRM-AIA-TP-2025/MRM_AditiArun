#include "rclcpp/rclcpp.hpp"

#include "cashier_system/msg/bill.hpp" 
#include "rcl_interfaces/srv/set_parameters.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    subscription_ = this->create_subscription<example_interfaces::msg::Bill>(
        "bill", 10, std::bind(&SubscriberNode::bill_callback, this, std::placeholders::_1));
    last_bill_ = example_interfaces::msg::Bill();
    this->declare_parameter<int>("inventory", 100);
    this->declare_parameter<float>("income", 0.0);
  }

private:
  void bill_callback(const example_interfaces::msg::Bill::SharedPtr msg)
  {
    last_bill_ = *msg;
    update_parameters(msg->quantity, msg->total);
  }

  void update_parameters(int quantity, float total)
  {
    auto client = this->create_client<rcl_interfaces::srv::SetParameters>("/parameter_server/set_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the parameter server...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto inventory_param = rcl_interfaces::msg::Parameter();
    auto income_param = rcl_interfaces::msg::Parameter();

    this->get_parameter("inventory", inventory_param.value.integer_value);
    this->get_parameter("income", income_param.value.double_value);

    inventory_param.name = "inventory";
    inventory_param.value.integer_value -= quantity;

    income_param.name = "income";
    income_param.value.double_value += total;

    request->parameters.push_back(inventory_param);
    request->parameters.push_back(income_param);

    auto future = client->async_send_request(request);
    try
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Parameters updated successfully.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to update parameters: %s", e.what());
    }
  }

  rclcpp::Subscription<example_interfaces::msg::Bill>::SharedPtr subscription_;
  example_interfaces::msg::Bill last_bill_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
