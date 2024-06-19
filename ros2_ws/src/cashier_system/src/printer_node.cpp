#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

class PrinterNode : public rclcpp::Node
{
public:
  PrinterNode() : Node("printer_node")
  {
    subscription_ = this->create_subscription<example_interfaces::msg::Bill>(
        "bill", 10, std::bind(&PrinterNode::bill_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&PrinterNode::print_status, this));
  }

private:
  void bill_callback(const example_interfaces::msg::Bill::SharedPtr msg)
  {
    last_bill_ = *msg;
  }

  void print_status()
  {
    RCLCPP_INFO(this->get_logger(), "Last Bill Number: %d", last_bill_.bill_number);
    RCLCPP_INFO(this->get_logger(), "Last Bill Total: %.2f", last_bill_.total);

    auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/parameter_server/get_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the parameter server...");
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("inventory");
    request->names.push_back("income");

    auto future = client->async_send_request(request);
    try
    {
      auto response = future.get();
      int inventory = response->values[0].integer_value;
      float income = response->values[1].double_value;
      RCLCPP_INFO(this->get_logger(), "Current Inventory: %d", inventory);
      RCLCPP_INFO(this->get_logger(), "Current Income: %.2f", income);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameters: %s", e.what());
    }
  }

  rclcpp::Subscription<example_interfaces::msg::Bill>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  example_interfaces::msg::Bill last_bill_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrinterNode>());
  rclcpp::shutdown();
  return 0;
}
