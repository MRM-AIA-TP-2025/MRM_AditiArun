#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include <iostream>

class PrinterNode : public rclcpp::Node
{
public:
  PrinterNode() : Node("printer_node")
  {
    subscription_ = this->create_subscription<cashier_system::msg::Bill>(
        "bill", 10, std::bind(&PrinterNode::bill_callback, this, std::placeholders::_1));

    this->declare_parameter<int>("inventory", 100);
    this->declare_parameter<double>("income", 0.0);

    param_event_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10, std::bind(&PrinterNode::parameter_event_callback, this, std::placeholders::_1));

    std::thread(&PrinterNode::listen_for_input, this).detach();
  }

private:
  void bill_callback(const cashier_system::msg::Bill::SharedPtr msg)
  {
    last_bill_ = *msg;
  }

  void parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    for (const auto &changed_parameter : event->changed_parameters)
    {
      if (changed_parameter.name == "inventory")
      {
        inventory_ = changed_parameter.value.integer_value;
      }
      else if (changed_parameter.name == "income")
      {
        income_ = changed_parameter.value.double_value;
      }
    }
  }

  void listen_for_input()
  {
    while (rclcpp::ok())
    {
      std::string input;
      std::getline(std::cin, input);

      std::cout << "Last Bill Number: " << last_bill_.bill_number << std::endl;
      std::cout << "Last Bill Quantity: " << last_bill_.quantity << std::endl;
      std::cout << "Last Bill Price: " << last_bill_.price << std::endl;
      std::cout << "Last Bill Total: " << last_bill_.total << std::endl;
      std::cout << "Current Inventory: " << inventory_ << std::endl;
      std::cout << "Current Income: " << income_ << std::endl;
    }
  }

  rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_subscriber_;
  cashier_system::msg::Bill last_bill_;
  int inventory_;
  double income_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrinterNode>());
  rclcpp::shutdown();
  return 0;
}
