#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    // Declare parameters
    this->declare_parameter<int>("inventory", 100);
    this->declare_parameter<double>("income", 0.0);

    subscription_ = this->create_subscription<cashier_system::msg::Bill>(
        "bill", 10, std::bind(&SubscriberNode::bill_callback, this, std::placeholders::_1));
  }

private:
  void bill_callback(const cashier_system::msg::Bill::SharedPtr msg)
  {
    int inventory = this->get_parameter("inventory").as_int();
    double income = this->get_parameter("income").as_double();

    inventory -= msg->quantity;
    income += msg->total;

    RCLCPP_INFO(this->get_logger(), "Received Bill: %d, Updated Inventory: %d, Updated Income: %.2f",
                msg->bill_number, inventory, income);

    // Update parameters
    this->set_parameter(rclcpp::Parameter("inventory", inventory));
    this->set_parameter(rclcpp::Parameter("income", income));
  }

  rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
