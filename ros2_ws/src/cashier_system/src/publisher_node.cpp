
#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"  


class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    publisher_ = this->create_publisher<example_interfaces::msg::Bill>("bill", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PublisherNode::publish_bill, this));
  }

private:
  void publish_bill()
  {
    auto message = example_interfaces::msg::Bill();
    message.bill_number = current_bill_number_;
    message.timestamp = this->now();
    message.quantity = 10;
    message.price = 5.0;
    message.total = message.quantity * message.price;

    RCLCPP_INFO(this->get_logger(), "Publishing Bill: %d", message.bill_number);
    publisher_->publish(message);

    current_bill_number_++;
  }

  rclcpp::Publisher<example_interfaces::msg::Bill>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int current_bill_number_ = 1;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
