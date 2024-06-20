#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include <iostream>

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    publisher_ = this->create_publisher<cashier_system::msg::Bill>("bill", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PublisherNode::publish_bill, this));
  }

private:
  void publish_bill()
  {
    auto message = cashier_system::msg::Bill();

    std::cout << "Enter quantity: ";
    std::cin >> message.quantity;
    std::cout << "Enter price: ";
    std::cin >> message.price;
    message.total = message.quantity * message.price;
    message.bill_number = current_bill_number_;
    message.timestamp = this->now();

    RCLCPP_INFO(this->get_logger(), "Publishing Bill: %d", message.bill_number);
    publisher_->publish(message);

    current_bill_number_++;
  }

  rclcpp::Publisher<cashier_system::msg::Bill>::SharedPtr publisher_;
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
