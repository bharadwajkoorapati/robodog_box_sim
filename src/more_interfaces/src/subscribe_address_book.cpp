#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookSubscriber : public rclcpp::Node 
{
public:
    AddressBookSubscriber()
    : Node("address_book_subscriber")
    {
        address_book_subscriber_ = 
            this->create_subscriber<more_interfaces::msg::AddressBook>(
                "address_book", 
                10, 
                std::bind(&AddressBookSubscriber::sub_callback, this, std::placeholders::_1)
            );   
    }

private:
    void sub_callback(const more_interfaces::msg::AddressBook::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s %s'", msg->first_name.c_str(), msg->last_name.c_str());
    }

    rclcpp::Subscriber<more_interfaces::msg::AddressBook>::SharedPtr address_book_subscriber_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookSubscriber>());
    rclcpp::shutdown();

    return 0;
}
