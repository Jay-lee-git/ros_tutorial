#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/num.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber() : Node("minimal_susbscriber"){
            subscription_ = this->create_subscription<custom_interfaces::msg::Num>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }
    private:
        void topic_callback(const custom_interfaces::msg::Num & msg) const
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << '"');
        }
        rclcpp::Subscription<custom_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}