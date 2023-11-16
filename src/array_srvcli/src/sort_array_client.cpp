#include "rclcpp/rclcpp.hpp"
#include <vector>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    if (argc == 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 'input: at least 1 num');
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = 
}