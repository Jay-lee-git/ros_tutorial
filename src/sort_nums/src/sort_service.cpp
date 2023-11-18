#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/sort_numbers.hpp"

#include <memory>
#include <algorithm>
#include <iostream>

class SortService : public rclcpp::Node {
public:
  SortService() : Node("sort_service") {
    service_ = this->create_service<tutorial_interfaces::srv::SortNumbers>(
        "sort_numbers", 
        std::bind(&SortService::handle_sort_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_sort_request(
      const std::shared_ptr<tutorial_interfaces::srv::SortNumbers::Request> request,
      std::shared_ptr<tutorial_interfaces::srv::SortNumbers::Response> response) {

    auto &numbers = request->numbers_to_sort.data;
    std::sort(numbers.begin(), numbers.end());

    RCLCPP_INFO(this->get_logger(), "Service: sorted numbers:");
    for (const auto& number : numbers) {
        std::cout << number << " ";
    }
    std::cout << std::endl;

    response->sorted_numbers.data = numbers;
  }

  rclcpp::Service<tutorial_interfaces::srv::SortNumbers>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SortService>());
  rclcpp::shutdown();
  return 0;
}
