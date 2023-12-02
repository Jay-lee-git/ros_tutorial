#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PointPublisher : public rclcpp::Node {
public:
    PointPublisher() : Node("turtle_tf2_message_broadcaster") {
        // Create a client to spawn a turtle
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        // Initialize state variables
        turtle_spawned_ = false;

        // Create publishers and subscribers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle3/cmd_vel", 10);
        pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("turtle3/turtle_point_stamped", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&PointPublisher::on_timer, this));
    }

private:
    void on_timer() {
        if (!turtle_spawned_) {
            if (!spawner_->service_is_ready()) {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
                return;
            }
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->name = "turtle3";
            request->x = 4.0;
            request->y = 2.0;
            request->theta = 0.0;
            spawner_->async_send_request(request, std::bind(&PointPublisher::spawn_callback, this, _1));
            turtle_spawned_ = true; // Assume the turtle will be spawned successfully
        }
    }

    void spawn_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", response->name.c_str());
        sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle3/pose", 10, std::bind(&PointPublisher::handle_turtle_pose, this, _1));
    }
    
    void handle_turtle_pose(const turtlesim::msg::Pose::SharedPtr msg) {
        auto vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = 1.0;
        vel_msg.angular.z = 1.0;
        vel_pub_->publish(vel_msg);

        auto ps = geometry_msgs::msg::PointStamped();
        ps.header.stamp = this->get_clock()->now();
        ps.header.frame_id = "world";
        ps.point.x = msg->x;
        ps.point.y = msg->y;
        ps.point.z = 0.0;
        pub_->publish(ps);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;   

    bool turtle_spawned_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointPublisher>());
    rclcpp::shutdown();
    return 0;
}
