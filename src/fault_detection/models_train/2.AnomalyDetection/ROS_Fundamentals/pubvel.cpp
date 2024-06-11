// This program publishes randomly-generated velocity
// messages for turtlesim.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv){

	// Initialize the ROS system and become a node:
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("publish_velocity");

	// Create a publisher object:
	auto pub = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

	// Seed the random number generator:
	srand(time(0));

	// Loop at 2Hz until the node is shut down:
	rclcpp::Rate rate(2s);
	while(rclcpp::ok()){
		// Create and fill in the message. The other four
		// fields, which are ignored by turtlesim, default to 0.
		auto msg = std::make_shared<geometry_msgs::msg::Twist>();
		msg->linear.x = double(rand())/double(RAND_MAX);
		msg->angular.z = 2*double(rand())/double(RAND_MAX) - 1;

		// Publish the message:
		pub->publish(msg);

		// Send a message to rosout with the details:
		RCLCPP_INFO(node->get_logger(), "Sending random velocity command:"
			" linear=%f"
			" angular=%f", msg->linear.x, msg->angular.z);

		//Wait until it's time for another iteration:
		rclcpp::spin_some(node);
		rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}