#include "target_multiplexer/target_multiplexer.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<target_multiplexer::MultiplexerNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
