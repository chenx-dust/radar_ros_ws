#include <rclcpp/rclcpp.hpp>
#include "target_matcher/matcher_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
