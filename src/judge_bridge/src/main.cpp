
#include <memory>
#include "judge_bridge/judge_bridge.hpp"
#include "judge_bridge/decode.hpp"



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<JudgeBridgeNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

