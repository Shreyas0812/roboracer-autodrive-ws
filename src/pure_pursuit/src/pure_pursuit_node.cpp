#include "rclcpp/rclcpp.hpp"
class PurePursuitCPPNode : public rclcpp::Node
{
public:
    PurePursuitCPPNode() : Node("pure_pursuit_cpp_node")
    {
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node from CPP has been started.");
    }
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitCPPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}