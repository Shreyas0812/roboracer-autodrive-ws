#include "rclcpp/rclcpp.hpp"

class SimpleMapNode : public rclcpp::Node
{
public:
    SimpleMapNode() : Node("simple_map_node")
    {
        RCLCPP_INFO(this->get_logger(), "Simple Map Node from CPP has been started.");
    }
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}