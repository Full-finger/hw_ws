#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

class YamlReaderNode : public rclcpp::Node
{
public:
    YamlReaderNode() : Node("yaml_reader_node")
    {
        // use_sim_time 由节点自动声明，只声明其他参数
        this->declare_parameter("frames/odom", "");
        this->declare_parameter("frames/baselink", "");
        this->declare_parameter("frames/lidar", "");
        this->declare_parameter("frames/imu", "");

        // 读取并打印
        RCLCPP_INFO(this->get_logger(), "========== YAML 参数内容 ==========");

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "use_sim_time : %s", use_sim_time ? "true" : "false");

        std::vector<std::string> frames = {
            "frames/odom", "frames/baselink", "frames/lidar", "frames/imu"
        };
        for (const auto & frame : frames)
        {
            std::string value = this->get_parameter(frame).as_string();
            RCLCPP_INFO(this->get_logger(), "%s : %s", frame.c_str(), value.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "====================================");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YamlReaderNode>();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
