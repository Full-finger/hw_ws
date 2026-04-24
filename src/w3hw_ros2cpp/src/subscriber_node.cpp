#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloWorldSubscriber : public rclcpp::Node
{
public:
    HelloWorldSubscriber() : Node("hello_world_subscriber")
    {   
        //创建订阅者，订阅 "hello_world" 话题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "hello_world", 10,
            std::bind(&HelloWorldSubscriber::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "HelloWorld订阅者已启动，等待消息...");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {   
        //当收到消息时，输出接收到的词条
        RCLCPP_INFO(this->get_logger(), "收到消息: \"%s\"", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldSubscriber>());
    rclcpp::shutdown();
    return 0;
}
