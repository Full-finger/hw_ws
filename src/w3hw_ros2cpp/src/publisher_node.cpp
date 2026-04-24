#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class HelloWorldPublisher : public rclcpp::Node
{
public:
    HelloWorldPublisher() : Node("hello_world_publisher"), count_(0)
    {
        //创建发布者，发布到 "hello_world" 话题，消息类型为 String
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
        //设置发布频率为10Hz（每0.1秒发布一次）
        timer_ = this->create_wall_timer(100ms, std::bind(&HelloWorldPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "HelloWorld发布者已启动，频率10Hz");
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "helloworld";
        publisher_->publish(msg);
        count_++;

        //每10次（1秒）打印一次日志
        if (count_ % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "发布第 %ld 次: \"%s\"", count_, msg.data.c_str());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldPublisher>());
    rclcpp::shutdown();
    return 0;
}
