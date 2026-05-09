#include <rclcpp/rclcpp.hpp>
#include <pose_transform/srv/compute_target_location.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TargetLocationCaller : public rclcpp::Node
{
public:
    TargetLocationCaller() : Node("target_location_caller")
    {
        // 声明参数
        this->declare_parameter("delta_mu", 0.0);
        this->declare_parameter("publish_rate", 10.0);  // 默认10Hz

        // 获取参数
        delta_mu_ = this->get_parameter("delta_mu").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // 创建客户端
        client_ = this->create_client<pose_transform::srv::ComputeTargetLocation>(
            "/compute_target_location");

        // 等待服务可用
        RCLCPP_INFO(this->get_logger(), "Waiting for service /compute_target_location...");
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        // 创建定时器
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&TargetLocationCaller::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Target Location Caller started");
        RCLCPP_INFO(this->get_logger(), "Delta mu: %.3f rad", delta_mu_);
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate);
    }

private:
    void timerCallback()
    {
        if (!client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Service not ready");
            return;
        }

        auto request = std::make_shared<pose_transform::srv::ComputeTargetLocation::Request>();
        request->delta_mu = delta_mu_;

        // 使用异步调用，不阻塞
        auto future = client_->async_send_request(request);

        // 可选：可以添加回调处理响应
        // 但为了高频率，这里不等待响应
    }

    rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double delta_mu_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetLocationCaller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
