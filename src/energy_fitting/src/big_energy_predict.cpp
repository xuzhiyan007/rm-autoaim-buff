#include <rclcpp/rclcpp.hpp>
#include <energy_fitting/srv/predict_energy.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

/**
 * @brief Client node that periodically calls the /predict_energy_big service
 *
 * The dt is hardcoded to 1.0 second.
 * Each timer tick sends a request and logs the returned delta_mu.
 */
class BigEnergyPredictClient : public rclcpp::Node
{
public:
    BigEnergyPredictClient() : Node("big_energy_predict_client")
    {
        // Declare parameters (dt and call_rate from launch)
        this->declare_parameter("dt", 1.0);
        this->declare_parameter("call_rate", 10.0);
        dt_ = this->get_parameter("dt").as_double();
        double call_rate = this->get_parameter("call_rate").as_double();

        // Create service client
        predict_client_ = this->create_client<energy_fitting::srv::PredictEnergy>("/predict_energy_big");

        // Wait until the service is available
        RCLCPP_INFO(this->get_logger(), "Waiting for /predict_energy_big service...");
        while (!predict_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, retrying...");
        }
        RCLCPP_INFO(this->get_logger(), "/predict_energy_big service is ready.");

        // Create periodic timer
        auto period = std::chrono::duration<double>(1.0 / call_rate);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&BigEnergyPredictClient::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "BigEnergyPredictClient started");
        RCLCPP_INFO(this->get_logger(), "  dt (hardcoded) : %.3f s", dt_);
        RCLCPP_INFO(this->get_logger(), "  Call rate       : %.1f Hz", call_rate);
    }

private:
    void timerCallback()
    {
        if (!predict_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "/predict_energy_big service not ready, skipping.");
            return;
        }

        // Build request with hit_time = now + dt
        auto request = std::make_shared<energy_fitting::srv::PredictEnergy::Request>();
        auto now = this->now();
        auto hit_time = now + rclcpp::Duration::from_seconds(dt_);
        request->hit_time.sec = hit_time.seconds();
        request->hit_time.nanosec = hit_time.nanoseconds() % 1000000000;

        RCLCPP_INFO(this->get_logger(), "Calling /predict_energy_big with dt=%.3f s", dt_);

        // Send request asynchronously
        predict_client_->async_send_request(
            request,
            std::bind(&BigEnergyPredictClient::responseCallback, this, std::placeholders::_1));
    }

    void responseCallback(rclcpp::Client<energy_fitting::srv::PredictEnergy>::SharedFuture future)
    {
        auto response = future.get();

        if (response->success) {
            RCLCPP_INFO(this->get_logger(),
                        "/predict_energy_big -> delta_mu = %.6f rad  (%.4f deg)",
                        response->delta_mu,
                        response->delta_mu * 180.0 / M_PI);
        } else {
            RCLCPP_ERROR(this->get_logger(), "/predict_energy_big returned success=false");
        }
    }

    rclcpp::Client<energy_fitting::srv::PredictEnergy>::SharedPtr predict_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double dt_;  // Read from "dt" parameter (default 1.0 s)
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<BigEnergyPredictClient>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("big_energy_predict_client"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
