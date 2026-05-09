#include <rclcpp/rclcpp.hpp>
#include <fitting/srv/predict_energy.hpp>
#include <pose_transform/srv/compute_target_location.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// Global variable for trackbar value (0-20, actual dt = value/10)
int g_dt_trackbar_value = 10;  // Default: dt = 1.0

/**
 * @brief Client node to test the little_energy prediction service
 * 
 * This node calls the /predict_energy service with a specified dt parameter,
 * receives the delta_mu response, and then calls /compute_target_location
 * service with the received delta_mu value.
 */
class LittleEnergyClient : public rclcpp::Node
{
public:
    LittleEnergyClient() : Node("little_energy_client")
    {
        // Declare parameter for call rate
        this->declare_parameter("call_rate", 20.0);  // Hz, how frequently to call service
        
        // Get parameters
        double call_rate = this->get_parameter("call_rate").as_double();
        
        // Create OpenCV window and trackbar for dt control
        cv::namedWindow("Predict Time", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("dt x10", "Predict Time", &g_dt_trackbar_value, 20, onTrackbarChange);
        
        // Get initial dt from trackbar
        dt_ = g_dt_trackbar_value / 10.0;
        
        // Create client for predict_energy service
        predict_client_ = this->create_client<fitting::srv::PredictEnergy>("/predict_energy");
        
        // Create client for compute_target_location service
        target_location_client_ = this->create_client<pose_transform::srv::ComputeTargetLocation>(
            "/compute_target_location");
        
        // Wait for service to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for /predict_energy service...");
        while (!predict_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Waiting for /compute_target_location service...");
        while (!target_location_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
        
        // Create timer to call service periodically
        auto period = std::chrono::duration<double>(1.0 / call_rate);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&LittleEnergyClient::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Little Energy Client started");
        RCLCPP_INFO(this->get_logger(), "dt parameter: %.3f seconds", dt_);
        RCLCPP_INFO(this->get_logger(), "Call rate: %.1f Hz", call_rate);
    }

private:
    // Static callback for trackbar (required by OpenCV)
    static void onTrackbarChange(int value, void* userdata)
    {
        (void)userdata;  // Unused
        // dt = trackbar_value / 10.0, range: 0.0 to 2.0
        double dt = value / 10.0;
        // std::cout << "[Predict Time] dt changed to: " << dt << " seconds" << std::endl;
    }
    
    void timerCallback()
    {
        // Process OpenCV events (for trackbar to work)
        cv::waitKey(1);
        
        // Update dt from trackbar value
        dt_ = g_dt_trackbar_value / 10.0;
        
        if (!predict_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Predict energy service not ready");
            return;
        }
        
        // Create request
        auto request = std::make_shared<fitting::srv::PredictEnergy::Request>();
        request->dt = dt_;
        
        RCLCPP_INFO(this->get_logger(), "Calling /predict_energy service with dt=%.3f", dt_);
        
        // Call service asynchronously with callback
        auto future = predict_client_->async_send_request(
            request,
            std::bind(&LittleEnergyClient::predictEnergyResponseCallback, this, std::placeholders::_1));
    }
    
    void predictEnergyResponseCallback(rclcpp::Client<fitting::srv::PredictEnergy>::SharedFuture future)
    {
        auto response = future.get();
        
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), 
                       "PredictEnergy service call successful! delta_mu = %.6f radians", 
                       response->delta_mu);
            
            // Now call compute_target_location with the received delta_mu
            if (target_location_client_->service_is_ready()) {
                auto target_request = std::make_shared<pose_transform::srv::ComputeTargetLocation::Request>();
                target_request->delta_mu = response->delta_mu;
                
                RCLCPP_INFO(this->get_logger(), 
                           "Calling /compute_target_location service with delta_mu=%.6f", 
                           response->delta_mu);
                
                // Call service asynchronously with callback
                auto target_future = target_location_client_->async_send_request(
                    target_request,
                    std::bind(&LittleEnergyClient::targetLocationResponseCallback, this, std::placeholders::_1));
            } else {
                RCLCPP_WARN(this->get_logger(), 
                           "/compute_target_location service not ready");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "PredictEnergy service call failed!");
        }
    }
    
    void targetLocationResponseCallback(
        rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedFuture future)
    {
        auto response = future.get();
        
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), 
                       "ComputeTargetLocation service call successful!");
            RCLCPP_INFO(this->get_logger(), 
                       "Target position: [%.3f, %.3f, %.3f]",
                       response->p_final.x, response->p_final.y, response->p_final.z);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "ComputeTargetLocation service call failed!");
        }
    }
    
    rclcpp::Client<fitting::srv::PredictEnergy>::SharedPtr predict_client_;
    rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedPtr target_location_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double dt_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<LittleEnergyClient>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("little_energy_client"), 
                    "Exception in client node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
