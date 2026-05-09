#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fitting/srv/predict_energy.hpp>
#include <cmath>
#include <chrono>
#include <array>

/**
 * @brief Extended Kalman Filter Node for estimating phase C and angular velocity ω
 * 
 * The observed signal follows: y(t) = cos(ω*t + C)
 * where C is an unknown phase and ω is the angular velocity to estimate.
 * 
 * State vector: x = [C, ω]^T  (2-dimensional state)
 * Measurement: z = y(t) = cos(ω*t + C)
 * Jacobian: H = [-sin(ω*t + C), -t*sin(ω*t + C)]
 * 
 * This node subscribes to /pnp_roll_angles and provides a service /predict_energy
 * that calculates the angular difference after a given time delta.
 */
class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("little_energy_fitting")
    {
        // Declare parameters
        this->declare_parameter("process_noise_c", 1e-4);       // Process noise for phase C
        this->declare_parameter("process_noise_omega", 1e-4);   // Process noise for angular velocity ω
        this->declare_parameter("measurement_noise", 0.001);    // Measurement noise covariance R
        this->declare_parameter("initial_c", 0.0);              // Initial estimate of phase C
        this->declare_parameter("initial_omega", 5.0 * M_PI / 3.0);  // Initial estimate of ω (default: 5π/3)
        this->declare_parameter("initial_p_c", 1.0);            // Initial covariance for C
        this->declare_parameter("initial_p_omega", 0.1);        // Initial covariance for ω
        this->declare_parameter("prediction_dt", 1.0);          // Prediction time horizon (seconds)
        
        // Get parameters
        Q_c_ = this->get_parameter("process_noise_c").as_double();
        Q_omega_ = this->get_parameter("process_noise_omega").as_double();
        R_ = this->get_parameter("measurement_noise").as_double();
        C_est_ = this->get_parameter("initial_c").as_double();
        omega_est_ = this->get_parameter("initial_omega").as_double();
        prediction_dt_ = this->get_parameter("prediction_dt").as_double();
        
        // Initialize 2x2 covariance matrix P = [[P_c, 0], [0, P_omega]]
        double initial_p_c = this->get_parameter("initial_p_c").as_double();
        double initial_p_omega = this->get_parameter("initial_p_omega").as_double();
        P_ = {initial_p_c, 0.0, 0.0, initial_p_omega};  // Row-major: [P00, P01, P10, P11]
        
        // Initialize time
        start_time_ = this->now();
        last_measurement_time_ = this->now();
        current_angle_ = 0.0;
        
        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/ene_pnp_roll_angles", 10,
            std::bind(&EKFNode::measurementCallback, this, std::placeholders::_1));
        
        // Create publisher
        prediction_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_pnp_roll_angles_predict", 10);
        
        // Create service server
        predict_service_ = this->create_service<fitting::srv::PredictEnergy>(
            "/predict_energy",
            std::bind(&EKFNode::predictEnergyCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Little Energy Fitting Node started (2D EKF: phase + omega)");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Process noise C (Q_c): %.6f", Q_c_);
        RCLCPP_INFO(this->get_logger(), "  Process noise omega (Q_omega): %.6f", Q_omega_);
        RCLCPP_INFO(this->get_logger(), "  Measurement noise (R): %.6f", R_);
        RCLCPP_INFO(this->get_logger(), "  Initial C: %.6f", C_est_);
        RCLCPP_INFO(this->get_logger(), "  Initial omega: %.6f (nominal 5*pi/3 = %.6f)", omega_est_, 5.0 * M_PI / 3.0);
        RCLCPP_INFO(this->get_logger(), "  Initial P: [[%.4f, %.4f], [%.4f, %.4f]]", P_[0], P_[1], P_[2], P_[3]);
        RCLCPP_INFO(this->get_logger(), "  Prediction dt: %.3f seconds", prediction_dt_);
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /pnp_roll_angles");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /pnp_roll_angles_predict");
        RCLCPP_INFO(this->get_logger(), "Service available: /predict_energy");
    }

private:
    /**
     * @brief Service callback for predicting energy angle difference
     * @param request Contains dt (time delta) for prediction
     * @param response Contains delta_mu (angular difference) and success flag
     */
    void predictEnergyCallback(
        const std::shared_ptr<fitting::srv::PredictEnergy::Request> request,
        std::shared_ptr<fitting::srv::PredictEnergy::Response> response)
    {
        // Get current time and calculate elapsed time since start
        auto current_time = this->now();
        double t_current = (current_time - start_time_).seconds();
        double t_future = t_current + request->dt;
        
        // Calculate current angle: mu(t) = cos(ω*t + C) using estimated ω
        double mu_current = omega_est_ * t_current + C_est_;
        
        // Calculate future angle: mu(t+dt) = cos(ω*(t+dt) + C)
        // double mu_future = omega_est_ * t_future + C_est_;
        
        // Calculate angular difference
        double delta_mu = omega_est_ * request->dt;
        
        // RCLCPP_INFO(this->get_logger(), 
        //            "Prediction: t=%.3f, dt=%.3f, omega=%.6f, C=%.6f, delta_mu=%.6f",
        //            t_current, request->dt, omega_est_, C_est_, delta_mu);
        
        // Store the response
        response->delta_mu = delta_mu;
        response->success = true;
    }
    
    /**
     * @brief Callback function for measurement updates
     * @param msg Measurement message from /pnp_roll_angles
     */
    void measurementCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Get current time and calculate elapsed time since start
        auto current_time = this->now();
        double t = (current_time - start_time_).seconds();
        double dt = (current_time - last_measurement_time_).seconds();
        last_measurement_time_ = current_time;
        
        double z = msg->data;  // Measurement
        
        // Store current angle
        current_angle_ = z;
        
        // =============== EKF Prediction Step ===============
        // State prediction: x_pred = x_est (both C and ω are assumed constant)
        double C_pred = C_est_;
        double omega_pred = omega_est_;
        
        // Covariance prediction: P_pred = P + Q*dt
        // Q is diagonal: [[Q_c, 0], [0, Q_omega]]
        std::array<double, 4> P_pred = {
            P_[0] + Q_c_ * dt,  P_[1],
            P_[2],              P_[3] + Q_omega_ * dt
        };
        
        // =============== EKF Update Step ===============
        // Calculate expected measurement: h(x) = cos(ω*t + C)
        double phase = omega_pred * t + C_pred;
        double h_x = std::cos(phase);
        double sin_phase = std::sin(phase);
        
        // Measurement Jacobian: H = [∂h/∂C, ∂h/∂ω] = [-sin(ω*t + C), -t*sin(ω*t + C)]
        double H_c = -sin_phase;          // ∂h/∂C
        double H_omega = -t * sin_phase;  // ∂h/∂ω
        
        // Innovation (measurement residual): y = z - h(x)
        double innovation = z - h_x;
        
        // Innovation covariance: S = H*P_pred*H^T + R (scalar)
        // S = H_c^2*P00 + 2*H_c*H_omega*P01 + H_omega^2*P11 + R
        double S = H_c * H_c * P_pred[0] + 
                   2.0 * H_c * H_omega * P_pred[1] + 
                   H_omega * H_omega * P_pred[3] + R_;
        
        // Check for numerical stability
        if (std::abs(S) < 1e-10) {
            RCLCPP_WARN(this->get_logger(), "Innovation covariance too small: %.10f, skipping update", S);
            return;
        }
        
        // Kalman Gain: K = P_pred*H^T / S (2x1 vector)
        // K = [K_c, K_omega]^T
        double K_c = (P_pred[0] * H_c + P_pred[1] * H_omega) / S;
        double K_omega = (P_pred[2] * H_c + P_pred[3] * H_omega) / S;
        
        // State update: x_est = x_pred + K * innovation
        C_est_ = C_pred + K_c * innovation;
        omega_est_ = omega_pred + K_omega * innovation;
        
        // Covariance update: P = (I - K*H) * P_pred
        // (I - K*H) is 2x2: [[1-K_c*H_c, -K_c*H_omega], [-K_omega*H_c, 1-K_omega*H_omega]]
        double I_KH_00 = 1.0 - K_c * H_c;
        double I_KH_01 = -K_c * H_omega;
        double I_KH_10 = -K_omega * H_c;
        double I_KH_11 = 1.0 - K_omega * H_omega;
        
        P_[0] = I_KH_00 * P_pred[0] + I_KH_01 * P_pred[2];
        P_[1] = I_KH_00 * P_pred[1] + I_KH_01 * P_pred[3];
        P_[2] = I_KH_10 * P_pred[0] + I_KH_11 * P_pred[2];
        P_[3] = I_KH_10 * P_pred[1] + I_KH_11 * P_pred[3];
        
        // Log update information
        RCLCPP_INFO(this->get_logger(), 
                   "t=%.3f | z=%.4f | h=%.4f | innov=%.4f | C=%.4f | omega=%.4f (nom=%.4f)",
                   t, z, h_x, innovation, C_est_, omega_est_, 5.0 * M_PI / 3.0);
        
        // =============== Prediction for Future Time ===============
        // Predict value at t + prediction_dt_ using estimated ω
        double t_future = t + prediction_dt_;
        double predicted_value = std::cos(omega_est_ * t_future + C_est_);
        
        // Publish prediction
        auto prediction_msg = std_msgs::msg::Float64();
        prediction_msg.data = predicted_value;
        prediction_publisher_->publish(prediction_msg);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Prediction for t=%.3f: %.4f (C=%.4f, omega=%.4f)",
                   t_future, predicted_value, C_est_, omega_est_);
    }
    
    // Subscriptions and publishers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prediction_publisher_;
    
    // Service server
    rclcpp::Service<fitting::srv::PredictEnergy>::SharedPtr predict_service_;
    
    // EKF state variables (2D: [C, ω])
    double C_est_;         // Estimated phase C
    double omega_est_;     // Estimated angular velocity ω
    std::array<double, 4> P_;  // 2x2 State covariance matrix (row-major)
    double Q_c_;           // Process noise for C
    double Q_omega_;       // Process noise for ω
    double R_;             // Measurement noise covariance
    double prediction_dt_; // Prediction time horizon
    double current_angle_; // Current measured angle
    
    // Time tracking
    rclcpp::Time start_time_;
    rclcpp::Time last_measurement_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<EKFNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("little_energy_fitting"), "Exception in EKF node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
