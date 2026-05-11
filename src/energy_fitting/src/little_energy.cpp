#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <energy_fitting/srv/predict_energy.hpp>
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
        // start_time_ 将在收到第一帧时用图像采集时间戳覆盖，此处先占位
        start_time_ = this->now();
        last_capture_time_ = this->now();
        current_angle_ = 0.0;
        
        // Create subscriber (订阅带图像采集时间戳的话题)
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/ene_pnp_roll_angles_stamped", 10,
            std::bind(&EKFNode::measurementCallback, this, std::placeholders::_1));
        
        // Create publisher
        prediction_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_pnp_roll_angles_predict", 10);
        
        // Create service server
        predict_service_ = this->create_service<energy_fitting::srv::PredictEnergy>(
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
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /ene_pnp_roll_angles_stamped (with image capture timestamp)");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /ene_pnp_roll_angles_predict");
        RCLCPP_INFO(this->get_logger(), "Service available: /predict_energy");
    }

private:
    /**
     * @brief Service callback for predicting energy angle difference
     * @param request Contains dt (time delta) for prediction
     * @param response Contains delta_mu (angular difference) and success flag
     */
    void predictEnergyCallback(
        const std::shared_ptr<energy_fitting::srv::PredictEnergy::Request> request,
        std::shared_ptr<energy_fitting::srv::PredictEnergy::Response> response)
    {
        // 将请求中的打中时刻转换为 rclcpp::Time
        rclcpp::Time hit_time(request->hit_time);
        
        // 预测区间：从最后一帧图像采集时刻 → 打中时刻
        // last_capture_time_ 在每次 EKF 更新时从消息 header.stamp 赋值，
        // 与 EKF 拟合所用时间轴严格一致，消除采集时刻与拟合时刻不对应的问题。
        double dt_capture_to_hit = (hit_time - last_capture_time_).seconds();
        
        if (dt_capture_to_hit <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                        "hit_time (%.3f s) is not after last_capture_time_ (%.3f s), dt=%.4f s",
                        hit_time.seconds(), last_capture_time_.seconds(), dt_capture_to_hit);
            response->delta_mu = 0.0;
            response->success = false;
            return;
        }
        
        // delta_mu = ω * Δt（从采集时刻到打中时刻的角度变化量）
        double delta_mu = omega_est_ * dt_capture_to_hit;
        
        RCLCPP_INFO(this->get_logger(),
                    "Prediction: last_capture=%.3f s, hit=%.3f s, dt=%.4f s, omega=%.6f, delta_mu=%.6f rad",
                    last_capture_time_.seconds(), hit_time.seconds(),
                    dt_capture_to_hit, omega_est_, delta_mu);
        
        response->delta_mu = delta_mu;
        response->success = true;
    }
    
    /**
     * @brief Callback function for measurement updates
     * @param msg Stamped measurement from /ene_pnp_roll_angles_stamped
     *            header.stamp = 图像采集时刻（由上游透传）
     *            vector.x    = cos(yaw * 5)（角度测量值）
     */
    void measurementCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        // 使用消息中携带的图像采集时间戳，而非 this->now()（处理时刻），
        // 保证与 EKF 拟合所用时间轴严格对应。
        rclcpp::Time capture_time(msg->header.stamp);
        
        // 首帧到达时，将时间零点锁定为该帧的采集时刻，
        // 使 t 从 0 开始增长，相位 C 的物理含义为第一帧时的初相。
        if (!start_time_initialized_) {
            start_time_ = capture_time;
            last_capture_time_ = capture_time;
            start_time_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "EKF time origin locked to first capture timestamp: %.3f s",
                        capture_time.seconds());
        }
        
        double t = (capture_time - start_time_).seconds();
        double dt = (capture_time - last_capture_time_).seconds();
        last_capture_time_ = capture_time;  // 记录图像采集时刻
        
        double z = msg->vector.x;  // Measurement: cos(yaw * 5)
        
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
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prediction_publisher_;
    
    // Service server
    rclcpp::Service<energy_fitting::srv::PredictEnergy>::SharedPtr predict_service_;
    
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
    rclcpp::Time last_capture_time_;  // 最后一帧图像采集时刻（由上游 header.stamp 透传）
    bool start_time_initialized_{false};  // 首帧到达前为 false，零点尚未锁定
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
