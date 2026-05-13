#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <energy_fitting/srv/predict_energy.hpp>
#include <cmath>
#include <chrono>
#include <array>
#include <deque>

/**
 * @brief Extended Kalman Filter Node for estimating phase C and angular velocity ω
 *
 * Uses 2D measurement [cos(ω*t+C), sin(ω*t+C)] to resolve the sign ambiguity of ω
 * (i.e., distinguish clockwise from counterclockwise rotation).
 *
 * State vector: x = [C, ω]^T  (2-dimensional state)
 * Measurement:  z = [cos(ω*t+C), sin(ω*t+C)]^T  (2-dimensional)
 * Jacobian H (2x2):
 *   H = [[-sin(phase), -t*sin(phase)],
 *        [ cos(phase),  t*cos(phase)]]
 *
 * This node subscribes to /ene_pnp_roll_angles_stamped (vector.x=cos, vector.y=sin)
 * and provides a service /predict_energy that calculates the angular difference
 * after a given time delta.
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
        
        // Create subscriber (订阅带图像采集时间戳的话题，vector.x=cos，vector.y=sin)
        subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/ene_pnp_roll_angles_stamped", 10,
            std::bind(&EKFNode::measurementCallback, this, std::placeholders::_1));
        
        // Create publisher
        prediction_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_pnp_roll_angles_predict", 10);
        dt_capture_to_hit_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_dt_capture_to_hit", 10);
        omega_est_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_omega_est", 10);
        dir_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "/ene_dir", 10);
        
        // Create service server
        predict_service_ = this->create_service<energy_fitting::srv::PredictEnergy>(
            "/predict_energy",
            std::bind(&EKFNode::predictEnergyCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        // RCLCPP_INFO(this->get_logger(), "Little Energy Fitting Node started (2D EKF: phase + omega, 2D obs: [cos,sin])");
        // RCLCPP_INFO(this->get_logger(), "Parameters:");
        // RCLCPP_INFO(this->get_logger(), "  Process noise C (Q_c): %.6f", Q_c_);
        // RCLCPP_INFO(this->get_logger(), "  Process noise omega (Q_omega): %.6f", Q_omega_);
        // RCLCPP_INFO(this->get_logger(), "  Measurement noise (R): %.6f", R_);
        // RCLCPP_INFO(this->get_logger(), "  Initial C: %.6f", C_est_);
        // RCLCPP_INFO(this->get_logger(), "  Initial omega: %.6f (nominal 5*pi/3 = %.6f)", omega_est_, 5.0 * M_PI / 3.0);
        // RCLCPP_INFO(this->get_logger(), "  Initial P: [[%.4f, %.4f], [%.4f, %.4f]]", P_[0], P_[1], P_[2], P_[3]);
        // RCLCPP_INFO(this->get_logger(), "  Prediction dt: %.3f seconds", prediction_dt_);
        // RCLCPP_INFO(this->get_logger(), "Subscribing to: /ene_pnp_roll_angles_stamped (vector.x=cos, vector.y=sin)");
        // RCLCPP_INFO(this->get_logger(), "Publishing to: /ene_pnp_roll_angles_predict, /ene_omega_est");
        // RCLCPP_INFO(this->get_logger(), "Service available: /predict_energy");
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
        
        // RCLCPP_INFO(this->get_logger(),
        //             "Prediction: last_capture=%.3f s, hit=%.3f s, dt=%.4f s, omega=%.6f, delta_mu=%.6f rad",
        //             last_capture_time_.seconds(), hit_time.seconds(),
        //             dt_capture_to_hit, omega_est_, delta_mu);
        
        // Publish dt_capture_to_hit (seconds)
        auto dt_msg = std_msgs::msg::Float64();
        dt_msg.data = dt_capture_to_hit;
        dt_capture_to_hit_publisher_->publish(dt_msg);

        response->delta_mu = delta_mu;
        response->success = true;
    }
    
    /**
     * @brief Determine rotation direction using a real-time sliding window of DIR_WINDOW_SIZE frames.
     *
     * Each frame the angle diff is pushed into a fixed-length deque.  Once the deque
     * is full the mean diff is used to judge direction (+1 CW / -1 CCW) and the
     * result is published every frame.  EKF is started (return false) only after the
     * window has been filled for the first time.
     *
     * @return true  while the window is still filling (EKF should be skipped)
     *         false once direction has been initialised (EKF should run)
     */
    bool processDirection(double z_cos, double z_sin, const rclcpp::Time& capture_time)
    {
        double current_raw_angle = std::atan2(z_sin, z_cos);

        // 首帧：仅记录初始角度，不产生差分
        if (!first_angle_received_) {
            previous_angle_ = current_raw_angle;
            first_angle_received_ = true;
            last_capture_time_ = capture_time;
            return true;  // 窗口未满，跳过 EKF
        }

        double delta_angle = current_raw_angle - previous_angle_;
        delta_angle = std::fmod(delta_angle, 2.0 * M_PI);
        if (delta_angle > M_PI)       delta_angle -= 2.0 * M_PI;
        else if (delta_angle < -M_PI) delta_angle += 2.0 * M_PI;

        // 计算帧间角速度 (rad/s)
        double delta_time = (capture_time - last_capture_time_).seconds();
        double omega = (delta_time > 1e-9) ? (delta_angle / delta_time) : 0.0;
        RCLCPP_INFO(this->get_logger(), "Frame delta_time: %.4f s", delta_time);
        // 更新角速度滑动窗口（保持最近 DIR_WINDOW_SIZE 帧）
        omega_window_.push_back(omega);
        if (static_cast<int>(omega_window_.size()) > DIR_WINDOW_SIZE) {
            omega_window_.pop_front();
        }

        // 更新角度差分滑动窗口（保持最近 DIR_WINDOW_SIZE 帧）
        angle_diff_window_.push_back(delta_angle);
        if (static_cast<int>(angle_diff_window_.size()) > DIR_WINDOW_SIZE) {
            angle_diff_window_.pop_front();
        }

        previous_angle_ = current_raw_angle;
        last_capture_time_ = capture_time;

        // 逐帧角速度投票：窗口内超过 2/3 的帧同向即判定旋转方向
        int count_cw = 0, count_ccw = 0;
        for (double w : omega_window_) {
            if (w > 0.5)      count_cw++;
            else if (w < 0.5) count_ccw++;
        }
        int win_size = static_cast<int>(omega_window_.size());
        int vote_threshold = win_size * 2 / 3;  // 2/3 多数阈值
        int new_direction = 0;
        if (count_cw > vote_threshold)           new_direction = 1;
        else if (count_ccw > vote_threshold)     new_direction = -1;

        if (!direction_initialized_) {
            // 等待窗口首次填满才初始化 EKF
            if (static_cast<int>(angle_diff_window_.size()) < DIR_WINDOW_SIZE) {
                return true;  // 窗口未满，跳过 EKF
            }

            // 窗口已满：首次初始化旋转方向与 EKF 状态
            current_direction_   = new_direction;
            direction_initialized_ = true;

            omega_est_ = current_direction_ * (5.0 * M_PI / 3.0);
            C_est_     = current_raw_angle;
            start_time_             = capture_time;
            start_time_initialized_ = true;

            // RCLCPP_INFO(this->get_logger(),
            //     "Direction initialized from %d-frame window: %d (cw: %d, ccw: %d, thresh: %d). "
            //     "EKF omega set to %.4f",
            //     DIR_WINDOW_SIZE, current_direction_, count_cw, count_ccw, vote_threshold, omega_est_);
        } else {
            // EKF 运行中：实时更新窗口方向，检测方向翻转
            if (new_direction != current_direction_) {
                RCLCPP_WARN(this->get_logger(),
                    "Direction changed: %d -> %d (cw: %d, ccw: %d, thresh: %d)",
                    current_direction_, new_direction, count_cw, count_ccw, vote_threshold);
                current_direction_ = new_direction;
            }
        }

        // 每帧发布实时旋转方向
        auto dir_msg = std_msgs::msg::Int32();
        dir_msg.data = current_direction_;
        dir_publisher_->publish(dir_msg);

        return !direction_initialized_;
    }

    /**
     * @brief Callback function for measurement updates (2D EKF)
     * @param msg Stamped measurement from /ene_pnp_roll_angles_stamped
     *            header.stamp = 图像采集时刻（由上游透传）
     *            vector.x    = cos(yaw * 5)（余弦测量值）
     *            vector.y    = sin(yaw * 5)（正弦测量值，用于辨别旋转方向）
     *
     * 2D 观测模型消除了 cos 单独使用时 ω 符号的不可观测性，
     * EKF 可以从数据中自动收敛到正确的旋转方向（ω 的符号）。
     */
    void measurementCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        // 使用消息中携带的图像采集时间戳，而非 this->now()（处理时刻），
        // 保证与 EKF 拟合所用时间轴严格对应。
        rclcpp::Time capture_time(msg->header.stamp);
        
        // 2D measurement: z = [cos(yaw*5), sin(yaw*5)]
        double z_cos = msg->vector.x;
        double z_sin = msg->vector.y;

        if (processDirection(z_cos, z_sin, capture_time)) {
            return; // Skip EKF update while calculating initial direction
        }

        // 首帧到达时，将时间零点锁定为该帧的采集时刻，
        // 使 t 从 0 开始增长，相位 C 的物理含义为第一帧时的初相。
        if (!start_time_initialized_) {
            start_time_ = capture_time;
            last_capture_time_ = capture_time;
            start_time_initialized_ = true;
            // RCLCPP_INFO(this->get_logger(),
            //             "EKF time origin locked to first capture timestamp: %.3f s",
            //             capture_time.seconds());
        }
        
        double t  = (capture_time - start_time_).seconds();
        double dt = (capture_time - last_capture_time_).seconds();
        last_capture_time_ = capture_time;  // 记录图像采集时刻
        
        // Store current cos angle (for compatibility)
        current_angle_ = z_cos;
        
        // =============== EKF Prediction Step ===============
        // State prediction: x_pred = x_est (C and ω are assumed constant)
        double C_pred     = C_est_;
        double omega_pred = omega_est_;
        
        // Covariance prediction: P_pred = P + Q*dt
        std::array<double, 4> P_pred = {
            P_[0] + Q_c_ * dt,  P_[1],
            P_[2],              P_[3] + Q_omega_ * dt
        };
        
        // =============== EKF Update Step (2D obs) ===============
        // Expected measurements: h(x) = [cos(phase), sin(phase)]
        double phase     = omega_pred * t + C_pred;
        double sin_phase = std::sin(phase);
        double cos_phase = std::cos(phase);
        double h_cos = cos_phase;
        double h_sin = sin_phase;
        
        // Jacobian H (2x2):
        //   row 0 (d cos / d[C, omega]): [-sin_phase, -t*sin_phase]
        //   row 1 (d sin / d[C, omega]): [ cos_phase,  t*cos_phase]
        double H00 = -sin_phase;        double H01 = -t * sin_phase;
        double H10 =  cos_phase;        double H11 =  t * cos_phase;
        
        // Innovation: y = z - h(x)  (2x1)
        double innov0 = z_cos - h_cos;
        double innov1 = z_sin - h_sin;
        
        // P * H^T  (2x2): (P*H^T)[i,j] = P[i,0]*H[j,0] + P[i,1]*H[j,1]
        double PH_T0 = P_pred[0]*H00 + P_pred[1]*H10;  // (0,0)
        double PH_T1 = P_pred[0]*H01 + P_pred[1]*H11;  // (0,1)
        double PH_T2 = P_pred[2]*H00 + P_pred[3]*H10;  // (1,0)
        double PH_T3 = P_pred[2]*H01 + P_pred[3]*H11;  // (1,1)
        
        // S = H * P*H^T + R*I  (2x2)
        // S[i,j] = H[i,0]*PH_T[0,j] + H[i,1]*PH_T[1,j]  (+R_ on diagonal)
        double S00 = H00*PH_T0 + H01*PH_T2 + R_;
        double S01 = H00*PH_T1 + H01*PH_T3;
        double S10 = H10*PH_T0 + H11*PH_T2;
        double S11 = H10*PH_T1 + H11*PH_T3 + R_;
        
        // Check for numerical stability (det of S)
        double det_S = S00 * S11 - S01 * S10;
        if (std::abs(det_S) < 1e-10) {
            RCLCPP_WARN(this->get_logger(), "Innovation covariance det too small: %.10f, skipping update", det_S);
            return;
        }
        
        // S^-1 (2x2 analytic inverse)
        double Si00 =  S11 / det_S;
        double Si01 = -S01 / det_S;
        double Si10 = -S10 / det_S;
        double Si11 =  S00 / det_S;
        
        // K = P*H^T * S^-1  (2x2): K[i,j] = PH_T[i,0]*Si[0,j] + PH_T[i,1]*Si[1,j]
        double K00 = PH_T0*Si00 + PH_T1*Si10;
        double K01 = PH_T0*Si01 + PH_T1*Si11;
        double K10 = PH_T2*Si00 + PH_T3*Si10;
        double K11 = PH_T2*Si01 + PH_T3*Si11;
        
        // State update: x = x_pred + K * innov
        C_est_     = C_pred     + K00*innov0 + K01*innov1;
        omega_est_ = omega_pred + K10*innov0 + K11*innov1;
        
        // Covariance update: P = (I - K*H) * P_pred
        // K*H (2x2): (K*H)[i,j] = K[i,0]*H[0,j] + K[i,1]*H[1,j]
        double KH00 = K00*H00 + K01*H10;    double KH01 = K00*H01 + K01*H11;
        double KH10 = K10*H00 + K11*H10;    double KH11 = K10*H01 + K11*H11;
        
        double IKH00 = 1.0 - KH00;  double IKH01 = -KH01;
        double IKH10 = -KH10;       double IKH11 = 1.0 - KH11;
        
        P_[0] = IKH00*P_pred[0] + IKH01*P_pred[2];
        P_[1] = IKH00*P_pred[1] + IKH01*P_pred[3];
        P_[2] = IKH10*P_pred[0] + IKH11*P_pred[2];
        P_[3] = IKH10*P_pred[1] + IKH11*P_pred[3];
        
        // Log update information
        // RCLCPP_INFO(this->get_logger(),
        //            "t=%.3f | zc=%.4f zs=%.4f | hc=%.4f hs=%.4f | C=%.4f | omega=%.4f (nom=%.4f)",
        //            t, z_cos, z_sin, h_cos, h_sin, C_est_, omega_est_, 5.0 * M_PI / 3.0);
        
        // =============== Prediction for Future Time ===============
        double t_future = t + prediction_dt_;
        double predicted_value = std::cos(omega_est_ * t_future + C_est_);
        
        // Publish prediction
        auto prediction_msg = std_msgs::msg::Float64();
        prediction_msg.data = predicted_value;
        prediction_publisher_->publish(prediction_msg);

        // Publish omega_est (rad/s, sign indicates direction: +CW / -CCW viewed from camera)
        auto omega_msg = std_msgs::msg::Float64();
        omega_msg.data = omega_est_;
        omega_est_publisher_->publish(omega_msg);
        
        // RCLCPP_INFO(this->get_logger(),
        //            "Prediction for t=%.3f: %.4f (C=%.4f, omega=%.4f)",
        //            t_future, predicted_value, C_est_, omega_est_);
    }
    
    // Subscriptions and publishers
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prediction_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dt_capture_to_hit_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr omega_est_publisher_;  // rad/s, sign = rotation direction
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dir_publisher_;
    
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

    // Direction calculation variables (real-time sliding window)
    bool direction_initialized_{false};
    bool first_angle_received_{false};     // 是否已收到首帧角度
    double previous_angle_{0.0};
    int current_direction_{0};             // +1 CW / -1 CCW
    std::deque<double> angle_diff_window_; // 最近 DIR_WINDOW_SIZE 帧的角度差分
    std::deque<double> omega_window_;      // 最近 DIR_WINDOW_SIZE 帧的逐帧角速度 (rad/s)
    static constexpr int DIR_WINDOW_SIZE = 30;
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
