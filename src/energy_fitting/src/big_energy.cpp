#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <energy_fitting/srv/predict_energy.hpp>
#include <ceres/ceres.h>
#include <cmath>
#include <chrono>
#include <deque>
#include <vector>
#include <cstdlib>

/**
 * @brief Sliding-Window Ceres Optimization Node for Non-Uniform Circular Motion
 *
 * The model follows: θ(t) = (a/b) * (1 - cos(b*t)) + dir * (2.09 - a)*t + θ0
 *   - dir: rotation direction (+1 = CCW / positive, -1 = CW / negative)
 * where:
 *   - a: amplitude parameter (range: 0.780-1.045)
 *   - b: angular frequency (omega, range: 1.884-2.0)
 *   - θ0: initial phase
 *
 * Estimation strategy:
 *   - Collects raw yaw measurements into a sliding window buffer
 *   - Pre-computes cos(5*theta) for each measurement, making residuals immune
 *     to target-blade jumps of 2*pi/5 (5-fold symmetry of the energy wheel)
 *   - Periodically solves a nonlinear least-squares problem via Ceres
 *   - Uses time-weighted residuals (newer data → higher weight)
 *   - Detects and resets boundary-stuck parameters
 *   - No angle unwrapping required: cos(5*theta) ∈ [-1, 1] is naturally bounded
 *
 * This node subscribes to /ene_pnp_yaw and provides a service /predict_energy_big
 * for angle prediction. The predicted angular difference theta(t_future)-theta(t_current)
 * is independent of theta0, so the 5-fold phase degeneracy has no effect on output.
 */
class BigEnergyEKFNode : public rclcpp::Node
{
public:
    BigEnergyEKFNode() : Node("big_energy_fitting")
    {
        // Declare parameters
        this->declare_parameter("window_size", 50);
        this->declare_parameter("update_rate", 10.0);   // Hz
        this->declare_parameter("publish_rate", 10.0);  // Hz
        this->declare_parameter("prediction_dt", 0.2);  // seconds
        this->declare_parameter("radius", 2.0);         // meters

        this->declare_parameter("initial_a", 0.9);
        this->declare_parameter("initial_b", 1.94);
        this->declare_parameter("initial_theta0", 0.0);

        this->declare_parameter("min_a", 0.780);
        this->declare_parameter("max_a", 1.045);
        this->declare_parameter("min_b", 1.884);
        this->declare_parameter("max_b", 2.000);
        this->declare_parameter("direction", 1);   // +1 = CCW, -1 = CW

        // Get parameters
        window_size_   = this->get_parameter("window_size").as_int();
        update_rate_   = this->get_parameter("update_rate").as_double();
        publish_rate_  = this->get_parameter("publish_rate").as_double();
        prediction_dt_ = this->get_parameter("prediction_dt").as_double();
        radius_        = this->get_parameter("radius").as_double();

        initial_a_      = this->get_parameter("initial_a").as_double();
        initial_b_      = this->get_parameter("initial_b").as_double();
        initial_theta0_ = this->get_parameter("initial_theta0").as_double();

        min_a_ = this->get_parameter("min_a").as_double();
        max_a_ = this->get_parameter("max_a").as_double();
        min_b_ = this->get_parameter("min_b").as_double();
        max_b_ = this->get_parameter("max_b").as_double();
        dir_   = this->get_parameter("direction").as_int();
        if (dir_ != 1 && dir_ != -1) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid direction parameter (%d), must be +1 or -1. Falling back to +1 (CCW).",
                        dir_);
            dir_ = 1;
        }

        // Initialize estimates
        a_est_      = initial_a_;
        b_est_      = initial_b_;
        theta0_est_ = initial_theta0_;

        // Initialize state
        start_time_initialized_ = false;
        estimation_valid_        = false;
        boundary_stuck_count_    = 0;

        // Create subscriber (订阅带图像采集时间戳的 cos(5θ) 话题)
        position_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/ene_pnp_roll_angles_stamped", 100,
            std::bind(&BigEnergyEKFNode::measurementCallback, this, std::placeholders::_1));

        // Create publishers
        prediction_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/predicted_angle", 10);
        predicted_x_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/predicted_x", 10);
        predicted_z_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/predicted_z", 10);

        // Create service server
        predict_service_ = this->create_service<energy_fitting::srv::PredictEnergy>(
            "/predict_energy_big",
            std::bind(&BigEnergyEKFNode::predictEnergyCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Create timers
        update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate_),
            std::bind(&BigEnergyEKFNode::updateTimerCallback, this));

        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&BigEnergyEKFNode::publishTimerCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Big Energy Fitting Node started (Ceres sliding-window: a, b, theta0)");
        RCLCPP_INFO(this->get_logger(),
                    "Model: theta(t) = (a/b)*(1-cos(b*t)) + dir*(2.09-a)*t + theta0");
        RCLCPP_INFO(this->get_logger(),
                    "  Direction: %+d (%s)",
                    dir_, dir_ >= 0 ? "CCW / positive" : "CW / negative");
        RCLCPP_INFO(this->get_logger(),
                    "  Initial a=%.4f, b=%.4f, theta0=%.4f", a_est_, b_est_, theta0_est_);
        RCLCPP_INFO(this->get_logger(),
                    "  Window size: %d, Update rate: %.1f Hz, Publish rate: %.1f Hz",
                    window_size_, update_rate_, publish_rate_);
        RCLCPP_INFO(this->get_logger(), "Service available: /predict_energy_big");
    }

private:
    /**
     * @brief Ceres cost function for sliding-window angle fitting
     *
     * Residual_i = sqrt(w_i) * (cos(5*theta_model(t_i)) - cos(5*theta_measured_i))
     * theta_model(t) = (a/b) * (1 - cos(b*t)) + dir * (2.09 - a)*t + theta0
     * dir: +1 = CCW (positive rotation), -1 = CW (negative rotation)
     *
     * Using cos(5*theta) as the fitting target makes the residual immune to
     * target-blade jumps of 2*pi/5: cos(5*(theta + k*2pi/5)) = cos(5*theta).
     * No angle unwrapping is needed because cos(5*theta) is naturally bounded
     * in [-1, 1]. The 5-fold phase degeneracy in theta0 has no effect on the
     * predicted angular difference theta(t_future) - theta(t_current).
     */
    struct AngleCostFunction {
        AngleCostFunction(const std::vector<double>& times,
                          const std::vector<double>& cos5_angles,
                          const std::vector<double>& weights,
                          int dir)
            : times_(times), cos5_angles_(cos5_angles), weights_(weights), dir_(dir) {}

        template <typename T>
        bool operator()(const T* const parameters, T* residuals) const
        {
            const T& a      = parameters[0];
            const T& b      = parameters[1];
            const T& theta0 = parameters[2];

            if (ceres::abs(b) < T(1e-10)) {
                return false;
            }

            for (size_t i = 0; i < times_.size(); ++i) {
                T predicted_angle = (a / b) * (T(1.0) - ceres::cos(b * T(times_[i]))) +
                                    T(dir_) * (T(2.09) - a) * T(times_[i]) + theta0;

                T predicted_cos5  = ceres::cos(T(5.0) * predicted_angle);
                residuals[i] = ceres::sqrt(T(weights_[i])) * (predicted_cos5 - T(cos5_angles_[i]));
            }

            return true;
        }

    private:
        const std::vector<double>& times_;
        const std::vector<double>& cos5_angles_;
        const std::vector<double>& weights_;
        int dir_;  // rotation direction: +1 = CCW (positive), -1 = CW (negative)
    };

    /** @brief Buffered position/angle measurement */
    struct PositionData {
        double timestamp;
        double x, z;
        double angle;
    };

    // ───────────────────────── Utility helpers ─────────────────────────

    /** theta(t) = (a/b)*(1-cos(b*t)) + dir*(2.09-a)*t + theta0
     *  @param dir  rotation direction: +1 = CCW (positive), -1 = CW (negative) */
    double calculateTheta(double a, double b, double theta0, double t, int dir = 1) const
    {
        if (std::abs(b) < 1e-10) {
            return dir * (2.09 - a) * t + theta0;
        }
        return (a / b) * (1.0 - std::cos(b * t)) + dir * (2.09 - a) * t + theta0;
    }

    /** Normalize angle to [-pi, pi] */
    static double unwrapAngle(double angle)
    {
        const double two_pi = 2.0 * M_PI;
        return angle - std::floor((angle + M_PI) / two_pi) * two_pi;
    }

    // ─────────────────────────── Callbacks ────────────────────────────

    /**
     * @brief Service callback: predict angular difference from last capture to hit_time
     *
     * 预测区间：从最后一帧图像采集时刻 → 打中时刻
     * 使用与 Ceres 拟合严格一致的时间轴（以首帧采集时刻为零点），
     * 消除采集时刻与拟合时刻不对应的问题。
     */
    void predictEnergyCallback(
        const std::shared_ptr<energy_fitting::srv::PredictEnergy::Request> request,
        std::shared_ptr<energy_fitting::srv::PredictEnergy::Response> response)
    {
        if (!estimation_valid_ || position_buffer_.empty() || !start_time_initialized_) {
            response->delta_mu = 0.0;
            response->success  = false;
            return;
        }

        // 将请求中的打中时刻转换为 rclcpp::Time
        rclcpp::Time hit_time(request->hit_time);

        // t_current：最后一帧采集时刻在拟合时间轴上的坐标
        // t_future ：打中时刻在同一时间轴上的坐标
        double t_current = (last_capture_time_ - start_time_).seconds();
        double t_future  = (hit_time - start_time_).seconds();

        double theta_current = calculateTheta(a_est_, b_est_, theta0_est_, t_current, dir_);
        double theta_future  = calculateTheta(a_est_, b_est_, theta0_est_, t_future, dir_);

        response->delta_mu = theta_future - theta_current;
        response->success  = true;
    }

    /**
     * @brief Measurement callback: receive stamped cos(5θ) from /ene_pnp_roll_angles_stamped
     *
     * header.stamp = 图像采集时刻（由上游透传）
     * vector.x    = cos(5*theta)，已由上游计算，无需重新计算
     * 使用图像采集时间戳而非 this->now()，保证与 Ceres 拟合所用时间轴严格对应。
     */
    void measurementCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        // 使用消息中携带的图像采集时间戳
        rclcpp::Time capture_time(msg->header.stamp);

        // 首帧到达时，将时间零点锁定为该帧的采集时刻
        if (!start_time_initialized_) {
            start_time_          = capture_time;
            last_capture_time_   = capture_time;
            start_time_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Time origin locked to first capture timestamp: %.3f s",
                        capture_time.seconds());
        }

        double timestamp  = (capture_time - start_time_).seconds();
        double cos5_angle = msg->vector.x;  // cos(5*theta)，由上游透传，无需重新计算

        last_capture_time_ = capture_time;
        position_buffer_.push_back({timestamp, 0.0, 0.0, cos5_angle});

        // Keep buffer bounded
        while (position_buffer_.size() > static_cast<size_t>(window_size_ * 2)) {
            position_buffer_.pop_front();
        }
    }

    /**
     * @brief Timer callback: run Ceres sliding-window optimization
     *
     *   1. Extract the latest `window_size_` samples
     *   2. Apply time-based weights (newer = higher weight, range [0.5, 2.0])
     *   3. Solve nonlinear least-squares with Levenberg-Marquardt
     *      (cos(5θ) is stored directly in the buffer from upstream, no recomputation needed)
     *   4. Detect and reset boundary-stuck parameters
     */
    void updateTimerCallback()
    {
        if (position_buffer_.size() <= static_cast<size_t>(window_size_)) {
            return;
        }

        // ── 1. Extract sliding window ──
        // cos(5*theta) is already stored in position_buffer_[i].angle (set by measurementCallback).
        // No recomputation needed: immune to target-blade jumps of 2*pi/5 and bounded in [-1,1].
        std::vector<double> times, cos5_angles, weights;
        for (size_t i = position_buffer_.size() - window_size_;
             i < position_buffer_.size(); ++i)
        {
            times.push_back(position_buffer_[i].timestamp);
            cos5_angles.push_back(position_buffer_[i].angle);  // 已是 cos(5*theta)

            // Time-weighted: older→0.5, newest→2.0
            double time_weight =
                double(i - (position_buffer_.size() - window_size_)) / window_size_;
            weights.push_back(0.5 + 1.5 * time_weight);
        }

        // ── 2. Build Ceres problem ──
        ceres::Problem problem;
        double parameters[3] = {a_est_, b_est_, theta0_est_};

        RCLCPP_INFO(this->get_logger(),
                    "Optimization start — a: %.6f, b: %.6f, theta0: %.6f",
                    parameters[0], parameters[1], parameters[2]);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<AngleCostFunction, ceres::DYNAMIC, 3>(
                new AngleCostFunction(times, cos5_angles, weights, dir_),
                static_cast<int>(times.size()));

        problem.AddResidualBlock(cost_function, nullptr, parameters);

        // Parameter bounds (with margin to avoid sticking at boundary)
        const double margin = 0.05;
        problem.SetParameterLowerBound(parameters, 0, min_a_ + margin);
        problem.SetParameterUpperBound(parameters, 0, max_a_ - margin);
        problem.SetParameterLowerBound(parameters, 1, min_b_ + margin);
        problem.SetParameterUpperBound(parameters, 1, max_b_ - margin);

        // ── 3. Solver options ──
        ceres::Solver::Options options;
        options.max_num_iterations            = 200;
        options.linear_solver_type            = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout  = false;
        options.function_tolerance            = 1e-8;
        options.gradient_tolerance            = 1e-10;
        options.parameter_tolerance           = 1e-8;
        options.trust_region_strategy_type    = ceres::LEVENBERG_MARQUARDT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        RCLCPP_INFO(this->get_logger(),
                    "Optimization result — a: %.6f, b: %.6f, theta0: %.6f",
                    parameters[0], parameters[1], parameters[2]);
        RCLCPP_INFO(this->get_logger(),
                    "Termination type: %d, iterations: %zu, final cost: %.6f",
                    static_cast<int>(summary.termination_type),
                    summary.iterations.size(), summary.final_cost);

        if (summary.termination_type == ceres::CONVERGENCE) {
            a_est_      = parameters[0];
            b_est_      = parameters[1];
            theta0_est_ = parameters[2];

            // ── 4. Boundary-stuck detection & reset ──
            bool at_upper_a = (a_est_ > (max_a_ - 0.1));
            bool at_lower_a = (a_est_ < (min_a_ + 0.1));
            bool at_upper_b = (b_est_ > (max_b_ - 0.1));
            bool at_lower_b = (b_est_ < (min_b_ + 0.1));

            if ((at_upper_a || at_lower_a || at_upper_b || at_lower_b) &&
                summary.final_cost > 20.0)
            {
                boundary_stuck_count_++;
                if (boundary_stuck_count_ > 5) {
                    RCLCPP_WARN(this->get_logger(),
                                "Parameters stuck at boundaries, resetting to random values");
                    a_est_      = min_a_ + (max_a_ - min_a_) *
                                  (static_cast<double>(rand()) / RAND_MAX);
                    b_est_      = min_b_ + (max_b_ - min_b_) *
                                  (static_cast<double>(rand()) / RAND_MAX);
                    theta0_est_ = -M_PI + 2.0 * M_PI *
                                  (static_cast<double>(rand()) / RAND_MAX);
                    boundary_stuck_count_ = 0;
                }
            } else {
                boundary_stuck_count_ = 0;
            }

            estimation_valid_ = true;
        }
    }

    /**
     * @brief Timer callback: publish estimated parameters and predictions
     */
    void publishTimerCallback()
    {
        if (!estimation_valid_ || position_buffer_.empty()) {
            return;
        }

        // Predict future position
        double t_current      = position_buffer_.back().timestamp;
        double t_future       = t_current + prediction_dt_;
        double predicted_theta = calculateTheta(a_est_, b_est_, theta0_est_, t_future, dir_);
        double predicted_x    = radius_ * std::cos(predicted_theta);
        double predicted_z    = radius_ * std::sin(predicted_theta);

        // Compress to [-1, 1] using the same formula as fitting_node.cpp:
        // cos(theta * 5), consistent with /pnp_roll_angles encoding
        double predicted_theta_normalized = std::cos(predicted_theta * 5.0);

        auto prediction_msg = std_msgs::msg::Float64();
        prediction_msg.data = predicted_theta_normalized;
        prediction_publisher_->publish(prediction_msg);

        auto pred_x_msg = std_msgs::msg::Float64();
        pred_x_msg.data = predicted_x;
        predicted_x_publisher_->publish(pred_x_msg);

        auto pred_z_msg = std_msgs::msg::Float64();
        pred_z_msg.data = predicted_z;
        predicted_z_publisher_->publish(pred_z_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Prediction for t=%.3f: theta_raw=%.4f | normalized=%.4f | x=%.4f | z=%.4f",
                    t_future, predicted_theta, predicted_theta_normalized, predicted_x, predicted_z);
    }

    // ──────────────────────── Member variables ─────────────────────────

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prediction_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr predicted_x_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr predicted_z_publisher_;
    rclcpp::Service<energy_fitting::srv::PredictEnergy>::SharedPtr predict_service_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Node parameters
    int    window_size_;
    double update_rate_;
    double publish_rate_;
    double prediction_dt_;
    double radius_;
    double initial_a_, initial_b_, initial_theta0_;
    double min_a_, max_a_, min_b_, max_b_;
    int    dir_;   // rotation direction: +1 = CCW, -1 = CW

    // Estimated state
    double a_est_;
    double b_est_;
    double theta0_est_;
    bool   estimation_valid_;
    int    boundary_stuck_count_;

    // Data buffer
    std::deque<PositionData> position_buffer_;

    // Time tracking
    rclcpp::Time start_time_;
    rclcpp::Time last_capture_time_;  // 最后一帧图像采集时刻（由上游 header.stamp 透传）
    bool start_time_initialized_{false};  // 首帧到达前为 false，零点尚未锁定
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<BigEnergyEKFNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("big_energy_fitting"),
                     "Exception in Big Energy node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
