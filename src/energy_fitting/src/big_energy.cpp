#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

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
 * The model follows: θ(t) = (a/b) * (1 - cos(b*t)) + (2.09 - a)*t + θ0
 * where:
 *   - a: amplitude parameter (range: 0.780-1.045)
 *   - b: angular frequency (omega, range: 1.884-2.0)
 *   - θ0: initial phase
 *
 * Estimation strategy (based on parameter_estimator_node.cpp):
 *   - Collects position measurements into a sliding window buffer
 *   - Periodically solves a nonlinear least-squares problem via Ceres
 *   - Uses time-weighted residuals (newer data → higher weight)
 *   - Detects and resets boundary-stuck parameters
 *
 * This node subscribes to /pnp_poses (PoseArray) and extracts the roll angle
 * via atan2 from each pose's quaternion, then provides a service /predict_energy_big
 * for angle prediction.
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

        // Initialize estimates
        a_est_      = initial_a_;
        b_est_      = initial_b_;
        theta0_est_ = initial_theta0_;

        // Initialize state
        first_measurement_    = true;
        estimation_valid_     = false;
        start_time_stamp_     = 0.0;
        boundary_stuck_count_ = 0;

        // Create subscriber
        position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/ene_pnp_roll_angles", 100,
            std::bind(&BigEnergyEKFNode::measurementCallback, this, std::placeholders::_1));

        // Create publishers
        prediction_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/predicted_angle", 10);
        predicted_x_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/predicted_x", 10);
        predicted_z_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/predicted_z", 10);
        a_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/a_parameter", 10);
        b_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/b_parameter", 10);
        theta0_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ene_big_energy/theta0_parameter", 10);

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
                    "Model: theta(t) = (a/b)*(1-cos(b*t)) + (2.09-a)*t + theta0");
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
     * Residual_i = sqrt(w_i) * unwrap( theta_model(t_i) - theta_measured_i )
     * theta_model(t) = (a/b) * (1 - cos(b*t)) + (2.09 - a)*t + theta0
     */
    struct AngleCostFunction {
        AngleCostFunction(const std::vector<double>& times,
                          const std::vector<double>& angles,
                          const std::vector<double>& weights)
            : times_(times), angles_(angles), weights_(weights) {}

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
                                    (T(2.09) - a) * T(times_[i]) + theta0;

                T angle_diff = unwrapAngle(predicted_angle - T(angles_[i]));
                residuals[i] = ceres::sqrt(T(weights_[i])) * angle_diff;
            }

            return true;
        }

        template <typename T>
        static T unwrapAngle(const T& angle)
        {
            T two_pi  = T(2.0 * M_PI);
            T wrapped = angle - ceres::floor((angle + T(M_PI)) / two_pi) * two_pi;
            return wrapped;
        }

    private:
        const std::vector<double>& times_;
        const std::vector<double>& angles_;
        const std::vector<double>& weights_;
    };

    /** @brief Buffered position/angle measurement */
    struct PositionData {
        double timestamp;
        double x, z;
        double angle;
    };

    // ───────────────────────── Utility helpers ─────────────────────────

    /** theta(t) = (a/b)*(1-cos(b*t)) + (2.09-a)*t + theta0 */
    double calculateTheta(double a, double b, double theta0, double t) const
    {
        if (std::abs(b) < 1e-10) {
            return (2.09 - a) * t + theta0;
        }
        return (a / b) * (1.0 - std::cos(b * t)) + (2.09 - a) * t + theta0;
    }

    /** Normalize angle to [-pi, pi] */
    static double unwrapAngle(double angle)
    {
        const double two_pi = 2.0 * M_PI;
        return angle - std::floor((angle + M_PI) / two_pi) * two_pi;
    }

    // ─────────────────────────── Callbacks ────────────────────────────

    /**
     * @brief Service callback: predict angular difference over request->dt seconds
     */
    void predictEnergyCallback(
        const std::shared_ptr<energy_fitting::srv::PredictEnergy::Request> request,
        std::shared_ptr<energy_fitting::srv::PredictEnergy::Response> response)
    {
        if (!estimation_valid_ || position_buffer_.empty()) {
            response->delta_mu = 0.0;
            response->success  = false;
            return;
        }

        double t_current = position_buffer_.back().timestamp;
        double t_future  = t_current + request->dt;

        double theta_current = calculateTheta(a_est_, b_est_, theta0_est_, t_current);
        double theta_future  = calculateTheta(a_est_, b_est_, theta0_est_, t_future);

        response->delta_mu = theta_future - theta_current;
        response->success  = true;
    }

    /**
     * @brief Measurement callback: receive raw roll angle from /pnp_roll_angles and buffer it
     *
     * The topic publishes a bounded atan2-based angle in [-π, π].
     * Continuity (unwrap) is applied later in updateTimerCallback, mirroring
     * the approach in parameter_estimator_node.cpp.
     */
    void measurementCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (first_measurement_) {
            start_time_stamp_ = this->now().seconds();
            first_measurement_ = false;
        }

        double timestamp = this->now().seconds() - start_time_stamp_;
        double angle = msg->data;  // raw roll angle in radians, bounded [-π, π]

        position_buffer_.push_back({timestamp, 0.0, 0.0, angle});

        // Keep buffer bounded
        while (position_buffer_.size() > static_cast<size_t>(window_size_ * 2)) {
            position_buffer_.pop_front();
        }
    }

    /**
     * @brief Timer callback: run Ceres sliding-window optimization
     *
     * Mirrors the approach in parameter_estimator_node.cpp:
     *   1. Extract the latest `window_size_` samples
     *   2. Apply time-based weights (newer = higher weight, range [0.5, 2.0])
     *   3. Unwrap angle sequence for continuity
     *   4. Solve nonlinear least-squares with Levenberg-Marquardt
     *   5. Detect and reset boundary-stuck parameters
     */
    void updateTimerCallback()
    {
        if (position_buffer_.size() <= static_cast<size_t>(window_size_)) {
            return;
        }

        // ── 1. Extract sliding window ──
        std::vector<double> times, angles, weights;
        for (size_t i = position_buffer_.size() - window_size_;
             i < position_buffer_.size(); ++i)
        {
            times.push_back(position_buffer_[i].timestamp);
            angles.push_back(position_buffer_[i].angle);

            // Time-weighted: older→0.5, newest→2.0
            double time_weight =
                double(i - (position_buffer_.size() - window_size_)) / window_size_;
            weights.push_back(0.5 + 1.5 * time_weight);
        }

        // ── 2. Unwrap angles for continuity ──
        for (size_t i = 1; i < angles.size(); ++i) {
            double diff = angles[i] - angles[i - 1];
            if (diff > M_PI) {
                for (size_t j = i; j < angles.size(); ++j) angles[j] -= 2.0 * M_PI;
            } else if (diff < -M_PI) {
                for (size_t j = i; j < angles.size(); ++j) angles[j] += 2.0 * M_PI;
            }
        }

        // ── 3. Build Ceres problem ──
        ceres::Problem problem;
        double parameters[3] = {a_est_, b_est_, theta0_est_};

        RCLCPP_INFO(this->get_logger(),
                    "Optimization start — a: %.6f, b: %.6f, theta0: %.6f",
                    parameters[0], parameters[1], parameters[2]);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<AngleCostFunction, ceres::DYNAMIC, 3>(
                new AngleCostFunction(times, angles, weights),
                static_cast<int>(times.size()));

        problem.AddResidualBlock(cost_function, nullptr, parameters);

        // Parameter bounds (with margin to avoid sticking at boundary)
        const double margin = 0.05;
        problem.SetParameterLowerBound(parameters, 0, min_a_ + margin);
        problem.SetParameterUpperBound(parameters, 0, max_a_ - margin);
        problem.SetParameterLowerBound(parameters, 1, min_b_ + margin);
        problem.SetParameterUpperBound(parameters, 1, max_b_ - margin);

        // ── 4. Solver options ──
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

            // ── 5. Boundary-stuck detection & reset ──
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

        // Publish estimated parameters
        auto a_msg = std_msgs::msg::Float64();
        a_msg.data = a_est_;
        a_publisher_->publish(a_msg);

        auto b_msg = std_msgs::msg::Float64();
        b_msg.data = b_est_;
        b_publisher_->publish(b_msg);

        auto theta0_msg = std_msgs::msg::Float64();
        theta0_msg.data = theta0_est_;
        theta0_publisher_->publish(theta0_msg);

        // Predict future position
        double t_current      = position_buffer_.back().timestamp;
        double t_future       = t_current + prediction_dt_;
        double predicted_theta = calculateTheta(a_est_, b_est_, theta0_est_, t_future);
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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prediction_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr predicted_x_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr predicted_z_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr a_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr b_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta0_publisher_;
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

    // Estimated state
    double a_est_;
    double b_est_;
    double theta0_est_;
    bool   estimation_valid_;
    int    boundary_stuck_count_;

    // Data buffer
    std::deque<PositionData> position_buffer_;

    // Time tracking
    double start_time_stamp_;
    bool   first_measurement_;
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
