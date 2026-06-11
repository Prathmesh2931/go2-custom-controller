#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unordered_map>
#include <cmath>

class TrotGait : public rclcpp::Node
{
public:
    TrotGait() : Node("trot_gait")
    {
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrotGait::cb_js, this, std::placeholders::_1));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TrotGait::cb_cmd, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&TrotGait::cb_imu, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/ground_truth", 10,
            std::bind(&TrotGait::cb_odom, this, std::placeholders::_1));

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_group_effort_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TrotGait::tick, this));

        RCLCPP_INFO(get_logger(), "TrotGait with normalized leg workspace ready.");
    }

private:
    //  Robot geometry ─
    static constexpr double HIP_OFFSET = 0.0955;
    static constexpr double THIGH_LEN  = 0.213;
    static constexpr double CALF_LEN   = 0.213;
    static constexpr double LEG_X_DIST = 0.1934; 

    //  Gait limits 
    static constexpr double STANCE_Z     = -0.28;
    static constexpr double LIFT_H       =  0.04;
    static constexpr double MAX_STEP_LEN =  0.12; // Lowered slightly to 0.12m for a safe kinematic margin
    static constexpr double MIN_CADENCE  =  0.6;
    static constexpr double MAX_CADENCE  =  3.0;
    static constexpr double CADENCE_GAIN =  3.0;
    static constexpr double SWING_DUTY   =  0.40;
    static constexpr double K_ROLL       =  0.08;
    static constexpr double K_PITCH      =  0.06;
    static constexpr double STEP_FF_GAIN =  0.14;

    //  Velocity feedback PI gains 
    static constexpr double VEL_KP       =  0.05;
    static constexpr double VEL_KI       =  0.008;
    static constexpr double VEL_I_MAX    =  0.025;

    //  Lateral drift correction gain 
    static constexpr double DRIFT_KP     =  0.3;

    //  Command smoothing 
    static constexpr double CMD_ALPHA    =  0.08;

    //  Deadbands 
    static constexpr double CMD_DEADBAND     = 0.05;   
    static constexpr double YAW_DEADBAND     = 0.05;   

    //  Leg layout 
    static constexpr bool   PAIR_A[4] = {true,  false, false, true };  // FL + RR
    static constexpr bool   PAIR_B[4] = {false, true,  true,  false};  // FR + RL
    static constexpr double Y_OFF[4]  = {+HIP_OFFSET, -HIP_OFFSET,
                                          +HIP_OFFSET, -HIP_OFFSET};

    //  State 
    double phase_     = 0.0;
    int    last_half_ = -1;
    bool   ready_     = false;

    double smooth_vx_  = 0.0;
    double smooth_vy_  = 0.0;
    double smooth_yaw_ = 0.0;

    double raw_vx_  = 0.0;
    double raw_vy_  = 0.0;
    double raw_yaw_ = 0.0;

    double odom_vx_ = 0.0;
    double odom_vy_ = 0.0;

    double vel_i_x_ = 0.0;

    double imu_roll_  = 0.0;
    double imu_pitch_ = 0.0;

    std::unordered_map<std::string, double> js_;

    struct Angles { double hip, thigh, calf; };

    Angles ik(double x, double y, double z)
    {
        double r2d = std::sqrt(y*y + z*z);
        double hip_sign = (y >= 0.0) ? 1.0 : -1.0;
        double hip = std::atan2(y, -z)
                   - hip_sign * std::atan2(HIP_OFFSET,
                                std::sqrt(std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET)));

        double z2d   = -std::sqrt(std::max(0.0, y*y + z*z - HIP_OFFSET*HIP_OFFSET));
        double D_raw = (x*x + z2d*z2d - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN)
                       / (2.0 * THIGH_LEN * CALF_LEN);

        double D     = std::clamp(D_raw, -1.0, 1.0);
        double calf  = -std::acos(D);
        double thigh = std::atan2(-x, -z2d)
                     - std::atan2(CALF_LEN * std::sin(calf),
                                  THIGH_LEN + CALF_LEN * std::cos(calf));
        return {hip, thigh, calf};
    }

    std::pair<double,double> swing_pos(double t, double step_len)
    {
        double pos = -step_len/2.0 + step_len * (0.5 - 0.5*std::cos(M_PI * t));
        double z   = STANCE_Z + LIFT_H * std::sin(M_PI * t);
        return {pos, z};
    }

    double stance_pos(double t, double step_len)
    {
        return step_len/2.0 - step_len * t;
    }

    void publish(const std::vector<double>& pos)
    {
        trajectory_msgs::msg::JointTrajectory msg;

        // Exact names confirmed working from Go2 URDF configuration
        msg.joint_names = {
            "lf_hip_joint",  "lf_upper_leg_joint",  "lf_lower_leg_joint",
            "rf_hip_joint",  "rf_upper_leg_joint",  "rf_lower_leg_joint",
            "lh_hip_joint",  "lh_upper_leg_joint",  "lh_lower_leg_joint",
            "rh_hip_joint",  "rh_upper_leg_joint",  "rh_lower_leg_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = pos;
        pt.time_from_start.nanosec = 20'000'000;
        msg.points.push_back(pt);
        pub_->publish(msg);
    }

    void cb_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
            js_[msg->name[i]] = msg->position[i];
        ready_ = true;
    }

    void cb_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        raw_vx_  = msg->linear.x;
        raw_vy_  = msg->linear.y;
        raw_yaw_ = msg->angular.z;
    }

    void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto& q    = msg->orientation;
        imu_roll_  =  std::atan2(2.0*(q.w*q.x + q.y*q.z),
                                  1.0 - 2.0*(q.x*q.x + q.y*q.y));
        imu_pitch_ = -std::asin(std::clamp(2.0*(q.w*q.y - q.z*q.x), -1.0, 1.0));
    }

    void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_vx_ = msg->twist.twist.linear.x;
        odom_vy_ = msg->twist.twist.linear.y;
    }

    void tick()
    {
        if (!ready_) return;

        // 1. Smooth commands
        smooth_vx_  += CMD_ALPHA * (raw_vx_  - smooth_vx_);
        smooth_vy_  += CMD_ALPHA * (raw_vy_  - smooth_vy_);
        smooth_yaw_ += CMD_ALPHA * (raw_yaw_ - smooth_yaw_);

        double vx_error = smooth_vx_ - odom_vx_;
        double vy_error = smooth_vy_ - odom_vy_;

        bool has_vx  = std::abs(smooth_vx_)  > CMD_DEADBAND;
        bool has_vy  = std::abs(smooth_vy_)  > CMD_DEADBAND;
        bool has_yaw = std::abs(smooth_yaw_) > YAW_DEADBAND;
        bool has_command = has_vx || has_vy || has_yaw;

        // 2. Longitudinal step sizing
        if (has_vx) {
            vel_i_x_ = std::clamp(vel_i_x_ + VEL_KI * vx_error * 0.020,
                                  -VEL_I_MAX, VEL_I_MAX);
        } else {
            vel_i_x_ *= 0.95;   
        }

        double step_ff = std::clamp(STEP_FF_GAIN * smooth_vx_, -MAX_STEP_LEN, MAX_STEP_LEN);
        double p_term  = has_vx ? (VEL_KP * vx_error) : 0.0;
        double step_len_x = std::clamp(step_ff + p_term + vel_i_x_, -MAX_STEP_LEN, MAX_STEP_LEN);

        if (!has_command) {
            step_len_x = 0.0;
        }

        double step_len_y_base = STEP_FF_GAIN * smooth_vy_;
        if (!has_yaw && has_vy) {
            step_len_y_base += DRIFT_KP * vy_error; 
        }

        //  FIX #1: Inverted Yaw signs to fix Left/Right orientation behavior 
        // Changing signs here swaps inner/outer rotation assignments cleanly.
        double yaw_x_stride = -0.05 * smooth_yaw_; 
        double yaw_y_stride = -0.05 * smooth_yaw_ * (LEG_X_DIST / HIP_OFFSET); 

        double leg_step_x[4];
        double leg_step_y[4];

        // Raw calculations combining linear footprint with turning offsets
        leg_step_x[0] = step_len_x - yaw_x_stride; // FL
        leg_step_y[0] = step_len_y_base - yaw_y_stride;

        leg_step_x[1] = step_len_x + yaw_x_stride; // FR
        leg_step_y[1] = step_len_y_base - yaw_y_stride;

        leg_step_x[2] = step_len_x - yaw_x_stride; // RL
        leg_step_y[2] = step_len_y_base + yaw_y_stride;

        leg_step_x[3] = step_len_x + yaw_x_stride; // RR
        leg_step_y[3] = step_len_y_base + yaw_y_stride;

        //  FIX #2: Vector Ellipse Normalization (Workspace Protection) 
        // This stops individual legs from exploding when mixing X + Yaw velocities together
        for (int i = 0; i < 4; ++i) {
            double combined_stride = std::sqrt(leg_step_x[i]*leg_step_x[i] + leg_step_y[i]*leg_step_y[i]);
            if (combined_stride > MAX_STEP_LEN) {
                double scale = MAX_STEP_LEN / combined_stride;
                leg_step_x[i] *= scale;
                leg_step_y[i] *= scale;
            }
        }

        // 3. Phase management
        double phase_inc = 0.0;
        if (has_command) {
            double speed = std::abs(smooth_vx_) + std::abs(smooth_vy_) + 0.4 * std::abs(smooth_yaw_);
            double cadence = std::clamp(MIN_CADENCE + CADENCE_GAIN * speed, MIN_CADENCE, MAX_CADENCE);
            phase_inc = 0.020 * cadence;
        }

        phase_ = std::fmod(phase_ + phase_inc, 1.0);
        int half = (phase_ < 0.5) ? 0 : 1;

        double half_phase = (half == 0) ? (phase_ / 0.5) : ((phase_ - 0.5) / 0.5);
        bool   in_swing   = (half_phase < SWING_DUTY);
        double swing_t    = in_swing ? (half_phase / SWING_DUTY) : 1.0;
        double stance_t   = half_phase;

        if (last_half_ != half) last_half_ = half;
        const bool* swing_mask = (half == 0) ? PAIR_A : PAIR_B;

        // 4. IMU stabilization
        double z_corr[4];
        z_corr[0] =  K_ROLL * imu_roll_ - K_PITCH * imu_pitch_;  // FL
        z_corr[1] = -K_ROLL * imu_roll_ - K_PITCH * imu_pitch_;  // FR
        z_corr[2] =  K_ROLL * imu_roll_ + K_PITCH * imu_pitch_;  // RL
        z_corr[3] = -K_ROLL * imu_roll_ + K_PITCH * imu_pitch_;  // RR

        // 5. Compute Cartesian paths & Inverse Kinematics
        std::vector<double> pos(12);
        for (int i = 0; i < 4; ++i) {
            double xi, yi, zi;

            if (swing_mask[i]) {
                auto [sx, sz] = swing_pos(swing_t, leg_step_x[i]);
                xi = sx;
                yi = Y_OFF[i] + (-leg_step_y[i]/2.0 + leg_step_y[i] * (0.5 - 0.5*std::cos(M_PI * swing_t)));
                zi = sz + z_corr[i];
            } else {
                xi = stance_pos(stance_t, leg_step_x[i]);
                yi = Y_OFF[i] + stance_pos(stance_t, leg_step_y[i]);
                zi = STANCE_Z + z_corr[i];
            }

            zi = std::clamp(zi, STANCE_Z - 0.05, STANCE_Z + LIFT_H + 0.01);

            Angles a = ik(xi, yi, zi);
            pos[i*3 + 0] = a.hip;
            pos[i*3 + 1] = a.thigh;
            pos[i*3 + 2] = a.calf;
        }

        publish(pos);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrotGait>());
    rclcpp::shutdown();
    return 0;
}