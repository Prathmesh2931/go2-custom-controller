#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_group_effort_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TrotGait::tick, this));

        RCLCPP_INFO(get_logger(), "TrotGait ready.");
    }

private:
    //  Robot geometry 
    static constexpr double HIP_OFFSET = 0.0955;
    static constexpr double THIGH_LEN  = 0.213;
    static constexpr double CALF_LEN   = 0.213;

    //  Gait limits 
    static constexpr double STANCE_Z      = -0.30;  // nominal foot height (negative = below hip)
    static constexpr double LIFT_H        =  0.04;  // swing clearance (m)
    static constexpr double MAX_STEP_LEN  =  0.10;  // per leg, per half-cycle (m)
    static constexpr double MIN_CADENCE   =  0.6;   // Hz when barely moving
    static constexpr double MAX_CADENCE   =  2.5;   // Hz at full speed
    static constexpr double CADENCE_GAIN  =  2.0;   // Hz per (m/s)

    // Duty factor: fraction of cycle spent in SWING (0.4 = 40% swing, 60% stance)
    // Lowering this makes the gait more stable (more time on ground)
    static constexpr double SWING_DUTY   = 0.40;

    // IMU stabilisation gains
    static constexpr double K_ROLL  = 0.08;  // m of z correction per rad of roll
    static constexpr double K_PITCH = 0.06;  // m of z correction per rad of pitch

    // Low-pass filter coefficient for cmd_vel smoothing (0 = frozen, 1 = instant)
    static constexpr double CMD_ALPHA = 0.08;

    //  Leg layout 
    // Index: 0=FL, 1=FR, 2=RL, 3=RR
    static constexpr bool PAIR_A[4] = {true,  false, false, true };  // FL+RR swing
    static constexpr bool PAIR_B[4] = {false, true,  true,  false};  // FR+RL swing
    static constexpr double Y_OFF[4] = {+HIP_OFFSET, -HIP_OFFSET, +HIP_OFFSET, -HIP_OFFSET};

    //  State 
    double phase_    = 0.0;
    int    last_half_ = -1;
    bool   ready_    = false;

    // Smoothed command values (updated each tick via low-pass filter)
    double smooth_vx_  = 0.0;
    double smooth_vy_  = 0.0;
    double smooth_yaw_ = 0.0;

    // Raw command values (written by cb_cmd, read by tick)
    double raw_vx_  = 0.0;
    double raw_vy_  = 0.0;
    double raw_yaw_ = 0.0;

    // IMU state
    double imu_roll_  = 0.0;
    double imu_pitch_ = 0.0;

    std::unordered_map<std::string, double> js_;

    //  IK 
    struct Angles { double hip, thigh, calf; };

    Angles ik(double x, double y, double z)
    {
        double r2d = std::sqrt(y*y + z*z);
        double hip = std::atan2(y, -z)
                   - std::atan2(HIP_OFFSET,
                                std::sqrt(std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET)));

        double z2d = -std::sqrt(std::max(0.0, y*y + z*z - HIP_OFFSET*HIP_OFFSET));

        double D_raw = (x*x + z2d*z2d - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN)
                       / (2.0 * THIGH_LEN * CALF_LEN);

        if (std::abs(D_raw) > 0.96)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "IK near singularity D=%.3f x=%.3f z2d=%.3f", D_raw, x, z2d);

        double D    = std::clamp(D_raw, -1.0, 1.0);
        double calf = -std::acos(D);
        double thigh = std::atan2(-x, -z2d)
                     - std::atan2(CALF_LEN * std::sin(calf),
                                  THIGH_LEN + CALF_LEN * std::cos(calf));

        return {hip, thigh, calf};
    }

    //  Trajectories 

    // t in [0,1] over the SWING portion of the half-cycle
    // step_len: signed (positive = forward)
    std::pair<double,double> swing_pos(double t, double step_len)
    {
        double x = -step_len/2.0 + step_len * t;
        double z = STANCE_Z + LIFT_H * std::sin(M_PI * t);
        return {x, z};
    }

    // t in [0,1] over the STANCE portion of the half-cycle
    double stance_x(double t, double step_len)
    {
        return step_len/2.0 - step_len * t;
    }

    //  Publish 
    void publish(const std::vector<double>& pos)
    {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = pos;
        pt.time_from_start.nanosec = 20'000'000;
        msg.points.push_back(pt);
        pub_->publish(msg);
    }

    //  Callbacks 
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
        // Convert quaternion → roll/pitch (ZYX Euler)
        // We only need small-angle approximation for stabilisation
        auto& q = msg->orientation;
        imu_roll_  =  std::atan2(2.0*(q.w*q.x + q.y*q.z),
                                  1.0 - 2.0*(q.x*q.x + q.y*q.y));
        imu_pitch_ = -std::asin(std::clamp(2.0*(q.w*q.y - q.z*q.x), -1.0, 1.0));
    }

    //  Main control tick 
    void tick()
    {
        if (!ready_) return;

        //  1. Smooth commands (low-pass filter) 
        smooth_vx_  += CMD_ALPHA * (raw_vx_  - smooth_vx_);
        smooth_vy_  += CMD_ALPHA * (raw_vy_  - smooth_vy_);
        smooth_yaw_ += CMD_ALPHA * (raw_yaw_ - smooth_yaw_);

        //  2. Derive cadence and step length from smoothed velocity 
        double speed    = std::abs(smooth_vx_) + std::abs(smooth_vy_);
        double cadence  = std::clamp(MIN_CADENCE + CADENCE_GAIN * speed,
                                     MIN_CADENCE, MAX_CADENCE);
        double phase_inc = 0.020 * cadence;  // recomputed every tick

        // Signed step length: positive = forward foot travels forward
        double step_len = std::clamp(0.12 * smooth_vx_, -MAX_STEP_LEN, MAX_STEP_LEN);
        // Lateral step offset (strafing): shifts foot y inward/outward
        double lat_off  = std::clamp(0.10 * smooth_vy_, -0.04, 0.04);

        //  3. Yaw: per-side step length differential 
        // Left legs (FL=0, RL=2) get +yaw contribution, right legs (FR=1, RR=3) get -yaw
        // This curves the robot without breaking the diagonal gait pairing.
        double yaw_delta = std::clamp(0.05 * smooth_yaw_, -0.04, 0.04);
        // step_len_per_leg[i] = base step ± yaw contribution
        double leg_step[4];
        leg_step[0] = step_len - yaw_delta;  // FL
        leg_step[1] = step_len + yaw_delta;  // FR
        leg_step[2] = step_len - yaw_delta;  // RL
        leg_step[3] = step_len + yaw_delta;  // RR

        //  4. Advance phase 
        phase_ = std::fmod(phase_ + phase_inc, 1.0);
        int half = (phase_ < 0.5) ? 0 : 1;

        //  5. Compute local t within current half-cycle 
        // With duty factor: first SWING_DUTY fraction of the half is swing,
        // remaining (1-SWING_DUTY) fraction is an extended stance "dwell".
        // Both swing_t and stance_t remain in [0,1].
        double half_phase = (half == 0) ? (phase_ / 0.5)
                                        : ((phase_ - 0.5) / 0.5);
        // half_phase ∈ [0,1) within this half-cycle

        bool   in_swing  = (half_phase < SWING_DUTY);
        double swing_t   = in_swing ? (half_phase / SWING_DUTY) : 1.0;
        double stance_t_dwell = in_swing ? 0.0
                                         : ((half_phase - SWING_DUTY) / (1.0 - SWING_DUTY));
        // For stance legs: full half-cycle traversal still [0,1]
        double stance_t  = half_phase;   // continuous 0→1 across full half-cycle

        //  6. Half-cycle transition: swap foot positions 
        if (last_half_ != half) {
            for (int i = 0; i < 4; ++i) {
                bool was_swinging = (last_half_ == 0) ? PAIR_A[i]
                                  : (last_half_ == 1) ? PAIR_B[i]
                                  : false;
                // Landed leg starts at front; stance legs reset to front too
                // (they will travel back during this half-cycle)
                (void)was_swinging;  // foot_x_ not used directly — trajectories are analytical
            }
            last_half_ = half;
        }

        const bool* swing_mask = (half == 0) ? PAIR_A : PAIR_B;

        //  7. IMU stabilisation: z offsets per leg 
        // Roll: body tilts right (+roll) → lower left legs, raise right legs
        // Pitch: body pitches forward (+pitch) → lower front legs, raise rear legs
        double z_corr[4];
        z_corr[0] = -K_ROLL * imu_roll_ - K_PITCH * imu_pitch_;  // FL
        z_corr[1] =  K_ROLL * imu_roll_ - K_PITCH * imu_pitch_;  // FR
        z_corr[2] = -K_ROLL * imu_roll_ + K_PITCH * imu_pitch_;  // RL
        z_corr[3] =  K_ROLL * imu_roll_ + K_PITCH * imu_pitch_;  // RR

        //  8. Compute IK for all legs 
        std::vector<double> pos(12);
        for (int i = 0; i < 4; ++i) {
            double xi, zi;
            double yi = Y_OFF[i] + lat_off * ((i == 0 || i == 2) ? 1.0 : -1.0);
            // lat_off shifts both legs on the same side outward for strafing

            if (swing_mask[i]) {
                // Swing leg: arc trajectory (only during swing portion of half-cycle)
                auto [sx, sz] = swing_pos(swing_t, leg_step[i]);
                xi = sx;
                zi = sz + z_corr[i];
            } else {
                // Stance leg: push backward continuously
                xi = stance_x(stance_t, leg_step[i]);
                zi = STANCE_Z + z_corr[i];
            }

            // Hard-clamp z so we never ask for an impossible leg extension
            zi = std::clamp(zi, STANCE_Z - 0.05, STANCE_Z + LIFT_H + 0.01);

            Angles a = ik(xi, yi, zi);
            pos[i*3 + 0] = a.hip;
            pos[i*3 + 1] = a.thigh;
            pos[i*3 + 2] = a.calf;
        }

        publish(pos);
    }

    //  ROS handles 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_imu_;
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