#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_group_effort_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TrotGait::tick, this));

        RCLCPP_INFO(get_logger(), "TrotGait started. Waiting for /joint_states...");
    }

private:
    //  Link lengths from URDF 
    static constexpr double HIP_OFFSET = 0.0955;   // L0: hip abduction distance (m)
    static constexpr double THIGH_LEN  = 0.213;    // L1
    static constexpr double CALF_LEN   = 0.213;    // L2

    //  Gait tuning 
    // Start conservative. Once stable, increase STEP_LEN and CADENCE_HZ.
    static constexpr double STANCE_Z   = -0.30;    // foot z below hip at stance (NEGATIVE = below)
    static constexpr double LIFT_H     =  0.03;    // peak clearance above stance plane (m)
    static constexpr double STEP_LEN   =  0.03;    // stride length (m) — keep small at first
    static constexpr double CADENCE_HZ =  0.5;     // full cycles per second (one cycle = A+B)

    // Legs indexed: 0=FL, 1=FR, 2=RL, 3=RR
    // Diagonal pairs: {FL,RR} and {FR,RL}
    // PAIR_A[i]=true means leg i is in the SWING phase during half-cycle A
    static constexpr bool PAIR_A[4] = {true, false, false, true};  // FL + RR swing in A
    static constexpr bool PAIR_B[4] = {false, true, true, false};  // FR + RL swing in B

    // Hip lateral offsets: positive = left of body, negative = right
    // Order: FL, FR, RL, RR
    static constexpr double Y_OFF[4] = {
        +HIP_OFFSET,   // FL: left
        -HIP_OFFSET,   // FR: right
        +HIP_OFFSET,   // RL: left
        -HIP_OFFSET    // RR: right
    };

    //  State 
    // Phase within [0, 1]: 0→0.5 = half-cycle A, 0.5→1.0 = half-cycle B
    double phase_     = 0.0;
    double phase_inc_ = 0.0;   // computed once joint_states arrive

    // foot_x[i]: current x position of foot i relative to hip (forward = positive)
    // All start at 0 (foot directly under hip)
    double foot_x_[4] = {0.0, 0.0, 0.0, 0.0};

    // Track which half-cycle we were in last tick to detect transitions
    int last_half_ = -1;

    std::unordered_map<std::string, double> js_;

    //  IK 
    // Input:  x = forward, y = lateral (+ left), z = vertical (NEGATIVE = below hip)
    // Output: hip(abduction), thigh(pitch), calf(pitch) in radians
    struct Angles { double hip, thigh, calf; };

    Angles ik(double x, double y, double z)
    {
        // Abduction (roll) angle
        // z is negative (below hip), so -z is positive → atan2(y, -z) gives correct sign
        double r2d  = std::sqrt(y*y + z*z);
        double hip  = std::atan2(y, -z)
                    - std::atan2(HIP_OFFSET, std::sqrt(std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET)));

        // Sagittal leg length after abduction
        double z2d  = -std::sqrt(std::max(0.0, y*y + z*z - HIP_OFFSET*HIP_OFFSET));
        // z2d is negative: the foot is below the thigh origin

        // Elbow (two-link planar IK)
        // D is cosine of knee angle
        double D_raw = (x*x + z2d*z2d - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN)
                       / (2.0 * THIGH_LEN * CALF_LEN);

        if (std::abs(D_raw) > 0.96) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 400,
                "IK singularity: D=%.3f x=%.3f z2d=%.3f — reduce STANCE_Z or LIFT_H", D_raw, x, z2d);
        }
        double D    = std::clamp(D_raw, -1.0, 1.0);
        double calf = -std::acos(D);   // negative = knee bent forward (Go2 convention)

        double thigh = std::atan2(-x, -z2d)
                     - std::atan2(CALF_LEN * std::sin(calf),
                                  THIGH_LEN + CALF_LEN * std::cos(calf));

        return {hip, thigh, calf};
    }

    //  Swing trajectory 
    // t in [0,1]. Returns {x_delta, z} for foot position.
    // x moves from -STEP_LEN/2 to +STEP_LEN/2 (symmetric about hip)
    // z rises as a half-sine arc
    std::pair<double,double> swing_pos(double t)
    {
        // x: linear from back to front
        double x = -STEP_LEN/2.0 + STEP_LEN * t;
        // z: parabolic lift (half-sine looks best)
        double z = STANCE_Z + LIFT_H * std::sin(M_PI * t);
        return {x, z};
    }

    //  Stance trajectory 
    // Stance foot moves backward at the same rate the swing foot moves forward
    // so the body translates forward.
    double stance_x(double t)
    {
        // Foot starts at +STEP_LEN/2 and moves to -STEP_LEN/2 over the half-cycle
        return STEP_LEN/2.0 - STEP_LEN * t;
    }

    //  Publish 
    void publish(const std::vector<double>& pos)
    {
        trajectory_msgs::msg::JointTrajectory msg;
        // *** JOINT NAMES MUST MATCH YOUR URDF EXACTLY ***
        // Order: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
        //        RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
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

    void cb_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
            js_[msg->name[i]] = msg->position[i];

        // Set phase increment once we know the robot is alive
        if (phase_inc_ == 0.0) {
            // 20 ms ticks, full cycle = 1/CADENCE_HZ seconds
            // phase advances by (20ms / period_ms) per tick
            phase_inc_ = 0.020 * CADENCE_HZ;
            RCLCPP_INFO(get_logger(), "Joint states received. Starting trot. phase_inc=%.4f", phase_inc_);
        }
    }

    //  Main loop 
    void tick()
    {
        if (js_.empty() || phase_inc_ == 0.0) return;

        // Advance phase
        phase_ = std::fmod(phase_ + phase_inc_, 1.0);

        // Determine which half-cycle we're in
        // Half A: phase in [0, 0.5)  → FL+RR swing, FR+RL stance
        // Half B: phase in [0.5, 1.0) → FR+RL swing, FL+RR stance
        int half = (phase_ < 0.5) ? 0 : 1;

        // Normalised time within this half-cycle [0, 1)
        double t = (half == 0) ? (phase_ / 0.5) : ((phase_ - 0.5) / 0.5);

        // On half-cycle transition, reset foot_x to the start of the new cycle.
        // This prevents accumulated drift. The swing foot was placed at +STEP_LEN/2
        // and the stance feet ended at -STEP_LEN/2; swap them.
        if (last_half_ != half) {
            for (int i = 0; i < 4; ++i) {
                bool was_swinging = (last_half_ == 0) ? PAIR_A[i] : PAIR_B[i];
                // swinging leg just landed at front; stance legs are at back
                foot_x_[i] = was_swinging ? STEP_LEN/2.0 : -STEP_LEN/2.0;
            }
            last_half_ = half;
        }

        const bool* swing_mask = (half == 0) ? PAIR_A : PAIR_B;

        std::vector<double> pos(12);
        for (int i = 0; i < 4; ++i) {
            double xi, zi;
            if (swing_mask[i]) {
                auto [sx, sz] = swing_pos(t);
                xi = sx;
                zi = sz;
            } else {
                xi = stance_x(t);
                zi = STANCE_Z;
            }
            Angles a = ik(xi, Y_OFF[i], zi);
            pos[i*3 + 0] = a.hip;
            pos[i*3 + 1] = a.thigh;
            pos[i*3 + 2] = a.calf;
        }

        publish(pos);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
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