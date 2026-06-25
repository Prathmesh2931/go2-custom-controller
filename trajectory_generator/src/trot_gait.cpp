#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cmath>
#include <algorithm>

class TrotGait : public rclcpp::Node
{
public:
    TrotGait() : Node("trot_gait")
    {
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TrotGait::cb_cmd, this, std::placeholders::_1));

        pub_nominal_positions_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gait/planned_positions", 10);
        pub_contact_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/state_estimator/contact_states", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TrotGait::tick, this));
        
        RCLCPP_INFO(get_logger(), "ADVANCED TROT GAIT ONLINE: IDLE OVERRIDE, YAW KINEMATICS, & WIDE STANCE ACTIVE.");
    }

private:
    const double DIR_MULTIPLIER = 1.0; 

    // Mechanical Constants
    const double HIP_X      = 0.1934; 
    const double HIP_OFFSET = 0.0955; // Used internally for IK math
    const double THIGH_LEN  = 0.213;
    const double CALF_LEN   = 0.213;

    // Gait Parameters
    const double HIP_Y      = 0.14;   // The Wide Stance target!
    const double STANCE_Z   = -0.326;  // Matches the MPC height target perfectly
    const double LIFT_H     = 0.08;   // Swing clearance

    double phase_ = 0.0;
    double raw_vx_ = 0.0;
    double raw_wz_ = 0.0; // Added Yaw velocity
    int log_counter_ = 0;

    struct Angles { double hip, thigh, calf; };

    Angles ik(double x, double y, double z)
    {
        Angles a;
        double r2d = std::sqrt(y*y + z*z);
        if (r2d < HIP_OFFSET) r2d = HIP_OFFSET + 0.001; 
        double hip_sign = (y >= 0.0) ? 1.0 : -1.0;
        
        double inner = std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET);
        // Hip Roll calculates the angle needed to reach the Wide Stance Y target
        a.hip = std::atan2(y, -z) - hip_sign * std::atan2(HIP_OFFSET, std::sqrt(inner));

        double L2 = std::sqrt(inner);
        double leg_distance = std::sqrt(x*x + L2*L2);
        leg_distance = std::clamp(leg_distance, 0.01, THIGH_LEN + CALF_LEN - 0.001);
        
        double cos_calf = (leg_distance*leg_distance - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * CALF_LEN);
        a.calf = -std::acos(std::clamp(cos_calf, -1.0, 1.0)); 

        double alpha = std::atan2(x, L2);
        double cos_beta = (THIGH_LEN*THIGH_LEN + leg_distance*leg_distance - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * leg_distance);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        a.thigh = alpha + beta;

        return a;
    }

    void cb_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
        raw_vx_ = msg->linear.x * DIR_MULTIPLIER; 
        raw_wz_ = msg->angular.z * DIR_MULTIPLIER;
    }

    void tick()
    {
        double vx = raw_vx_;
        double wz = raw_wz_;
        double speed = std::sqrt(vx*vx + wz*wz);
        bool is_moving = speed > 0.05;

        std::vector<double> nominal_positions(12, 0.0);
        std::vector<int> live_contacts(4, 1);
        std::vector<double> current_X(4, 0.0);
        std::vector<double> current_Y(4, 0.0);
        std::vector<double> current_Z(4, 0.0);

        if (is_moving) {
            phase_ += 0.02 * 1.5; // Cadence
            if (phase_ >= 1.0) phase_ -= 1.0;
        } else {
            phase_ = 0.0; // 4-LEG IDLE OVERRIDE
        }

        for (int i = 0; i < 4; ++i) {
            // Determine mechanical positions relative to CoM
            double rx = (i < 2) ? HIP_X : -HIP_X;
            double base_y = (i % 2 == 0) ? HIP_Y : -HIP_Y; // The target wide stance
            
            double local_x = 0.0;
            double local_y = base_y;
            double local_z = STANCE_Z;

            if (is_moving) {
                // DIFFERENTIAL KINEMATICS FOR YAW
                double foot_vx = vx - wz * base_y;
                double foot_vy = wz * rx;
                
                double step_x = std::clamp(foot_vx * 0.4, -0.15, 0.15);
                double step_y = std::clamp(foot_vy * 0.4, -0.08, 0.08);
                
                double p = (i == 0 || i == 3) ? phase_ : std::fmod(phase_ + 0.5, 1.0);
                
                if (p < 0.4) { 
                    // SWING PHASE
                    live_contacts[i] = 0; 
                    double t = p / 0.4;
                    local_x = -step_x/2.0 + step_x * (0.5 - 0.5*std::cos(M_PI * t));
                    local_y = base_y - step_y/2.0 + step_y * (0.5 - 0.5*std::cos(M_PI * t));
                    local_z = STANCE_Z + LIFT_H * std::sin(M_PI * t); 
                } else { 
                    // STANCE PHASE
                    live_contacts[i] = 1; 
                    double t = (p - 0.4) / 0.6;
                    local_x = step_x/2.0 - step_x * t; 
                    local_y = base_y + step_y/2.0 - step_y * t;
                    local_z = STANCE_Z; 
                }
            } else {
                // IDLE: All 4 feet locked to the ground
                live_contacts[i] = 1;
            }

            // Save for diagnostics
            current_X[i] = local_x;
            current_Y[i] = local_y;
            current_Z[i] = local_z;

            // Solve Inverse Kinematics
            Angles a = ik(local_x, local_y, local_z);
            nominal_positions[i*3 + 0] = a.hip;
            nominal_positions[i*3 + 1] = a.thigh;
            nominal_positions[i*3 + 2] = a.calf;
        }

        if (++log_counter_ >= 50) { // Log every 1 second
            log_counter_ = 0;
            RCLCPP_INFO(get_logger(), "\n--- GAIT PLANNER DIAGNOSTICS ---");
            RCLCPP_INFO(get_logger(), "CMD VX: %.3f | CMD WZ: %.3f | PHASE: %.2f | MOVING: %d", vx, wz, phase_, is_moving);
            RCLCPP_INFO(get_logger(), "LF [C:%d] X=%.3f, Y=%.3f, Z=%.3f | HIP=%.3f, TH=%.3f, CA=%.3f", 
                live_contacts[0], current_X[0], current_Y[0], current_Z[0], nominal_positions[0], nominal_positions[1], nominal_positions[2]);
        }

        std_msgs::msg::Int32MultiArray contact_msg; 
        contact_msg.data = live_contacts;
        pub_contact_->publish(contact_msg);

        std_msgs::msg::Float64MultiArray nominal_payload; 
        nominal_payload.data = nominal_positions;
        pub_nominal_positions_->publish(nominal_payload);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_nominal_positions_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_contact_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TrotGait>()); rclcpp::shutdown(); return 0;
}