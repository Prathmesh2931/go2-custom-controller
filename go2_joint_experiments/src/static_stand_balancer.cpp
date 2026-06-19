// go2_stand_controller.cpp
// Sign-Corrected Noise-Damped Posture Engine for Unitree Go2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <iostream>
#include <iomanip>

class Go2StandController : public rclcpp::Node
{
public:
    Go2StandController() : Node("go2_stand_controller")
    {
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&Go2StandController::js_callback, this, std::placeholders::_1));

        pub_effort_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_effort_controller/commands", 10);

        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };

        leg_prefixes_ = {"lf", "rf", "lh", "rh"};
        tau_prev_.resize(12, 0.0);

        RCLCPP_INFO(get_logger(), "Sign-Corrected Posture Engine Active.");
    }

private:
    static constexpr double Q_STAND_HIP       =  0.00;
    static constexpr double Q_STAND_UPPER_LEG =  0.67;
    static constexpr double Q_STAND_LOWER_LEG = -1.30; 

    // *** BALANCED GAINS: High Kp stiffness for alignment, lowered Kd to suppress noise loops ***
    static constexpr double KP_HIP   = 85.0; 
    static constexpr double KD_HIP   =  2.5; // Lowered to prevent derivative chattering
    static constexpr double KP_LEG   = 90.0;
    static constexpr double KD_LEG   =  3.5; // Lowered to smooth out extension paths

    static constexpr double TORQUE_FILTER_ALPHA = 0.20; // Increased filtering to block transient ripples
    static constexpr double TRAJ_DURATION       = 6.0; 

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_effort_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> leg_prefixes_;
    std::vector<double> tau_prev_;
    bool first_tick_ = true;
    std::unordered_map<std::string, double> q_start_;
    rclcpp::Time start_time_;
    int log_counter_ = 0;

    void js_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::unordered_map<std::string, double> q_map, v_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            q_map[msg->name[i]] = msg->position[i];
            if (i < msg->velocity.size()) v_map[msg->name[i]] = msg->velocity[i];
        }

        for (const auto& name : joint_names_) {
            if (q_map.find(name) == q_map.end()) return;
        }

        if (first_tick_) {
            q_start_ = q_map;
            start_time_ = this->now();
            first_tick_ = false;
            return;
        }

        double elapsed = (this->now() - start_time_).seconds();
        double t = std::clamp(elapsed / TRAJ_DURATION, 0.0, 1.0);
        double alpha = t * t * (3.0 - 2.0 * t); 

        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(12, 0.0);

        for (int i = 0; i < 4; ++i) {
            int idx = i * 3;
            const std::string hip_n   = joint_names_[idx + 0];
            const std::string thigh_n = joint_names_[idx + 1];
            const std::string calf_n  = joint_names_[idx + 2];

            double q_h = q_map.at(hip_n);
            double q_t = q_map.at(thigh_n);
            double q_c = q_map.at(calf_n);
            
            double v_h = v_map.count(hip_n)   ? v_map.at(hip_n)   : 0.0;
            double v_t = v_map.count(thigh_n) ? v_map.at(thigh_n) : 0.0;
            double v_c = v_map.count(calf_n)  ? v_map.at(calf_n)  : 0.0;

            double qd_h = q_start_.at(hip_n)   + (Q_STAND_HIP       - q_start_.at(hip_n))   * alpha;
            double qd_t = q_start_.at(thigh_n) + (Q_STAND_UPPER_LEG - q_start_.at(thigh_n)) * alpha;
            double qd_c = q_start_.at(calf_n)  + (Q_STAND_LOWER_LEG - q_start_.at(calf_n))  * alpha;

            // *** CRITICAL CORRECTION: Mirrored Hip Sign Remapping ***
            // Left legs (i=0,2) and Right legs (i=1,3) use inverted sign references 
            // to pull inward toward the centerline together.
            double hip_sign_multiplier = (i == 0 || i == 2) ? 1.0 : -1.0;

            double tau_raw_h = (KP_HIP * (qd_h - q_h) - KD_HIP * v_h) * hip_sign_multiplier;
            double tau_raw_t = KP_LEG * (qd_t - q_t) - KD_LEG * v_t;
            double tau_raw_c = KP_LEG * (qd_c - q_c) - KD_LEG * v_c;

            // Smooth high-frequency transients across the 4ms sampling window
            tau_prev_[idx + 0] = tau_prev_[idx + 0] + TORQUE_FILTER_ALPHA * (tau_raw_h - tau_prev_[idx + 0]);
            tau_prev_[idx + 1] = tau_prev_[idx + 1] + TORQUE_FILTER_ALPHA * (tau_raw_t - tau_prev_[idx + 1]);
            tau_prev_[idx + 2] = tau_prev_[idx + 2] + TORQUE_FILTER_ALPHA * (tau_raw_c - tau_prev_[idx + 2]);

            double limit = (idx % 3 == 2) ? 45.43 : 23.70;
            effort_msg.data[idx + 0] = std::clamp(tau_prev_[idx + 0], -23.70, 23.70);
            effort_msg.data[idx + 1] = std::clamp(tau_prev_[idx + 1], -23.70, 23.70);
            effort_msg.data[idx + 2] = std::clamp(tau_prev_[idx + 2], -45.43, 45.43);
        }

        pub_effort_->publish(effort_msg);

        if (++log_counter_ >= 100) { 
            log_counter_ = 0;
            std::cout << "\n========================================= UNFILTERED POSTURE TRACKING MATRIX =========================================\n"
                      << "Progress: " << std::fixed << std::setprecision(2) << (alpha * 100.0) << "%\n"
                      << "JOINT NAME            | TARGET POS | CURRENT POS | POSITION ERROR | VELOCITY   | APPLIED TORQUE\n"
                      << "-------------------------------------------------------------------------------------------------------------------------\n";
            for (int j = 0; j < 12; ++j) {
                double target_val = (j % 3 == 0) ? Q_STAND_HIP : ((j % 3 == 1) ? Q_STAND_UPPER_LEG : Q_STAND_LOWER_LEG);
                std::cout << std::left << std::setw(21) << joint_names_[j] << " | "
                          << std::right << std::setw(10) << target_val << " | "
                          << std::setw(11) << q_map.at(joint_names_[j]) << " | "
                          << std::setw(14) << (target_val - q_map.at(joint_names_[j])) << " | "
                          << std::setw(10) << (v_map.count(joint_names_[j]) ? v_map.at(joint_names_[j]) : 0.0) << " | "
                          << std::setw(14) << effort_msg.data[j] << "\n";
                if ((j + 1) % 3 == 0) std::cout << "-------------------------------------------------------------------------------------------------------------------------\n";
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Go2StandController>()); rclcpp::shutdown(); return 0;
}