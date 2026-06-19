#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

class Go2StandNode : public rclcpp::Node {
public:
    Go2StandNode() : Node("go2_stand_node") {
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_effort_controller/commands", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Go2StandNode::js_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(4), std::bind(&Go2StandNode::control_loop, this));
        
        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };

        RCLCPP_INFO(this->get_logger(), "Standing Controller Active. User-Optimized PD + Feedforward enabled.");
    }

private:
    const std::vector<double> targets = {0.0, 0.67, -1.30, 0.0, 0.67, -1.30, 0.0, 0.67, -1.30, 0.0, 0.67, -1.30};
    
    // USER OPTIMIZED STABLE GAINS
    // These specific values prevent Gazebo's numerical physics engine from vibrating.
    double kp = 45.0; // Slightly bumped from 40 for just a tiny bit more structure
    double kd = 0.4;

    // GRAVITY FEEDFORWARD (Derived directly from your logs)
    // This perfectly cancels the weight of the robot so the soft Kp=45 doesn't sag.
    // Thighs: -2.0 Nm | Calves: +5.0 Nm
    const std::vector<double> ff = {
        0.0, -2.0, 5.0, // LF
        0.0, -2.0, 5.0, // RF
        0.0, -2.0, 5.0, // LH
        0.0, -2.0, 5.0  // RH
    };

    std::vector<std::string> joint_names_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState last_js_;
    
    bool first_tick_ = true;
    std::vector<double> start_pos_;
    rclcpp::Time start_time_;
    int log_counter_ = 0;

    void js_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = *msg;
    }

    void control_loop() {
        if (last_js_.name.empty()) return;

        // Capture initial posture to prevent violent snapping
        if (first_tick_) {
            start_pos_.resize(12, 0.0);
            for (int i = 0; i < 12; ++i) {
                auto it = std::find(last_js_.name.begin(), last_js_.name.end(), joint_names_[i]);
                if (it != last_js_.name.end()) {
                    int idx = std::distance(last_js_.name.begin(), it);
                    start_pos_[i] = last_js_.position[idx];
                }
            }
            start_time_ = this->now();
            first_tick_ = false;
            return;
        }

        // Calculate smooth trajectory ramp over 2.5 seconds
        double elapsed = (this->now() - start_time_).seconds();
        double ramp = std::clamp(elapsed / 2.5, 0.0, 1.0);

        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(12, 0.0);
        
        std::vector<double> current_pos(12, 0.0);
        std::vector<double> raw_tau(12, 0.0);

        for (int i = 0; i < 12; ++i) {
            auto it = std::find(last_js_.name.begin(), last_js_.name.end(), joint_names_[i]);
            if (it == last_js_.name.end()) continue;
            int idx = std::distance(last_js_.name.begin(), it);
            
            double pos = last_js_.position[idx];
            double vel = last_js_.velocity[idx];
            current_pos[i] = pos;

            // VELOCITY DEADBAND: Kills microscopic noise chatter
            if (std::abs(vel) < 0.05) {
                vel = 0.0;
            }
            
            // Interpolate target from resting pose to standing pose
            double active_target = start_pos_[i] + (targets[i] - start_pos_[i]) * ramp;
            double error = active_target - pos;

            // PURE PD + FEEDFORWARD LOOP (No Integral Windup to cause swaying!)
            double tau = (kp * error) - (kd * vel) + (ff[i] * ramp);

            raw_tau[i] = tau;

            // ABSOLUTE MAXIMUM Torque Limits immediately available.
            // Calves (index 2, 5, 8, 11) get 45.43 Nm. Hips/Thighs get 23.7 Nm.
            double torque_limit = (i % 3 == 2) ? 45.43 : 23.70;

            effort_msg.data[i] = std::clamp(tau, -torque_limit, torque_limit);
        }
        
        pub_->publish(effort_msg);

        // Logging (Every ~1 second)
        if (++log_counter_ >= 250) { 
            log_counter_ = 0;
            std::cout << "\n============================== CONTROLLER DIAGNOSTICS ==============================\n"
                      << "RAMP PROGRESS: " << (ramp * 100.0) << "%\n"
                      << std::left << std::setw(20) << "JOINT" 
                      << std::right << std::setw(10) << "TARGET" 
                      << std::setw(10) << "ACTUAL" 
                      << std::setw(10) << "ERROR" 
                      << std::setw(12) << "RAW_TAU" 
                      << std::setw(12) << "CLAMP_TAU" << "\n"
                      << "------------------------------------------------------------------------------------\n";
            for (int j = 0; j < 12; ++j) {
                std::cout << std::left << std::setw(20) << joint_names_[j]
                          << std::right << std::setw(10) << std::fixed << std::setprecision(3) << targets[j]
                          << std::setw(10) << current_pos[j]
                          << std::setw(10) << (targets[j] - current_pos[j])
                          << std::setw(12) << raw_tau[j]
                          << std::setw(12) << effort_msg.data[j] << "\n";
                if ((j + 1) % 3 == 0) std::cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n";
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2StandNode>());
    rclcpp::shutdown();
    return 0;
}