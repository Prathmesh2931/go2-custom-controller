#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <Eigen/Dense>

// --- NATIVE MATH INJECTION ---
// Using the exact Go2 physics math provided to bypass CMake dependencies!
namespace go2_physics {
    const double HIP_OFFSET = 0.0955;
    const double THIGH_LEN = 0.213;
    const double CALF_LEN = 0.213;

    Eigen::Vector3d calcFootPosition(double q1, double q2, double q3, int leg_index) {
        double l1 = (leg_index == 0 || leg_index == 2) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;

        double s1 = std::sin(q1);
        double c1 = std::cos(q1);
        double s2 = std::sin(q2);
        double c2 = std::cos(q2);
        double s23 = std::sin(q2 + q3);
        double c23 = std::cos(q2 + q3);

        Eigen::Vector3d pos;
        pos(0) = -l2 * s2 - l3 * s23;
        pos(1) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
        pos(2) = l1 * s1 - c1 * (l2 * c2 + l3 * c23);
        return pos;
    }
}

class PurePdTracker : public rclcpp::Node {
public:
    PurePdTracker() : Node("pure_pd_tracker") {
        
        pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_effort_controller/commands", 10);
            
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&PurePdTracker::js_callback, this, std::placeholders::_1));
            
        sub_gait_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gait/planned_positions", 10,
            std::bind(&PurePdTracker::gait_callback, this, std::placeholders::_1));

        sub_contact_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/state_estimator/contact_states", 10,
            std::bind(&PurePdTracker::contact_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&PurePdTracker::control_loop, this));

        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };

        RCLCPP_INFO(this->get_logger(), "LAYER 1: ZERO-LAG TRACKER (WITH REAR MASS BIAS).");
    }

private:
    std::vector<std::string> joint_names_;
    sensor_msgs::msg::JointState last_js_;
    bool js_received_ = false;

    std::mutex data_mutex_;
    
    // Baseline Standing Targets
    std::vector<double> dynamic_targets_{
        0.154, 0.796, -1.591, 
        -0.154, 0.796, -1.591, 
        0.154, 0.796, -1.591, 
        -0.154, 0.796, -1.591 
    };
    
    std::vector<int> current_contacts_{1, 1, 1, 1}; 
    std::vector<double> start_pos_;
    std::vector<double> prev_target_ = std::vector<double>(12, 0.0);
    std::vector<double> filtered_vel_ = std::vector<double>(12, 0.0);

    const double hip_offset_x_[4] = { 0.1934,  0.1934, -0.1934, -0.1934};
    const double hip_offset_y_[4] = { 0.0465, -0.0465,  0.0465, -0.0465};

    rclcpp::Time start_time_;
    bool first_tick_ = true;
    int log_counter_ = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_gait_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_contact_;

    void js_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = *msg;
        js_received_ = true;
    }

    void gait_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 12) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            dynamic_targets_ = msg->data;
        }
    }

    void contact_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i=0; i<4; i++) current_contacts_[i] = msg->data[i];
        }
    }

    void control_loop() {
        if (!js_received_ || last_js_.name.empty() || last_js_.position.size() < 12) return;

        static bool time_set = false;
        if (!time_set) {
            start_time_ = this->now();
            time_set = true;
        }

        double elapsed = (this->now() - start_time_).seconds();

        if (first_tick_) {
            start_pos_.resize(12, 0.0);
            for (int i = 0; i < 12; ++i) {
                auto it = std::find(last_js_.name.begin(), last_js_.name.end(), joint_names_[i]);
                if (it != last_js_.name.end()) {
                    start_pos_[i] = last_js_.position[std::distance(last_js_.name.begin(), it)];
                    prev_target_[i] = start_pos_[i]; // Initialize prev target
                }
            }
            first_tick_ = false;
            return;
        }

        std::vector<double> current_pos(12, 0.0);
        std::vector<double> current_vel(12, 0.0);
        for (int i = 0; i < 12; ++i) {
            auto it = std::find(last_js_.name.begin(), last_js_.name.end(), joint_names_[i]);
            if (it != last_js_.name.end()) {
                int idx = std::distance(last_js_.name.begin(), it);
                current_pos[i] = last_js_.position[idx];
                current_vel[i] = last_js_.velocity[idx];
            }
        }

        std::vector<double> target_q(12, 0.0);
        std::vector<int> contacts(4, 1);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            target_q = dynamic_targets_;
            contacts = current_contacts_;
        }

        double ramp = std::clamp(elapsed / 2.0, 0.0, 1.0); 
        
        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(12, 0.0);
        
        std::vector<double> current_pd_tau(12, 0.0);

        for (int i = 0; i < 12; ++i) {
            double vel = current_vel[i];

            // 250Hz instant passthrough (no filter lag!)
            double gait_target = target_q[i]; 

            double active_target = start_pos_[i] + (gait_target - start_pos_[i]) * ramp;
            double error = active_target - current_pos[i];

            // --- THE REAR MASS BIAS ---
            // Legs 2 & 3 (Rear) get a 40% boost in stiffness and gravity comp
            // to counteract the heavy battery pack and lift the rear up to Z=-0.28m!
            int leg_idx = i / 3;
            double rear_weight_bias = (leg_idx >= 2) ? 1.2 : 1.0;

            double kp, kd;
            if (i % 3 == 0) {
                kp = 65.0 * ramp;
                kd = 0.5  * ramp;
            } else if (contacts[leg_idx] == 1) {
                // Stance
                kp = 55.0 * rear_weight_bias * ramp;
                kd = 0.5  * ramp;
            } else {
                // Swing
                kp = 55.0 * ramp;
                kd = 0.4  * ramp;
            }

            double tau = (kp * error) - (kd * vel);

            // if (i % 3 == 0) {
            //     if (leg_idx == 0 || leg_idx == 2) tau -= 9.0 * ramp; 
            //     else                              tau += 9.0 * ramp;
            // }

            // Gravity Comp (boosted for the heavy rear!)
            if (contacts[leg_idx] == 1) {
                if (i % 3 == 1) tau += -1.5 * rear_weight_bias * ramp;   
                if (i % 3 == 2) tau +=  4.0 * rear_weight_bias * ramp;   
            } else {
                if (i % 3 == 1) tau += -0.5 * ramp;  
                if (i % 3 == 2) tau +=  1.5 * ramp;
            }

            current_pd_tau[i] = tau;
            double max_tau = (i % 3 == 2) ? 45.0 : 23.7;
            effort_msg.data[i] = std::clamp(tau, -max_tau, max_tau);
        }

        pub_cmd_->publish(effort_msg);

        if (++log_counter_ >= 100) { 
            log_counter_ = 0;
            std::cout << "\n===============================================================\n";
            std::cout << "PURE PD TRACKING VERIFICATION | Elapsed: " << std::fixed << std::setprecision(1) << elapsed << "s\n";
            
            std::cout << "=== 3D CARTESIAN ERROR (WORLD FRAME) ===\n";
            for (int leg = 0; leg < 4; ++leg) {
                Eigen::Vector3d actual_fk = go2_physics::calcFootPosition(
                    current_pos[leg*3], current_pos[leg*3+1], current_pos[leg*3+2], leg);
                actual_fk(0) += hip_offset_x_[leg];
                
                Eigen::Vector3d target_fk = go2_physics::calcFootPosition(
                    target_q[leg*3], target_q[leg*3+1], target_q[leg*3+2], leg);
                target_fk(0) += hip_offset_x_[leg];
                
                double err_x = target_fk(0) - actual_fk(0);
                double err_z = target_fk(2) - actual_fk(2);

                std::string status = (contacts[leg] == 1) ? "[GROUND]" : "[ AIR  ]";
                std::cout << "LEG " << leg << " " << status
                          << "  ACTUAL X=" << std::setw(6) << std::fixed << std::setprecision(3) << actual_fk(0)
                          << " (Err X: " << std::setw(6) << err_x << ")"
                          << "  ACTUAL Z=" << std::setw(6) << actual_fk(2) 
                          << " (Err Z: " << std::setw(6) << err_z << ")\n";
            }
            std::cout << "---------------------------------------------------------------\n";
            
            std::cout << std::left << std::setw(20) << "JOINT" 
                      << std::right << std::setw(8) << "PHASE"
                      << std::setw(10) << "TARGET" 
                      << std::setw(10) << "ACTUAL" 
                      << std::setw(10) << "ERROR" 
                      << std::setw(12) << "PD_TAU\n";
            for (int j = 0; j < 12; ++j) {
                std::string phase = (contacts[j/3] == 1) ? "[STANCE]" : "[SWING ]";
                std::cout << std::left << std::setw(20) << joint_names_[j]
                          << std::right << std::setw(8) << phase
                          << std::setw(10) << std::setprecision(3) << target_q[j]
                          << std::setw(10) << current_pos[j]
                          << std::setw(10) << (target_q[j] - current_pos[j])
                          << std::setw(12) << current_pd_tau[j] << "\n";
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<PurePdTracker>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}