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
#include <ament_index_cpp/get_package_share_directory.hpp>

// --- SINGLE SOURCE OF TRUTH ---
#include "go2_mpc_controller/robot_model.hpp"

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

        try {
            std::string urdf_path = ament_index_cpp::get_package_share_directory("go2_description") + "/urdf/go2_description.urdf";
            go2_physics::RobotModel::initialize(urdf_path);
            RCLCPP_INFO(this->get_logger(), "LAYER 1 ONLINE: ZERO-LAG TRACKER (STABLE DAMPING).");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "URDF LOAD FAILED! %s", e.what());
        }
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

        // Your exact 2-second safe spawn ramp
        double ramp = std::clamp(elapsed / 2.0, 0.0, 1.0); 
        
        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(12, 0.0);
        
        std::vector<double> current_pd_tau(12, 0.0);

        for (int i = 0; i < 12; ++i) {
            // YOUR REQUEST: Absolute Zero-Lag Velocity (No filters)
            double vel = current_vel[i];

            double gait_target = target_q[i]; 
            double active_target = start_pos_[i] + (gait_target - start_pos_[i]) * ramp;
            double error = active_target - current_pos[i];

            // Rear mass bias to support the heavy battery
            int leg_idx = i / 3;
            double rear_weight_bias = (leg_idx >= 2) ? 1.2 : 1.0;

            double kp, kd;
            if (i % 3 == 0) {
                kp = 95.0 * ramp;
                kd = 1.0  * ramp; // Raised slightly to damp hip shakes
            } else if (contacts[leg_idx] == 1) {
                kp = 60.0 * rear_weight_bias * ramp;
                kd = 0.8  * ramp; // CRITICAL DAMPING: Stops the calf from vibrating
            } else {
                kp = 50.0 * ramp;
                kd = 0.8  * ramp;
            }

            double tau = (kp * error) - (kd * vel);

            // ANTI-SPLITS FIX: Pull the hips inward gently so the Kp spring doesn't fight the whole chassis weight
            if (i % 3 == 0) {
                if (leg_idx == 0 || leg_idx == 2) tau -= 3.0 * ramp; // Left legs pull inward
                else                              tau += 3.0 * ramp; // Right legs pull inward
            }

            // --- MASSIVE GRAVITY FEEDFORWARD BOOST ---
            // We increase the baseline torque so the motors carry the 15kg chassis natively.
            // This eliminates the 3cm Z-sag without causing high-frequency PD shaking!
            if (contacts[leg_idx] == 1) {
                if (i % 3 == 1) tau += -3.0 * rear_weight_bias * ramp;   
                if (i % 3 == 2) tau +=  4.5 * rear_weight_bias * ramp;   
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
                go2_physics::Leg leg_enum = static_cast<go2_physics::Leg>(leg);
                Eigen::Vector3d hip_off = go2_physics::RobotModel::getHipOffset(leg_enum);
                
                // Properly integrated RobotModel
                Eigen::Vector3d actual_fk = go2_physics::RobotModel::calcForwardKinematics(
                    current_pos[leg*3], current_pos[leg*3+1], current_pos[leg*3+2], leg_enum);
                actual_fk(0) += hip_off(0);
                
                Eigen::Vector3d target_fk = go2_physics::RobotModel::calcForwardKinematics(
                    target_q[leg*3], target_q[leg*3+1], target_q[leg*3+2], leg_enum);
                target_fk(0) += hip_off(0);
                
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

            go2_physics::Leg val_leg = go2_physics::Leg::LF;
            try {
                Eigen::Matrix3d M = go2_physics::RobotModel::calcLegMassMatrix(
                    current_pos[0], current_pos[1], current_pos[2], val_leg);
                
                Eigen::Vector3d g = go2_physics::RobotModel::calcLegGravity(
                    current_pos[0], current_pos[1], current_pos[2], val_leg);

                std::cout << "\n--- PINOCCHIO DYNAMICS VALIDATION (LF LEG) ---\n";
                std::cout << "Gravity Torque g(q) required for LEGS ONLY (No chassis mass):\n";
                std::cout << "  Hip:   " << std::setw(6) << g(0) << " Nm\n";
                std::cout << "  Thigh: " << std::setw(6) << g(1) << " Nm (vs Manual PD 1.5 Nm)\n";
                std::cout << "  Calf:  " << std::setw(6) << g(2) << " Nm (vs Manual PD 4.0 Nm)\n";
                
                std::cout << "\nMass Matrix M(q) Diagonal (Inertia):\n";
                std::cout << "  M_11 (Hip):   " << M(0,0) << "\n";
                std::cout << "  M_22 (Thigh): " << M(1,1) << "\n";
                std::cout << "  M_33 (Calf):  " << M(2,2) << "\n";

            } catch (const std::exception& e) {
                std::cout << "\n[!] PINOCCHIO VALIDATION FAILED: " << e.what() << "\n";
            }
            std::cout << "===============================================================\n";
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