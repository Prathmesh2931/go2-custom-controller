#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp> 
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
#include "go2_mpc_controller/mpc_solver.hpp"
#include "go2_mpc_controller/robot_model.hpp"

namespace go2_physics {
    const double HIP_OFFSET_N = 0.0955;
    const double THIGH_LEN_N = 0.213;
    const double CALF_LEN_N = 0.213;
}

class DynamicTracker : public rclcpp::Node {
public:
    DynamicTracker() : Node("dynamic_tracker"), mpc_running_(false), shutdown_(false) {
        
        auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group;

        pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_effort_controller/commands", 10);
            
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DynamicTracker::js_callback, this, std::placeholders::_1), sub_opt);
            
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/ground_truth", 10,
            std::bind(&DynamicTracker::odom_callback, this, std::placeholders::_1), sub_opt);

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
                cmd_vx_ = msg->linear.x;
                cmd_vy_ = msg->linear.y;
                cmd_wz_ = msg->angular.z;
            }, sub_opt);

        sub_gait_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gait/planned_positions", 10,
            std::bind(&DynamicTracker::gait_callback, this, std::placeholders::_1), sub_opt);

        // --- THE FIX: Listen to Intent, not the Gazebo Bumpers! ---
        sub_contact_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/gait/planned_contacts", 10,
            std::bind(&DynamicTracker::contact_callback, this, std::placeholders::_1), sub_opt);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&DynamicTracker::control_loop, this), cb_group);

        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };

        void* mem = std::aligned_alloc(32, sizeof(MpcSolver));
        solver_ = new (mem) MpcSolver();
        latest_f_optimal_std_.assign(12, 0.0);

        mpc_thread_ = std::thread(&DynamicTracker::mpc_thread_func, this);

        RCLCPP_INFO(this->get_logger(), "LAYER 2 ONLINE: INTENT TRACKING AND DELTA-FORCE UNLEASHED.");
    }

    ~DynamicTracker() {
        shutdown_.store(true);
        if (mpc_thread_.joinable()) mpc_thread_.join();
    }

private:
    MpcSolver* solver_;
    std::vector<std::string> joint_names_;
    sensor_msgs::msg::JointState last_js_;

    double hip_offset_x_[4] = { 0.1934,  0.1934, -0.1934, -0.1934};
    double hip_offset_y_[4] = { 0.0465, -0.0465,  0.0465, -0.0465};
    
    std::mutex state_mutex_;
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    double pos_z_ = 0.28; 
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    double wx_ = 0.0, wy_ = 0.0, wz_ = 0.0;
    std::vector<double> shared_pos_{12, 0.0};

    std::mutex cmd_mutex_;
    double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;
    double smooth_vx_ = 0.0, smooth_vy_ = 0.0, smooth_wz_ = 0.0;

    bool js_received_ = false, odom_received_ = false;

    std::mutex mpc_mutex_;
    std::vector<double> latest_f_optimal_std_;
    bool mpc_initialized_ = false;

    std::mutex gait_mutex_;
    double current_alpha_ = 0.0;
    
    std::vector<double> dynamic_targets_{
         0.203, 0.668, -1.533, 
        -0.203, 0.668, -1.533, 
         0.203, 0.668, -1.533, 
        -0.203, 0.668, -1.533
    };
    
    std::vector<int> current_contacts_{1, 1, 1, 1}; 
    std::vector<double> filtered_vel_ = std::vector<double>(12, 0.0);
    std::vector<double> start_pos_;
    
    std::thread mpc_thread_;
    std::atomic<bool> mpc_running_;
    std::atomic<bool> shutdown_;

    rclcpp::Time start_time_;
    bool first_tick_ = true;
    int log_counter_ = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_gait_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_contact_;

    void mpc_thread_func() {
        int print_counter = 0;
        while (!shutdown_.load() && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); 

            if (!js_received_ || !odom_received_ || first_tick_) continue;
            
            double elapsed = (this->now() - start_time_).seconds();
            if (elapsed < 1.0) continue; 

            double roll, pitch, pos_z, vx, vy, vz, wx, wy, wz;
            std::vector<double> pos(12, 0.0);
            {
                std::lock_guard<std::mutex> state_lock(state_mutex_);
                roll = roll_; pitch = pitch_; pos_z = pos_z_;
                vx = vx_; vy = vy_; vz = vz_;
                wx = wx_; wy = wy_; wz = wz_;
                pos = shared_pos_;
            }

            double current_cmd_vx, current_cmd_vy, current_cmd_wz;
            {
                std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
                current_cmd_vx = cmd_vx_;
                current_cmd_vy = cmd_vy_;
                current_cmd_wz = cmd_wz_;
            }

            smooth_vx_ += 0.05 * (current_cmd_vx - smooth_vx_);
            smooth_vy_ += 0.05 * (current_cmd_vy - smooth_vy_);
            smooth_wz_ += 0.05 * (current_cmd_wz - smooth_wz_);

            tf2::Matrix3x3 rot_yaw_aligned;
            rot_yaw_aligned.setRPY(roll, pitch, 0.0);

            double vx_yaw = rot_yaw_aligned[0][0]*vx + rot_yaw_aligned[0][1]*vy + rot_yaw_aligned[0][2]*vz;
            double vy_yaw = rot_yaw_aligned[1][0]*vx + rot_yaw_aligned[1][1]*vy + rot_yaw_aligned[1][2]*vz;
            double vz_yaw = rot_yaw_aligned[2][0]*vx + rot_yaw_aligned[2][1]*vy + rot_yaw_aligned[2][2]*vz;

            MpcSolver::StateVector state = MpcSolver::StateVector::Zero();
            
            double roll_clamped = std::clamp(roll, -0.35, 0.35);
            double pitch_clamped = std::clamp(pitch, -0.35, 0.35);
            
            // --- THE DELTA FORCE FIX ---
            // Set Gravity to 0.0. The solver will now only output the EXTRA force needed to push forward!
            state << roll_clamped, pitch_clamped, 0.0, 0.0, 0.0, pos_z, wx, wy, wz, vx_yaw, vy_yaw, vz_yaw, 0.0;

            std::vector<Eigen::Vector3d> foot_pos_yaw(4);
            for (int i = 0; i < 4; ++i) {
                Eigen::Vector3d hip_off(hip_offset_x_[i], hip_offset_y_[i], 0.0);
                Eigen::Vector3d r_body = hip_off + go2_physics::RobotModel::calcForwardKinematics(pos[i*3], pos[i*3+1], pos[i*3+2], static_cast<go2_physics::Leg>(i));
                
                foot_pos_yaw[i](0) = rot_yaw_aligned[0][0]*r_body(0) + rot_yaw_aligned[0][1]*r_body(1) + rot_yaw_aligned[0][2]*r_body(2);
                foot_pos_yaw[i](1) = rot_yaw_aligned[1][0]*r_body(0) + rot_yaw_aligned[1][1]*r_body(1) + rot_yaw_aligned[1][2]*r_body(2);
                foot_pos_yaw[i](2) = rot_yaw_aligned[2][0]*r_body(0) + rot_yaw_aligned[2][1]*r_body(1) + rot_yaw_aligned[2][2]*r_body(2);
            } 

            std::vector<int> contacts_for_mpc(4, 1);
            {
                std::lock_guard<std::mutex> gait_lock(gait_mutex_);
                contacts_for_mpc = current_contacts_;
            }

            double cmd_vx_to_solver = (std::abs(smooth_vx_) < 0.05) ? 0.0 : smooth_vx_;
            double cmd_vy_to_solver = (std::abs(smooth_vy_) < 0.05) ? 0.0 : smooth_vy_;
            double cmd_wz_to_solver = (std::abs(smooth_wz_) < 0.05) ? 0.0 : smooth_wz_;

            MpcSolver::ForceVector f = solver_->solve(state, foot_pos_yaw, 0.28, contacts_for_mpc, cmd_vx_to_solver, cmd_vy_to_solver, cmd_wz_to_solver);

            bool solver_failed = (f.norm() < 1e-3);

            {
                std::lock_guard<std::mutex> mpc_lock(mpc_mutex_);
                if (solver_failed && mpc_initialized_) {
                    for (int k = 0; k < 12; ++k) {
                        latest_f_optimal_std_[k] *= 0.85; 
                        f(k) = latest_f_optimal_std_[k];
                    }
                } else {
                    constexpr double MPC_LPF_ALPHA = 0.3;  
                    if (mpc_initialized_) {
                        for (int k = 0; k < 12; ++k)
                            latest_f_optimal_std_[k] += MPC_LPF_ALPHA * (f(k) - latest_f_optimal_std_[k]);
                    } else {
                        for (int k = 0; k < 12; ++k) latest_f_optimal_std_[k] = f(k);
                    }
                    mpc_initialized_ = true;
                }
            }

            if (++print_counter >= 30) {
                print_counter = 0;
                double total_fz = 0.0;
                
                int num_stance = contacts_for_mpc[0] + contacts_for_mpc[1] + contacts_for_mpc[2] + contacts_for_mpc[3];
                double fz_ff_base = (num_stance > 0) ? (15.0 * 9.81 / num_stance) : 0.0;

                std::cout << "\n===== NATIVE MPC BRAIN: GROUND REACTION FORCES =====\n";
                if (solver_failed) std::cout << ">>> WARNING: SOLVER INFEASIBLE. DECAYING LAST VALID FORCES <<<\n";
                std::cout << "CMD_VX=" << std::fixed << std::setprecision(3) << smooth_vx_ 
                          << "   |   ACTUAL CoM Z: " << pos_z << " m\n";
                          
                for (int leg = 0; leg < 4; ++leg) {
                    double fx = f(leg*3+0), fy = f(leg*3+1), fz = f(leg*3+2);
                    
                    if (contacts_for_mpc[leg] == 1) {
                        double rear_weight_bias = (leg >= 2) ? 1.2 : 0.8; 
                        fz += fz_ff_base * rear_weight_bias;
                    }
                    
                    total_fz += fz;
                    std::cout << "LEG " << leg
                            << "  Fx=" << std::setw(7) << std::fixed << std::setprecision(1) << fx
                            << "  Fy=" << std::setw(7) << fy
                            << "  Fz=" << std::setw(7) << fz
                            << "\n";
                }
                std::cout << "TOTAL Fz=" << total_fz << " N (Gravity=~150N)  Blend Alpha=" << current_alpha_ << "\n====================================================\n";
            }
        }
    }

    void js_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = *msg;
        js_received_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);

        std::lock_guard<std::mutex> state_lock(state_mutex_);
        pos_z_ = msg->pose.pose.position.z;
        vx_ = msg->twist.twist.linear.x;
        vy_ = msg->twist.twist.linear.y;
        vz_ = msg->twist.twist.linear.z;
        wx_ = msg->twist.twist.angular.x;
        wy_ = msg->twist.twist.angular.y;
        wz_ = msg->twist.twist.angular.z;
        m.getRPY(roll_, pitch_, yaw_);
        odom_received_ = true;
    }

    void gait_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 12) {
            std::lock_guard<std::mutex> lock(gait_mutex_);
            dynamic_targets_ = msg->data;
        }
    }

    void contact_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) {
            std::lock_guard<std::mutex> lock(gait_mutex_);
            for (int i=0; i<4; i++) current_contacts_[i] = msg->data[i];
        }
    }

    void control_loop() {
        if (!js_received_ || !odom_received_ || last_js_.name.empty() || last_js_.position.size() < 12) return;

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

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            shared_pos_ = current_pos;
        }

        std::vector<double> target_q(12, 0.0);
        std::vector<int> contacts(4, 1);
        {
            std::lock_guard<std::mutex> lock(gait_mutex_);
            target_q = dynamic_targets_;
            contacts = current_contacts_;
        }
        
        double alpha = 0.0;
        if (mpc_initialized_ && elapsed > 2.5) {
            double t = elapsed - 2.5;
            alpha = std::min(t / 1.5, 1.0); 
        }
        current_alpha_ = alpha;  
        
        double ramp = std::clamp(elapsed / 2.0, 0.0, 1.0); 
        
        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(12, 0.0);
        
        std::vector<double> pd_tau(12, 0.0);
        std::vector<double> mpc_tau(12, 0.0);

        for (int i = 0; i < 12; ++i) {
            filtered_vel_[i] = (0.20 * filtered_vel_[i]) + (0.80 * current_vel[i]);
            double vel = filtered_vel_[i];
            
            double gait_target = target_q[i]; 
            double active_target = start_pos_[i] + (gait_target - start_pos_[i]) * ramp;
            double error = active_target - current_pos[i];

            int leg_idx = i / 3;
            double rear_weight_bias = (leg_idx >= 2) ? 1.2 : 1.0; 
            
            double kp, kd;
            if (i % 3 == 0) { 
                kp = 85.0 - (65.0 * alpha); 
                kd = 1.0; 
            } else if (contacts[leg_idx] == 1) { 
                // STANCE: Fade Kp down to let MPC take over, but keep Kd firm to stop chatter!
                kp = (60.0 * rear_weight_bias) - (40.0 * alpha);
                kd = 1.0; 
            } else { 
                // SWING: The Anti-Shake Fix! 
                kp = 50.0 * rear_weight_bias;
                kd = 1.0; // Massive brakes in the air to absorb the violent 27 Nm snaps
            }

            double tau = (kp * error) - (kd * vel);
            double fade_out = std::max(0.0, 1.0 - alpha);
            
            if (i % 3 == 0) {
                if (leg_idx == 0 || leg_idx == 2) tau -= 3.5 * ramp * fade_out; 
                else                              tau += 3.5 * ramp * fade_out;
            } else {
                if (contacts[leg_idx] == 1) {
                    if (i % 3 == 1) tau += -1.5 * rear_weight_bias * ramp * fade_out;   
                    if (i % 3 == 2) tau +=  4.0 * rear_weight_bias * ramp * fade_out;   
                } else {
                    if (i % 3 == 1) tau += -1.0 * ramp;  
                    if (i % 3 == 2) tau +=  2.0 * ramp;
                }
            }

            pd_tau[i] = tau;
        }

        std::vector<double> f_local(12, 0.0);
        bool mpc_ready = false;
        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            mpc_ready = mpc_initialized_;
            if (mpc_ready) f_local = latest_f_optimal_std_;
        }

        tf2::Matrix3x3 rot;
        rot.setRPY(roll_, pitch_, 0.0);

        int num_stance = contacts[0] + contacts[1] + contacts[2] + contacts[3];
        double fz_ff_base = (num_stance > 0) ? (15.0 * 9.81 / num_stance) : 0.0;

        for (int i = 0; i < 4; ++i) {
            go2_physics::Leg leg_enum = static_cast<go2_physics::Leg>(i);
            
            if (mpc_ready && contacts[i] == 1) {
                // Use the official RobotModel so the math perfectly matches the Physics engine!
                Eigen::Matrix3d J = go2_physics::RobotModel::calcAnalyticalJacobian(
                    current_pos[i*3], current_pos[i*3+1], current_pos[i*3+2], leg_enum);

                double fx = f_local[i*3+0];
                double fy = f_local[i*3+1];
                double fz = f_local[i*3+2];
                double rear_weight_bias = (i >= 2) ? 1.2 : 0.8; 
                fz += fz_ff_base * rear_weight_bias;

                double bx = rot[0][0]*fx + rot[1][0]*fy + rot[2][0]*fz;
                double by = rot[0][1]*fx + rot[1][1]*fy + rot[2][1]*fz;
                double bz = rot[0][2]*fx + rot[1][2]*fy + rot[2][2]*fz;

                mpc_tau[i*3+0] = -1.0*(J(0,0)*bx + J(1,0)*by + J(2,0)*bz);
                mpc_tau[i*3+1] = -1.0*(J(0,1)*bx + J(1,1)*by + J(2,1)*bz);
                mpc_tau[i*3+2] = -1.0*(J(0,2)*bx + J(1,2)*by + J(2,2)*bz);
            } else {
                mpc_tau[i*3+0] = 0.0;
                mpc_tau[i*3+1] = 0.0;
                mpc_tau[i*3+2] = 0.0;
            }

            double max_tau = (i == 2 || i == 5 || i == 8 || i == 11) ? 45.43 : 23.70;
            effort_msg.data[i*3+0] = std::clamp(pd_tau[i*3+0] + alpha * mpc_tau[i*3+0], -max_tau, max_tau);
            effort_msg.data[i*3+1] = std::clamp(pd_tau[i*3+1] + alpha * mpc_tau[i*3+1], -max_tau, max_tau);
            effort_msg.data[i*3+2] = std::clamp(pd_tau[i*3+2] + alpha * mpc_tau[i*3+2], -max_tau, max_tau);
        }

        pub_cmd_->publish(effort_msg);

        if (++log_counter_ >= 250) { 
            log_counter_ = 0;
            std::cout << "\n--- DYNAMIC TRACKER (LAYER 2) | Elapsed: " << std::fixed << std::setprecision(1) << elapsed 
                      << "s | MPC BLEND: " << (alpha*100) << "% ---\n";
                      
            std::cout << std::left << std::setw(20) << "JOINT" 
                      << std::right << std::setw(8) << "PHASE"
                      << std::setw(10) << "TARGET" 
                      << std::setw(10) << "ACTUAL" 
                      << std::setw(10) << "PD_TAU"
                      << std::setw(10) << "MPC_TAU"
                      << std::setw(10) << "FINAL\n";
            for (int j = 0; j < 12; ++j) {
                std::string phase = (contacts[j/3] == 1) ? "[STANCE]" : "[SWING ]";
                std::cout << std::left << std::setw(20) << joint_names_[j]
                          << std::right << std::setw(8) << phase
                          << std::setw(10) << std::setprecision(3) << target_q[j]
                          << std::setw(10) << current_pos[j]
                          << std::setw(10) << pd_tau[j]
                          << std::setw(10) << (alpha * mpc_tau[j])
                          << std::setw(10) << effort_msg.data[j] << "\n";
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<DynamicTracker>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}