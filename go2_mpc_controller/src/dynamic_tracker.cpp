#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
#include "go2_mpc_controller/robot_dynamics.hpp"

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

        sub_gait_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gait/planned_positions", 10,
            std::bind(&DynamicTracker::gait_callback, this, std::placeholders::_1), sub_opt);

        sub_contact_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/state_estimator/contact_states", 10,
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

        RCLCPP_INFO(this->get_logger(), "FINAL STEP: ALL FILTERS DELETED. ZERO LATENCY SWING TRACKING.");
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

    bool js_received_ = false, odom_received_ = false;

    std::mutex mpc_mutex_;
    std::vector<double> latest_f_optimal_std_;
    bool mpc_initialized_ = false;

    std::mutex gait_mutex_;

    double current_alpha_ = 0.0;
    
    // Default Stance targets matching 0.28m
    std::vector<double> dynamic_targets_{
         0.154, 0.796, -1.591, 
        -0.154, 0.796, -1.591, 
         0.154, 0.796, -1.591, 
        -0.154, 0.796, -1.591
    };
    std::vector<int> current_contacts_{1, 1, 1, 1}; 

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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_gait_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_contact_;

    std::vector<double> start_pos_;

    void mpc_thread_func() {
        int print_counter = 0;
        while (!shutdown_.load() && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); 

            if (!js_received_ || !odom_received_ || first_tick_) continue;
            
            double elapsed = (this->now() - start_time_).seconds();
            if (elapsed < 1.0) continue; 

            double roll, pitch, yaw, pos_z, vx, vy, vz, wx, wy, wz;
            std::vector<double> pos(12, 0.0);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                roll = roll_; pitch = pitch_; yaw = yaw_; pos_z = pos_z_;
                vx = vx_; vy = vy_; vz = vz_;
                wx = wx_; wy = wy_; wz = wz_;
                pos = shared_pos_;
            }

            MpcSolver::StateVector state = MpcSolver::StateVector::Zero();
            state << roll, pitch, 0.0, 0.0, 0.0, pos_z, wx, wy, wz, vx, vy, vz, -9.81;

            std::vector<Eigen::Vector3d> foot_pos(4);
            for (int i = 0; i < 4; ++i) {
                Eigen::Vector3d hip_off(hip_offset_x_[i], hip_offset_y_[i], 0.0);
                foot_pos[i] = hip_off + go2_physics::calcFootPosition(pos[i*3], pos[i*3+1], pos[i*3+2], i);
            }

            std::vector<int> contacts_for_mpc(4, 1);
            {
                std::lock_guard<std::mutex> lock(gait_mutex_);
                contacts_for_mpc = current_contacts_;
            }

            MpcSolver::ForceVector f = solver_->solve(state, foot_pos, 0.28, contacts_for_mpc);

            if (++print_counter >= 30) {
                print_counter = 0;
                double total_fz = 0.0;
                std::cout << "\n===== MPC FORCE OUTPUT =====\n";
                for (int leg = 0; leg < 4; ++leg) {
                    double fx = f(leg*3+0), fy = f(leg*3+1), fz = f(leg*3+2);
                    total_fz += fz;
                    std::cout << "LEG " << leg
                            << "  Fx=" << std::setw(7) << std::fixed << std::setprecision(1) << fx
                            << "  Fy=" << std::setw(7) << fy
                            << "  Fz=" << std::setw(7) << fz
                            << (fz < 0 ? "  <<< VIOLATION" : "")
                            << "\n";
                }
                std::cout << "TOTAL Fz=" << total_fz << "  alpha=" << current_alpha_
                        << (total_fz > 250 ? "  <<< TOO HIGH" : "")
                        << (total_fz <  80 ? "  <<< TOO LOW"  : "")
                        << "\n============================\n";
            }

            {
                std::lock_guard<std::mutex> lock(mpc_mutex_);
                for (int k = 0; k < 12; ++k) latest_f_optimal_std_[k] = f(k);
                mpc_initialized_ = true;
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

        std::lock_guard<std::mutex> lock(state_mutex_);
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

        double ramp = std::clamp(elapsed / 2.5, 0.0, 1.0); 
        
        // 70% MPC Authority.
        double alpha = 0.0;
        if (mpc_initialized_ && elapsed > 3.0) {
            double t      = elapsed - 3.0;
            double target = 1.00;   
            alpha = std::min(t / 2.0, 1.0) * target;  
        }
        current_alpha_ = alpha;  
        
        double ff_fade = std::clamp(1.0 - alpha, 0.0, 1.0);

        std::vector<double> pd_tau(12, 0.0);
        std::vector<double> mpc_tau(12, 0.0);

        for (int i = 0; i < 12; ++i) {
            // PURE RAW VELOCITY
            double vel = current_vel[i];
            
            // THE FIX: PURE RAW POSITION! No low-pass filter to slow down the swing trajectory!
            // The leg will now lift exactly when and where the Gait Planner tells it to.
            double active_target = start_pos_[i] + (target_q[i] - start_pos_[i]) * ramp;
            double error = active_target - current_pos[i];
            
            double kp, kd;
            if (contacts[i/3] == 1) {
                // Stance Phase: Soft so it doesn't fight the MPC
                
                kp = 40.0 * ramp;
                kd = 0.5 * ramp;
            } else {
                // Swing Phase: Very stiff so it strictly traces the air curve
                kp = 70.0 * ramp;
                kd = 0.5 * ramp;
            }
            int joint_type = i % 3;  // 0=hip, 1=thigh, 2=calf


            // if (joint_type == 0) {
            //     kp = 90.0 * ramp;   // hip needs 2× — fighting gravity moment
            //     kd = 0.8  * ramp;   // more damping too — stop the oscillation
            // }
            
            pd_tau[i] = (kp * error) - (kd * vel);

            if (i % 3 == 0) {  // hip joint
                int leg = i / 3;
                if (leg == 0 || leg == 2) {
                    // LF, LH: positive hip target needs negative feedforward
                    pd_tau[i] -= 4.0 * ramp;
                } else {
                    // RF, RH: negative hip target needs positive feedforward
                    pd_tau[i] += 4.0 * ramp;
                }
            }
            
            // Symmetrical Static Gravity
            if (contacts[i / 3] == 1) {
                if (i % 3 == 1) pd_tau[i] += -1.0 * ramp * ff_fade;
                if (i % 3 == 2) pd_tau[i] +=  5.0 * ramp * ff_fade;
            } else {
                if (i % 3 == 1) pd_tau[i] += -0.5 * ramp;
                if (i % 3 == 2) pd_tau[i] +=  1.5 * ramp;
            }
        }

        std::vector<double> f_local(12, 0.0);
        bool mpc_ready = false;
        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            mpc_ready = mpc_initialized_;
            if (mpc_ready) f_local = latest_f_optimal_std_;
        }

        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(12, 0.0);

        tf2::Matrix3x3 rot;
        rot.setRPY(roll_, pitch_, 0.0);

        for (int i = 0; i < 4; ++i) {
            if (mpc_ready && contacts[i] == 1) {
                Eigen::Matrix3d J = go2_physics::calcLegJacobian(current_pos[i*3], current_pos[i*3+1], current_pos[i*3+2], i);

                double fx = f_local[i*3+0];
                double fy = f_local[i*3+1];
                double fz = f_local[i*3+2];

                double bx = rot[0][0]*fx + rot[1][0]*fy + rot[2][0]*fz;
                double by = rot[0][1]*fx + rot[1][1]*fy + rot[2][1]*fz;
                double bz = rot[0][2]*fx + rot[1][2]*fy + rot[2][2]*fz;

                mpc_tau[i*3+0] = -1.0*(J(0,0)*bx + J(1,0)*by + J(2,0)*bz);
                mpc_tau[i*3+1] = -1.0*(J(0,1)*bx + J(1,1)*by + J(2,1)*bz);
                mpc_tau[i*3+2] = -1.0*(J(0,2)*bx + J(1,2)*by + J(2,2)*bz);
            } else {
                // When in swing phase, MPC provides EXACTLY 0.0 force! PD handles the swing purely.
                mpc_tau[i*3+0] = 0.0;
                mpc_tau[i*3+1] = 0.0;
                mpc_tau[i*3+2] = 0.0;
            }

            effort_msg.data[i*3+0] = std::clamp(pd_tau[i*3+0] , -23.70, 23.70);
            effort_msg.data[i*3+1] = std::clamp(pd_tau[i*3+1] + alpha*mpc_tau[i*3+1], -23.70, 23.70);
            effort_msg.data[i*3+2] = std::clamp(pd_tau[i*3+2] + alpha*mpc_tau[i*3+2], -45.43, 45.43);
        }

        pub_cmd_->publish(effort_msg);

        if (++log_counter_ >= 250) {
            log_counter_ = 0;
            std::cout << "\n--- CONTROL STATUS  elapsed=" << std::fixed << std::setprecision(1) << elapsed
                      << "s  ramp=" << (ramp*100) << "%"
                      << "  MPC=" << (mpc_ready ? "BLENDING " + std::to_string((int)(alpha*100)) + "%" : "WAIT") << " ---\n";
                      
            double r_deg, p_deg, y_deg;
            rot.getRPY(r_deg, p_deg, y_deg);
            std::cout << "CHASSIS TILT -> Roll: " << std::setprecision(2) << r_deg * 180.0 / M_PI 
                      << " deg | Pitch: " << p_deg * 180.0 / M_PI << " deg\n";
                      
            std::cout << "Z-HEIGHT -> Actual: " << std::setprecision(3) << pos_z_ << " m | Target: 0.280 m\n";
                      
            std::cout << std::left << std::setw(20) << "JOINT" 
                      << std::right << std::setw(10) << "TARGET" 
                      << std::setw(10) << "ACTUAL" 
                      << std::setw(10) << "ERROR" 
                      << std::setw(10) << "PD_TAU"
                      << std::setw(10) << "MPC_TAU"
                      << std::setw(12) << "FINAL_TAU\n";
            for (int j = 0; j < 12; ++j) {
                std::cout << std::left << std::setw(20) << joint_names_[j]
                          << std::right << std::setw(10) << std::setprecision(3) << target_q[j]
                          << std::setw(10) << current_pos[j]
                          << std::setw(10) << (target_q[j] - current_pos[j])
                          << std::setw(10) << pd_tau[j]
                          << std::setw(10) << mpc_tau[j]
                          << std::setw(12) << effort_msg.data[j] << "\n";
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