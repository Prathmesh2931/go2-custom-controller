#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

class StateEstimate : public rclcpp::Node
{
public:
    StateEstimate() : Node("state_estimate")
    {
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&StateEstimate::cb_js, this, std::placeholders::_1));
        
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&StateEstimate::cb_imu, this, std::placeholders::_1));   
            
        // THE FIX: Consume the smooth Bayesian Probabilities, not binary states!
        sub_contact_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/state_estimator/contact_probs", 10, std::bind(&StateEstimate::cb_contact, this, std::placeholders::_1));
        
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/estimate", 10);
        pub_diag_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ekf/diagnostics", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(4), std::bind(&StateEstimate::tick, this));
        initialize_ekf();
        RCLCPP_INFO(get_logger(), "LAYER 0: 250Hz AUTONOMOUS EKF ONLINE (GROUND TRUTH SEVERED).");
    }

private:
    static constexpr double HIP_OFFSET = 0.0955;
    static constexpr double THIGH_LEN  = 0.213;
    static constexpr double CALF_LEN   = 0.213;
    static constexpr double LEG_X_DIST = 0.1934;
    static constexpr double LEG_Y_DIST = 0.0465; 
    
    static constexpr int STATE_SIZE = 18;

    Eigen::Matrix<double, STATE_SIZE, 1> x_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q_;

    std::unordered_map<std::string, double> js_;
    std::unordered_map<std::string, double> js_vel_;

    Eigen::Matrix3d R_body_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu_accel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyro_  = Eigen::Vector3d::Zero(); 

    bool ready_ = false;
    rclcpp::Time last_time_;
    bool first_tick_ = true;
    int print_counter_ = 0;
    
    double contact_probs_[4] = {1.0, 1.0, 1.0, 1.0}; 
    bool prev_stance_[4] = {false, false, false, false};

    void initialize_ekf()
    {
        x_.setZero(); P_.setZero(); Q_.setZero();
        x_(2) = 0.28; // Initialize at spawn height
        P_.diagonal().setConstant(0.01); 

        // Process noise (Confidence in IMU Integration)
        Q_.diagonal().segment<3>(0).setConstant(0.001);  // Body pos
        Q_.diagonal().segment<3>(3).setConstant(0.01);   // Body vel
        Q_.diagonal().segment<12>(6).setConstant(0.001); // Foot pos
    }

    Eigen::Vector3d forward_kinematics(double q1, double q2, double q3, int leg_index)
    {
        double l1 = (leg_index == 0 || leg_index == 2) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;
        double s1 = std::sin(q1), c1 = std::cos(q1);
        double s2 = std::sin(q2), c2 = std::cos(q2);
        double s23 = std::sin(q2+q3), c23 = std::cos(q2+q3);
        Eigen::Vector3d pos;
        pos(0) = -l2*s2 - l3*s23;
        pos(1) = l1*c1 + s1*(l2*c2 + l3*c23);
        pos(2) = l1*s1 - c1*(l2*c2 + l3*c23);
        return pos;
    }

    Eigen::Matrix3d calcLegJacobian(double q1, double q2, double q3, int leg_index) {
        double l1 = (leg_index == 0 || leg_index == 2) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;
        double s1 = std::sin(q1), c1 = std::cos(q1);
        double s2 = std::sin(q2), c2 = std::cos(q2);
        double s23 = std::sin(q2 + q3), c23 = std::cos(q2 + q3);

        Eigen::Matrix3d J;
        J(0, 0) = 0.0;
        J(0, 1) = -l2 * c2 - l3 * c23;
        J(0, 2) = -l3 * c23;
        J(1, 0) = -l1 * s1 + c1 * (l2 * c2 + l3 * c23);
        J(1, 1) = s1 * (-l2 * s2 - l3 * s23);
        J(1, 2) = s1 * (-l3 * s23);
        J(2, 0) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
        J(2, 1) = c1 * (l2 * s2 + l3 * s23);
        J(2, 2) = c1 * (l3 * s23);
        return J;
    }

    void cb_js(const sensor_msgs::msg::JointState::SharedPtr msg) {        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            js_[msg->name[i]] = msg->position[i];
            if (js_vel_.find(msg->name[i]) == js_vel_.end()) {
                js_vel_[msg->name[i]] = msg->velocity[i];
            } else {
                js_vel_[msg->name[i]] = 0.8 * js_vel_[msg->name[i]] + 0.2 * msg->velocity[i];
            }
        }
        ready_ = true;
    }

    void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto& q = msg->orientation;
        imu_accel_ = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        imu_gyro_  = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
        R_body_ = eigen_q.toRotationMatrix();
    }

    void cb_contact(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if(msg->data.size() == 4) {
            for(int i=0; i<4; ++i) contact_probs_[i] = msg->data[i];
        }
    }

    void tick()
    {
        if (!ready_) return;

        Eigen::Vector3d gravity(0.0, 0.0, 9.81);
        Eigen::Vector3d accel_world = R_body_ * imu_accel_ - gravity;

        bool has_nan   = x_.array().isNaN().any();
        bool diverged  = std::abs(x_(2)) > 3.0 || P_(0,0) > 1e6;
        if (has_nan || diverged) {
            RCLCPP_WARN(get_logger(), "EKF reset: Z=%.2f P00=%.1f", x_(2), P_(0,0));
            initialize_ekf();
            first_tick_ = true;
            return;
        }
        
        std::string leg_prefix[4] = {"lf_", "rf_", "lh_", "rh_"};
        
        if (first_tick_) {
            last_time_ = this->now();
            for (int i = 0; i < 4; ++i) {
                std::string hip_str = leg_prefix[i] + "hip_joint";
                std::string thigh_str = leg_prefix[i] + "upper_leg_joint";
                std::string calf_str = leg_prefix[i] + "lower_leg_joint";
                if (js_.find(hip_str) != js_.end()) {
                    Eigen::Vector3d hip_body;
                    hip_body(0) = (i < 2) ?  LEG_X_DIST : -LEG_X_DIST;
                    hip_body(1) = (i == 0 || i == 2) ? LEG_Y_DIST : -LEG_Y_DIST;
                    hip_body(2) = 0.0;

                    Eigen::Vector3d r_foot_body = forward_kinematics(js_[hip_str], js_[thigh_str], js_[calf_str], i);
                    x_.segment<3>(6 + i * 3) = x_.segment<3>(0) + R_body_ * (hip_body + r_foot_body);
                    x_(6 + i * 3 + 2) = 0.0; 
                }
            }
            first_tick_ = false; return; 
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0.0 || dt > 0.1) dt = 0.004;

        // --- PREDICTION STEP (IMU) ---
        x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * accel_world * dt * dt; 
        x_.segment<3>(3) += accel_world * dt; 

        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        
        P_.noalias() = F * P_ * F.transpose() + (Q_ * dt);

        double latest_nis = 0.0;
        Eigen::Vector3d w_world = R_body_ * imu_gyro_;
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I_sys = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();

        // --- MEASUREMENT STEP (KINEMATICS) ---
        for (int i = 0; i < 4; ++i) {
            std::string hip_str = leg_prefix[i] + "hip_joint";
            std::string thigh_str = leg_prefix[i] + "upper_leg_joint";
            std::string calf_str = leg_prefix[i] + "lower_leg_joint";
            if (js_.find(hip_str) == js_.end()) continue;

            double p_contact = std::clamp(contact_probs_[i], 0.001, 1.0);
            int foot_state_offset = 6 + i*3;
            bool is_stance = (p_contact > 0.5); 

            Eigen::Vector3d hip_body;
            hip_body(0) = (i < 2) ?  LEG_X_DIST : -LEG_X_DIST;
            hip_body(1) = (i == 0 || i == 2) ? LEG_Y_DIST : -LEG_Y_DIST;
            hip_body(2) = 0.0;

            Eigen::Vector3d r_foot_body = forward_kinematics(js_[hip_str], js_[thigh_str], js_[calf_str], i);
            Eigen::Vector3d r_foot_world_meas = x_.segment<3>(0) + R_body_ * (hip_body + r_foot_body);
            
            if (is_stance && !prev_stance_[i]) {
                // Foot just touched the floor, anchor its global position
                x_.segment<3>(foot_state_offset) = r_foot_world_meas;
                x_(foot_state_offset + 2) = 0.0; 
                P_.block<3, STATE_SIZE>(foot_state_offset, 0).setZero();
                P_.block<STATE_SIZE, 3>(0, foot_state_offset).setZero();
                P_.block<3, 3>(foot_state_offset, foot_state_offset) = Eigen::Matrix3d::Identity() * 0.01;
            }
            prev_stance_[i] = is_stance; 

            // THE BAYESIAN FUSION: Scale the Measurement Covariance (R) by the Bayesian Probability!
            // If p_contact is 1.0, variance is tiny (trust kinematics). 
            // If p_contact is 0.01, variance explodes to infinity (ignore kinematics).
            Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * (0.01 / p_contact); 
            Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * (0.05 / p_contact); 

            // 1. POSITION UPDATE
            Eigen::Matrix<double, 3, STATE_SIZE> H_pos = Eigen::Matrix<double, 3, STATE_SIZE>::Zero();
            H_pos.block<3,3>(0, 0)                 = -Eigen::Matrix3d::Identity(); 
            H_pos.block<3,3>(0, foot_state_offset) =  Eigen::Matrix3d::Identity(); 

            Eigen::Vector3d z_meas = R_body_ * (hip_body + r_foot_body);
            Eigen::Vector3d h_x = x_.segment<3>(foot_state_offset) - x_.segment<3>(0);
            Eigen::Vector3d y_pos = z_meas - h_x;

            Eigen::Matrix3d S_pos = H_pos * P_ * H_pos.transpose() + R_pos;
            Eigen::Matrix<double, STATE_SIZE, 3> K_pos = P_ * H_pos.transpose() * S_pos.inverse();
            
            x_.noalias() += K_pos * y_pos;
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> IKH_pos = I_sys - K_pos * H_pos;
            P_.noalias() = IKH_pos * P_ * IKH_pos.transpose() + K_pos * R_pos * K_pos.transpose();

            // 2. VELOCITY UPDATE
            Eigen::Vector3d q_dot(js_vel_[hip_str], js_vel_[thigh_str], js_vel_[calf_str]);
            Eigen::Matrix3d J = calcLegJacobian(js_[hip_str], js_[thigh_str], js_[calf_str], i);
            Eigen::Vector3d relative_foot_velocity = J * q_dot;

            Eigen::Vector3d v_body_meas = -R_body_ * relative_foot_velocity - w_world.cross(R_body_ * (hip_body + r_foot_body));
            
            Eigen::Matrix<double, 3, STATE_SIZE> H_vel = Eigen::Matrix<double, 3, STATE_SIZE>::Zero();
            H_vel.block<3,3>(0,3) = Eigen::Matrix3d::Identity(); 
            
            Eigen::Vector3d y_vel = v_body_meas - x_.segment<3>(3);
            Eigen::Matrix3d S_vel = H_vel * P_ * H_vel.transpose() + R_vel;
            
            Eigen::Matrix<double, STATE_SIZE, 3> K_vel = P_ * H_vel.transpose() * S_vel.inverse();
            x_.noalias() += K_vel * y_vel;
            
            // Record NIS for plotting
            if (i == 0 && is_stance) latest_nis = y_vel.transpose() * S_vel.inverse() * y_vel;

            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> IKH_vel = I_sys - K_vel * H_vel;
            P_.noalias() = IKH_vel * P_ * IKH_vel.transpose() + K_vel * R_vel * K_vel.transpose();
            
            // 3. ABSOLUTE Z-HEIGHT ANCHOR (Prevent Drift)
            Eigen::Matrix<double, 1, STATE_SIZE> H_z;
            H_z.setZero();
            H_z(0, foot_state_offset + 2) = 1.0;
            double y_z = 0.0 - x_(foot_state_offset + 2);
            double R_z = 0.005 / p_contact; 
            double S_z = (H_z * P_ * H_z.transpose())(0,0) + R_z;
            
            Eigen::Matrix<double, STATE_SIZE, 1> K_z = P_ * H_z.transpose() / S_z;
            x_.noalias() += K_z * y_z;
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> IKH_z = I_sys - K_z * H_z;
            P_.noalias() = IKH_z * P_ * IKH_z.transpose() + K_z * R_z * K_z.transpose();
        }

        // Clamp Covariance Matrix to prevent mathematical explosions
        if (P_(0,0) > 1.0) P_(0,0) = 1.0; 
        if (P_(1,1) > 1.0) P_(1,1) = 1.0; 
        for (int i = 0; i < 4; i++) {
            if (P_(6+i*3, 6+i*3) > 1.0)     P_(6+i*3, 6+i*3) = 1.0;
            if (P_(6+i*3+1, 6+i*3+1) > 1.0) P_(6+i*3+1, 6+i*3+1) = 1.0;
        }

        if (++print_counter_ >= 250) { 
            print_counter_ = 0;
            RCLCPP_INFO(this->get_logger(), "EKF STATE | Z: %.3f m | Vx: %.3f m/s | NIS: %.2f", x_(2), x_(3), latest_nis);
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now(); odom_msg.header.frame_id = "odom"; odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_(0); odom_msg.pose.pose.position.y = x_(1); odom_msg.pose.pose.position.z = x_(2);
        Eigen::Quaterniond q_out(R_body_);
        odom_msg.pose.pose.orientation.x = q_out.x(); odom_msg.pose.pose.orientation.y = q_out.y(); odom_msg.pose.pose.orientation.z = q_out.z(); odom_msg.pose.pose.orientation.w = q_out.w();
        
        // Rotate World Velocity to Body Frame for the Gait Planner to consume
        Eigen::Vector3d v_body_out = R_body_.transpose() * x_.segment<3>(3);
        odom_msg.twist.twist.linear.x = v_body_out.x();
        odom_msg.twist.twist.linear.y = v_body_out.y();
        odom_msg.twist.twist.linear.z = v_body_out.z();
        
        pub_odom_->publish(odom_msg);

        // Publish Diagnostics for Python Plotter
        std_msgs::msg::Float64MultiArray diag_msg;
        diag_msg.data = {P_(3,3), P_(4,4), P_(5,5), x_(0), x_(1), x_(2), latest_nis};
        pub_diag_->publish(diag_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_contact_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_diag_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<StateEstimate>()); rclcpp::shutdown(); return 0;
}