#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
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
        sub_contact_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/state_estimator/contact_states", 10, std::bind(&StateEstimate::cb_contact, this, std::placeholders::_1));
        
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/estimate", 10);
        pub_diag_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ekf/diagnostics", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&StateEstimate::tick, this));
        initialize_ekf();
        RCLCPP_INFO(get_logger(), "Production 18-State Position EKF Initialized.");
    }

private:
    static constexpr double HIP_OFFSET = 0.0955;
    static constexpr double THIGH_LEN  = 0.213;
    static constexpr double CALF_LEN   = 0.213;
    static constexpr double LEG_X_DIST = 0.1934;
    static constexpr double LEG_Y_DIST = 0.0465; 
    static constexpr double STANCE_Z   = -0.28;

    static constexpr int STATE_SIZE = 18;
    static constexpr int MEAS_SIZE = 3;

    Eigen::Matrix<double, STATE_SIZE, 1> x_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q_;

    std::unordered_map<std::string, double> js_;
    Eigen::Matrix3d R_body_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu_accel_ = Eigen::Vector3d::Zero();

    bool ready_ = false;
    rclcpp::Time last_time_;
    bool first_tick_ = true;
    
    int active_contacts_[4] = {1, 1, 1, 1}; 
    bool prev_stance_[4] = {false, false, false, false};
    Eigen::Vector3d prev_r_foot_body_[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    void initialize_ekf()
    {
        x_.setZero(); P_.setZero(); Q_.setZero();
        x_(2) = 0.28; 
        P_.diagonal().segment<3>(0).setConstant(0.001); 
        P_.diagonal().segment<3>(3).setConstant(0.01);  
        P_.diagonal().segment<12>(6).setConstant(0.01); 

        Q_.diagonal().segment<3>(0).setConstant(0.001); 
        Q_.diagonal().segment<3>(3).setConstant(1.2);   
        Q_.diagonal().segment<12>(6).setConstant(0.05); 
    }

    Eigen::Vector3d forward_kinematics(double q_hip, double q_thigh, double q_calf, int leg_index)
    {
        Eigen::Vector3d p_foot;
        double x_sign = (leg_index == 0 || leg_index == 1) ? 1.0 : -1.0;    
        double y_sign = (leg_index == 0 || leg_index == 2) ? 1.0 : -1.0;    
        double side_sign = (leg_index == 0 || leg_index == 2) ? 1.0 : -1.0; 

        double s1 = std::sin(q_hip);   double c1 = std::cos(q_hip);
        double s2 = std::sin(q_thigh); double c2 = std::cos(q_thigh);
        double s23 = std::sin(q_thigh + q_calf); double c23 = std::cos(q_thigh + q_calf);

        double x_leg = THIGH_LEN * s2 + CALF_LEN * s23;
        double y_leg = side_sign * HIP_OFFSET * c1 - (THIGH_LEN * c2 + CALF_LEN * c23) * s1;
        double z_leg = -side_sign * HIP_OFFSET * s1 - (THIGH_LEN * c2 + CALF_LEN * c23) * c1;

        p_foot(0) = x_sign * LEG_X_DIST + x_leg; 
        p_foot(1) = y_sign * LEG_Y_DIST + y_leg; 
        p_foot(2) = z_leg;
        return p_foot;
    }

    void cb_js(const sensor_msgs::msg::JointState::SharedPtr msg) {        
        for (size_t i = 0; i < msg->name.size(); ++i) js_[msg->name[i]] = msg->position[i];
        ready_ = true;
    }

    void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto& q = msg->orientation;
        imu_accel_ = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
        R_body_ = eigen_q.toRotationMatrix();
    }

    void cb_contact(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if(msg->data.size() == 4) {
            for(int i=0; i<4; ++i) active_contacts_[i] = msg->data[i];
        }
    }

    void tick()
    {
        if (!ready_) return;

        // FIX: Calculated and declared ahead of the inspection gates so it passes compile variables smoothly
        Eigen::Vector3d gravity(0.0, 0.0, 9.81);
        Eigen::Vector3d accel_world = R_body_ * imu_accel_ - gravity;

        // ── DIAGNOSTIC INSPECTOR: Expose the exact cause of the reset loop ──
        if (std::isnan(x_(0)) || std::abs(x_(0)) > 100.0 || std::abs(x_(2)) > 10.0 || P_(0,0) > 1e5) {
            RCLCPP_WARN(this->get_logger(), "=== EKF RESET TRIGGER DIAGNOSTICS ===");
            RCLCPP_WARN(this->get_logger(), "Body Position X: %f, Z: %f", x_(0), x_(2));
            RCLCPP_WARN(this->get_logger(), "Body Velocity X: %f, Z: %f", x_(3), x_(5));
            RCLCPP_WARN(this->get_logger(), "Position Covariance P(0,0): %f", P_(0,0));
            RCLCPP_WARN(this->get_logger(), "World Accel Z-Axis: %f", accel_world.z());
            RCLCPP_WARN(this->get_logger(), "======================================");

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
                    Eigen::Vector3d r_foot_body = forward_kinematics(js_[hip_str], js_[thigh_str], js_[calf_str], i);
                    x_.segment<3>(6 + i * 3) = x_.segment<3>(0) + R_body_ * r_foot_body;
                    prev_r_foot_body_[i] = r_foot_body;
                }
            }
            first_tick_ = false; return; 
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0.0 || dt > 0.1) dt = 0.02;

        x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * accel_world * dt * dt; 
        x_.segment<3>(3) += accel_world * dt; 

        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        P_.noalias() = F * P_ * F.transpose() + Q_;

        double latest_nis = 0.0;
        Eigen::Vector3d latest_y = Eigen::Vector3d::Zero();

        for (int i = 0; i < 4; ++i) {
            std::string hip_str = leg_prefix[i] + "hip_joint";
            std::string thigh_str = leg_prefix[i] + "upper_leg_joint";
            std::string calf_str = leg_prefix[i] + "lower_leg_joint";
            if (js_.find(hip_str) == js_.end()) continue;

            Eigen::Vector3d r_foot_body = forward_kinematics(js_[hip_str], js_[thigh_str], js_[calf_str], i);
            Eigen::Vector3d relative_foot_velocity = (r_foot_body - prev_r_foot_body_[i]) / dt;
            prev_r_foot_body_[i] = r_foot_body; 

            double phase_factor  = (active_contacts_[i] == 1) ? 1.0 : 0.0;
            double vel_factor    = std::exp(-std::pow(relative_foot_velocity.z(), 2) / (2.0 * 0.08 * 0.08));
            double height_factor = std::exp(-std::pow(r_foot_body.z() - STANCE_Z, 2) / (2.0 * 0.05 * 0.05));
            
            double p_contact = (0.5 * phase_factor) + (0.3 * vel_factor) + (0.2 * height_factor);
            p_contact = std::clamp(p_contact, 0.01, 1.0);

            int foot_state_offset = 6 + i*3;
            bool is_stance = (p_contact > 0.35); 

            if (is_stance && !prev_stance_[i]) {
                x_.segment<3>(foot_state_offset) = x_.segment<3>(0) + R_body_ * r_foot_body;
                P_.block<3, STATE_SIZE>(foot_state_offset, 0).setZero();
                P_.block<STATE_SIZE, 3>(0, foot_state_offset).setZero();
                P_.block<3, 3>(foot_state_offset, foot_state_offset) = Eigen::Matrix3d::Identity() * 0.05;
            }
            prev_stance_[i] = is_stance; 

            Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> H = Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE>::Zero();
            H.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();            
            H.block<3,3>(0,foot_state_offset) = Eigen::Matrix3d::Identity(); 

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (0.002 / p_contact); 
            Eigen::Vector3d y = -(x_.segment<3>(foot_state_offset)) - x_.segment<3>(0) + (R_body_ * r_foot_body); 
            Eigen::Matrix3d S = H * P_ * H.transpose() + R;
            
            double mahalanobis = y.transpose() * S.inverse() * y;
            if (is_stance && mahalanobis > 100.0) {
                latest_nis = mahalanobis;
                continue; 
            }

            Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K = P_ * H.transpose() * S.inverse();
            x_.noalias() += K * y;
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
            P_.noalias() = (I - K * H) * P_;
            latest_nis = mahalanobis;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now(); odom_msg.header.frame_id = "odom"; odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_(0); odom_msg.pose.pose.position.y = x_(1); odom_msg.pose.pose.position.z = x_(2);
        Eigen::Quaterniond q_out(R_body_);
        odom_msg.pose.pose.orientation.x = q_out.x(); odom_msg.pose.pose.orientation.y = q_out.y(); odom_msg.pose.pose.orientation.z = q_out.z(); odom_msg.pose.pose.orientation.w = q_out.w();
        odom_msg.twist.twist.linear.x = x_(3); odom_msg.twist.twist.linear.y = x_(4); odom_msg.twist.twist.linear.z = x_(5);
        pub_odom_->publish(odom_msg);

        std_msgs::msg::Float64MultiArray diag_msg;
        diag_msg.data = {P_(0,0), P_(1,1), P_(2,2), x_(0), x_(1), x_(2), latest_nis};
        pub_diag_->publish(diag_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_contact_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_diag_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<StateEstimate>()); rclcpp::shutdown(); return 0;
}