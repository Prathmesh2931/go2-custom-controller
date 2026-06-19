#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

class ConvexMPCNode : public rclcpp::Node
{
public:
    ConvexMPCNode() : Node("convex_mpc_node")
    {
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom/estimate", 10, std::bind(&ConvexMPCNode::cb_odom, this, std::placeholders::_1));
            
        sub_contact_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/state_estimator/contact_states", 10, std::bind(&ConvexMPCNode::cb_contact, this, std::placeholders::_1));

        sub_nominal_traj_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gait/planned_positions", 10, std::bind(&ConvexMPCNode::cb_nominal_traj, this, std::placeholders::_1));

        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ConvexMPCNode::cb_js, this, std::placeholders::_1));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ConvexMPCNode::cb_cmd, this, std::placeholders::_1));

        pub_joint_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_group_effort_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&ConvexMPCNode::control_loop, this));

        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };

        std::fill(std::begin(smooth_d_thigh_), std::end(smooth_d_thigh_), 0.0);
        std::fill(std::begin(smooth_d_calf_), std::end(smooth_d_calf_), 0.0);
    }

private:
    static constexpr double MASS = 12.0; 
    static constexpr double K_YAML = 100.0; 
    
    // Smooth command tracking coefficients matching the gait generation layer
    static constexpr double CMD_ALPHA = 0.08; 
    static constexpr double TRIM_ALPHA = 0.15; 
    
    // ── SLEW LIMIT: Maximum allowable coordinate trim shift per frame (rad) ──
    static constexpr double MAX_TRIM_SLEW = 0.015; 

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_contact_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_nominal_traj_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d p_b_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_b_ = Eigen::Vector3d::Zero();
    
    int contacts_[4] = {1, 1, 1, 1};
    std::vector<double> nominal_positions_ = std::vector<double>(12, 0.0);
    std::unordered_map<std::string, double> current_joint_positions_;
    std::vector<std::string> joint_names_;

    double raw_vx_ = 0.0;
    double raw_vy_ = 0.0;
    double smooth_vx_ = 0.0;
    double smooth_vy_ = 0.0;

    double smooth_d_thigh_[4];
    double smooth_d_calf_[4];

    void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        p_b_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        v_b_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    }

    void cb_contact(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) { for (int i = 0; i < 4; ++i) contacts_[i] = msg->data[i]; }
    }

    void cb_nominal_traj(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 12) { nominal_positions_ = msg->data; }
    }

    void cb_js(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) current_joint_positions_[msg->name[i]] = msg->position[i];
    }

    void cb_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
        raw_vx_ = msg->linear.x;
        raw_vy_ = msg->linear.y;
    }

    void control_loop()
    {
        if (nominal_positions_[0] == 0.0 && nominal_positions_[1] == 0.0) return;
        if (current_joint_positions_.empty()) return;

        // ── FIX 1: Ramp targets via smooth low-pass filters to mirror gait node tracking ──
        smooth_vx_ += CMD_ALPHA * (raw_vx_ - smooth_vx_);
        smooth_vy_ += CMD_ALPHA * (raw_vy_ - smooth_vy_);

        Eigen::VectorXd error = Eigen::VectorXd::Zero(12);
        error(5) = p_b_.z() - 0.28;               
        error(9) = v_b_.x() - smooth_vx_;         
        error(10) = v_b_.y() - smooth_vy_;        

        Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(12, 12);
        double dt = 0.02;
        for (int i = 0; i < 4; ++i) {
            if (contacts_[i] == 1) { 
                Bd(9, i * 3 + 0) = dt / MASS; 
                Bd(10, i * 3 + 1) = dt / MASS;
                Bd(11, i * 3 + 2) = dt / MASS; 
            }
        }

        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12) * 40.0;
        Q(5,5) = 350.0; 
        Q(9,9) = 180.0; 
        Eigen::MatrixXd R_f = Eigen::MatrixXd::Identity(12, 12) * 1e-3;

        Eigen::MatrixXd MatInverse = (Bd.transpose() * Q * Bd + R_f).inverse();
        Eigen::VectorXd forces = -MatInverse * Bd.transpose() * Q * error;

        int stance_count = 0;
        for (int i = 0; i < 4; ++i) if (contacts_[i] == 1) stance_count++;
        double fz_ff = (MASS * 9.81) / std::max(stance_count, 1);

        trajectory_msgs::msg::JointTrajectory command_msg;
        command_msg.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.resize(12, 0.0);

        for (int i = 0; i < 4; ++i) {
            int idx = i * 3;
            if (contacts_[i] == 0) {
                pt.positions[idx + 0] = nominal_positions_[idx + 0];
                pt.positions[idx + 1] = nominal_positions_[idx + 1];
                pt.positions[idx + 2] = nominal_positions_[idx + 2];
                smooth_d_thigh_[i] = 0.0; smooth_d_calf_[i] = 0.0; 
            } else {
                double f_x = forces(idx + 0);
                double f_z = forces(idx + 2) + fz_ff;

                double target_d_thigh = -f_x * 0.0015; 
                double target_d_calf  = -f_z * (1.0 / K_YAML); 

                // ── FIX 2: Enforce strict step velocity limitations via Slew Rate Saturation ──
                double delta_thigh = std::clamp(target_d_thigh - smooth_d_thigh_[i], -MAX_TRIM_SLEW, MAX_TRIM_SLEW);
                double delta_calf  = std::clamp(target_d_calf - smooth_d_calf_[i], -MAX_TRIM_SLEW, MAX_TRIM_SLEW);

                smooth_d_thigh_[i] += delta_thigh;
                smooth_d_calf_[i]  += delta_calf;

                pt.positions[idx + 0] = nominal_positions_[idx + 0]; 
                pt.positions[idx + 1] = nominal_positions_[idx + 1] + smooth_d_thigh_[i];
                pt.positions[idx + 2] = nominal_positions_[idx + 2] + smooth_d_calf_[i];
            }
        }

        pt.time_from_start.nanosec = 20'000'000;
        command_msg.points.push_back(pt);
        pub_joint_trajectory_->publish(command_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ConvexMPCNode>()); rclcpp::shutdown(); return 0;
}