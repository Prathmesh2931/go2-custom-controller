#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <algorithm>

class TrotGait : public rclcpp::Node
{
public:
    TrotGait() : Node("trot_gait")
    {
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TrotGait::cb_cmd, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/ground_truth", 10, std::bind(&TrotGait::cb_odom, this, std::placeholders::_1));

        pub_nominal_positions_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gait/planned_positions", 10);
        pub_contact_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/state_estimator/contact_states", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(4), std::bind(&TrotGait::tick, this));
        
        RCLCPP_INFO(get_logger(), "ADVANCED TROT GAIT ONLINE: WIDER STANCE APPLIED!");
    }

private:
    const double DIR_MULTIPLIER = 1.0; 
    
    const double DT = 0.004;          
    const double GAIT_FREQ = 1.8;     

    const double HIP_X      = 0.1934; 
    const double HIP_OFFSET = 0.0955; 
    const double THIGH_LEN  = 0.213;
    const double CALF_LEN   = 0.213;

    // FIX: Widened the lateral stance base from 0.14m to 0.155m. 
    // This creates a broader support polygon, significantly improving sideways stability during the trot!
    const double HIP_Y      = 0.155;   
    const double STANCE_Z   = -0.28; 
    const double LIFT_H     = 0.06;   

    const double MAX_STRIDE_X = 0.22; 

    double phase_ = 0.0;
    
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_wz_ = 0.0; 
    double filtered_vx_ = 0.0;
    double filtered_vy_ = 0.0;
    double filtered_wz_ = 0.0;

    int log_counter_ = 0;
    double actual_vx_ = 0.0;
    double actual_vy_ = 0.0;
    double actual_wz_ = 0.0;

    struct Angles { double hip, thigh, calf; };

    Angles ik(double x, double y, double z, int leg_index = 0)
    {
        Angles a;
        double hip_sign = (leg_index == 0 || leg_index == 2) ? 1.0 : -1.0;
        
        double r2d = std::sqrt(y*y + z*z);
        if (r2d < HIP_OFFSET) r2d = HIP_OFFSET + 0.001; 
        
        double inner = std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET);
        a.hip = std::atan2(y, -z) - hip_sign * std::atan2(HIP_OFFSET, std::sqrt(inner));

        double L2 = std::sqrt(inner); 
        double leg_distance = std::sqrt(x*x + L2*L2); 
        leg_distance = std::clamp(leg_distance, 0.01, THIGH_LEN + CALF_LEN - 0.001);
        
        double cos_calf = (leg_distance*leg_distance - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * CALF_LEN);
        a.calf = -std::acos(std::clamp(cos_calf, -1.0, 1.0)); 

        double alpha = std::atan2(x, L2);
        
        double cos_beta = (THIGH_LEN*THIGH_LEN + leg_distance*leg_distance - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * leg_distance);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        
        a.thigh = beta - alpha;

        return a;
    }

    void cb_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vx_ = msg->linear.x * DIR_MULTIPLIER; 
        cmd_vy_ = msg->linear.y * DIR_MULTIPLIER;
        cmd_wz_ = msg->angular.z * DIR_MULTIPLIER;
    }
    
    void cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        actual_vx_ = msg->twist.twist.linear.x;
        actual_wz_ = msg->twist.twist.angular.z;
        actual_vy_ = msg->twist.twist.linear.y;
    }

    void tick()
    {
        filtered_vx_ += 0.02 * (cmd_vx_ - filtered_vx_);
        filtered_vy_ += 0.02 * (cmd_vy_ - filtered_vy_);
        filtered_wz_ += 0.02 * (cmd_wz_ - filtered_wz_);

        double speed = std::sqrt(filtered_vx_*filtered_vx_ + filtered_vy_*filtered_vy_ + filtered_wz_*filtered_wz_);
        bool is_moving = speed > 0.05;

        std::vector<double> nominal_positions(12, 0.0);
        std::vector<int> live_contacts(4, 1);
        std::vector<double> current_X(4, 0.0);
        std::vector<double> current_Y(4, 0.0);
        std::vector<double> current_Z(4, 0.0);

        if (is_moving) {
            phase_ = std::fmod(phase_ + (GAIT_FREQ * DT), 1.0);
        } else {
            phase_ = 0.0; 
        }

        double diagnostic_raw_stride = 0.0;
        double diagnostic_clamped_stride = 0.0;

        for (int i = 0; i < 4; ++i) {
            double rx = (i < 2) ? HIP_X : -HIP_X;
            double base_y = (i % 2 == 0) ? HIP_Y : -HIP_Y; 
            
            double base_x = (i < 2) ? 0.03 : -0.03; 
            
            double local_x = base_x;
            double local_y = base_y;
            double local_z = STANCE_Z;

            if (is_moving) {
                double T_stance = 0.6 / GAIT_FREQ; 

                double foot_cmd_vx = filtered_vx_ - filtered_wz_ * base_y;
                double foot_cmd_vy = filtered_vy_ + filtered_wz_ * rx; 

                double raw_stride_x = foot_cmd_vx * T_stance * 1.5;
                double stride_x = std::clamp(raw_stride_x, -MAX_STRIDE_X, MAX_STRIDE_X);
                double stride_y = std::clamp(foot_cmd_vy * T_stance * 1.5, -0.10, 0.10);

                if (i == 0) { 
                    diagnostic_raw_stride = raw_stride_x;
                    diagnostic_clamped_stride = stride_x;
                }

                double offset_x = 0.0; 
                double offset_y = 0.0;
                
                double p = (i == 0 || i == 3) ? phase_ : std::fmod(phase_ + 0.5, 1.0);
                
                if (p < 0.4) { 
                    live_contacts[i] = 0; 
                    double t = p / 0.4;
                    local_x = base_x - stride_x/2.0 + stride_x * (0.5 - 0.5*std::cos(M_PI * t)) + offset_x;
                    local_y = base_y - stride_y/2.0 + stride_y * (0.5 - 0.5*std::cos(M_PI * t)) + offset_y;
                    local_z = STANCE_Z + LIFT_H * std::sin(M_PI * t); 
                } else { 
                    live_contacts[i] = 1; 
                    double t = (p - 0.4) / 0.6;
                    local_x = base_x + stride_x/2.0 - stride_x * t + offset_x; 
                    local_y = base_y + stride_y/2.0 - stride_y * t + offset_y;
                    local_z = STANCE_Z; 
                }
            } else {
                live_contacts[i] = 1;
            }

            current_X[i] = local_x;
            current_Y[i] = local_y;
            current_Z[i] = local_z;

            Angles a = ik(local_x, local_y, local_z, i);
            nominal_positions[i*3 + 0] = a.hip;
            nominal_positions[i*3 + 1] = a.thigh;
            nominal_positions[i*3 + 2] = a.calf;
        }

        std_msgs::msg::Int32MultiArray contact_msg; 
        contact_msg.data = live_contacts;
        pub_contact_->publish(contact_msg);

        std_msgs::msg::Float64MultiArray nominal_payload; 
        nominal_payload.data = nominal_positions;
        pub_nominal_positions_->publish(nominal_payload);

        if (++log_counter_ >= 250) { 
            log_counter_ = 0;
            RCLCPP_INFO(get_logger(), "\n--- KINEMATIC FEASIBILITY & GAIT PLANNER ---");
            RCLCPP_INFO(get_logger(), "CMD VX: %.3f m/s | CMD WZ: %.3f rad/s | PHASE: %.2f", filtered_vx_, filtered_wz_, phase_);
            
            if (is_moving) {
                double utilization = (std::abs(diagnostic_raw_stride) / MAX_STRIDE_X) * 100.0;
                if (utilization <= 100.0) {
                    RCLCPP_INFO(get_logger(), "[REACHABILITY] Stride X: %.3fm (%.1f%% of max limit) | STATUS: [SAFE]", 
                        diagnostic_raw_stride, utilization);
                } else {
                    RCLCPP_WARN(get_logger(), "[REACHABILITY] Stride X: %.3fm (%.1f%% of max limit) | STATUS: [WARNING - CLAMPED TO %.3fm]", 
                        diagnostic_raw_stride, utilization, MAX_STRIDE_X);
                }
            } else {
                RCLCPP_INFO(get_logger(), "[REACHABILITY] Stride X: 0.000m (0.0%% of max limit) | STATUS: [STANDING]");
            }

            RCLCPP_INFO(get_logger(), "LF [C:%d] X=%.3f, Y=%.3f, Z=%.3f | HIP=%.3f, TH=%.3f, CA=%.3f", 
                live_contacts[0], current_X[0], current_Y[0], current_Z[0], nominal_positions[0], nominal_positions[1], nominal_positions[2]);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_nominal_positions_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_contact_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TrotGait>()); rclcpp::shutdown(); return 0;
}