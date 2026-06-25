#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cmath>
#include <algorithm>
#include <vector>

class TrotGait : public rclcpp::Node
{
public:
    TrotGait() : Node("trot_gait")
    {
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TrotGait::cb_cmd, this, std::placeholders::_1));

        pub_nominal_positions_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gait/planned_positions", 10);
        pub_contact_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/state_estimator/contact_states", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TrotGait::tick, this));
        
        RCLCPP_INFO(get_logger(), "PLANNER ACTIVE: 4-LEG RESTING STAND FIXED.");
    }

private:
    const double DIR_MULTIPLIER = 1.0; // Change to -1.0 if it moonwalks

    const double HIP_OFFSET = 0.0955;
    const double THIGH_LEN  = 0.213;
    const double CALF_LEN   = 0.213;

    const double STANCE_Z     = -0.32; // Tall stand
    const double LIFT_H       =  0.06;  
    const double MAX_STEP_LEN =  0.10; 
    const double CMD_DEADBAND =  0.10;   

    double phase_ = 0.0;
    double raw_vx_ = 0.0;

    struct Angles { double hip, thigh, calf; };

    Angles ik(double x, double y, double z)
    {
        Angles a;
        double r2d = std::sqrt(y*y + z*z);
        if (r2d < HIP_OFFSET) r2d = HIP_OFFSET + 0.001; 
        double hip_sign = (y >= 0.0) ? 1.0 : -1.0;
        
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
        a.thigh = alpha + beta;

        return a;
    }

    void cb_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
        raw_vx_ = msg->linear.x * DIR_MULTIPLIER; 
    }

    void tick()
    {
        double speed = std::abs(raw_vx_);
        bool is_moving = speed > CMD_DEADBAND;

        double step_len = std::clamp(raw_vx_ * 0.4, -MAX_STEP_LEN, MAX_STEP_LEN);

        std::vector<double> nominal_positions(12, 0.0);
        std::vector<int> live_contacts(4, 1);

        if (!is_moving) {
            phase_ = 0.0; 
            // THE FIX: Force all 4 legs to firmly plant on the ground!
            for (int i = 0; i < 4; ++i) {
                live_contacts[i] = 1; // STANCE
                double x = 0.0;
                double z = STANCE_Z; 
                double y = (i == 0 || i == 2) ? 0.0955 : -0.0955; 
                
                Angles a = ik(x, y, z);
                nominal_positions[i*3 + 0] = a.hip;
                nominal_positions[i*3 + 1] = a.thigh;
                nominal_positions[i*3 + 2] = a.calf;
            }
        } else {
            // Normal Trot Logic
            phase_ += 0.02 * 1.5; 
            if (phase_ >= 1.0) phase_ -= 1.0;

            double leg_phases[4];
            leg_phases[0] = phase_;                                 // LF
            leg_phases[1] = std::fmod(phase_ + 0.5, 1.0);           // RF
            leg_phases[2] = std::fmod(phase_ + 0.5, 1.0);           // LH
            leg_phases[3] = phase_;                                 // RH

            for (int i = 0; i < 4; ++i) {
                double p = leg_phases[i];
                double x, z;
                
                if (p < 0.4) { 
                    live_contacts[i] = 0; // Swing
                    double t = p / 0.4;
                    x = -step_len/2.0 + step_len * (0.5 - 0.5*std::cos(M_PI * t)); 
                    z = STANCE_Z + LIFT_H * std::sin(M_PI * t); 
                } else { 
                    live_contacts[i] = 1; // Stance
                    double t = (p - 0.4) / 0.6;
                    x = step_len/2.0 - step_len * t; 
                    z = STANCE_Z; 
                }
                
                double y = (i == 0 || i == 2) ? 0.0955 : -0.0955; 

                Angles a = ik(x, y, z);
                nominal_positions[i*3 + 0] = a.hip;
                nominal_positions[i*3 + 1] = a.thigh;
                nominal_positions[i*3 + 2] = a.calf;
            }
        }

        std_msgs::msg::Int32MultiArray contact_msg; 
        contact_msg.data = live_contacts;
        pub_contact_->publish(contact_msg);

        std_msgs::msg::Float64MultiArray nominal_payload; 
        nominal_payload.data = nominal_positions;
        pub_nominal_positions_->publish(nominal_payload);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_nominal_positions_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_contact_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TrotGait>()); rclcpp::shutdown(); return 0;
}