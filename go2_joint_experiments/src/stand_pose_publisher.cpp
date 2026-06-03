#include<rclcpp/rclcpp.hpp>
#include<trajectory_msgs/msg/joint_trajectory.hpp>
#include<trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <chrono>

class stand_pose_publisher : public rclcpp ::Node
{
    public:
        stand_pose_publisher() : Node("stand_pose_publisher")
        {
            pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_group_effort_controller/joint_trajectory", 10);

            sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
                    std::bind(&stand_pose_publisher::callback_js, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&stand_pose_publisher::timer_callback, this));
            // double q = 0.5;
        }

    private:
        std::unordered_map<std::string,double> actual_pos_;

        double q = 0.5;
        double upper = 1.8;
        double lower = -1.8;
        bool increasing = true;
        void callback_js(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            for(size_t i=0; i<msg->name.size();i++)
            {
                actual_pos_[msg->name[i]] = msg->position[i];
            }
        }

        void timer_callback()
        {   
            // RCLCPP_INFO(this->get_logger(), "Publishing stand pose trajectory");
            trajectory_msgs::msg::JointTrajectory msg;
            msg.joint_names = {"lf_hip_joint","lf_upper_leg_joint", "lf_lower_leg_joint", 
                               "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
                               "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", 
                               "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            
            if (q >= upper)
            {
                increasing = false;
            }
            else if (q <= lower)
            {
                increasing = true;
            }

            // move based on direction
            if (increasing)
                q += 0.1;
            else
                q -= 0.1;
            
            double t = rclcpp::Clock().now().seconds();

            double hip   = 0.0;

            double upper = 0.8 + 0.2*sin(t);

            double lower = -1.6 - 0.4*sin(t);

            double upper_opp = 0.8 + 0.2*sin(t+M_PI);
            double lower_opp = -1.6 - 0.4*sin(t+M_PI);
            point.positions = {
                hip, upper, lower, // left front leg
                hip , upper_opp, lower_opp, // right front leg
                hip, upper_opp , lower_opp,  // left hind leg
                hip, upper , lower  // right hind leg
            };

            point.time_from_start.sec = 1;
            msg.points.push_back(point);
            RCLCPP_INFO(this->get_logger(),"lower:  %f upper: %f", lower, upper);
            pub_->publish(msg);
        }

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stand_pose_publisher>());
    rclcpp::shutdown();
    return 0;
}