#include<rclcpp/rclcpp.hpp>
#include<trajectory_msgs/msg/joint_trajectory.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <chrono>

class trajectory_inspector : public rclcpp ::Node
{
    public:
        trajectory_inspector() : Node("trajectory_inspector")
        {
            sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("/joint_group_effort_controller/joint_trajectory", 10,
                    std::bind(&trajectory_inspector::callback, this, std::placeholders::_1));   

            sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
                    std::bind(&trajectory_inspector::callback_js, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&trajectory_inspector::timer_callback, this));
        }

    private:
        std::unordered_map<std::string,double> desired_pos_;
        std::unordered_map<std::string,double> actual_pos_;

        void callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
        {
            for(size_t i=0; i<msg->points.size();i++)
            {
                // RCLCPP_INFO(this->get_logger(), "point %d: time_from_start = %f", (int)i, msg->points[i].time_from_start.sec + msg->points[i].time_from_start.nanosec*1e-9);
                for(size_t j=0; j<msg->points[i].positions.size();j++)
                {
                    if (msg->joint_names[j] == "lf_upper_leg_joint")
                    {
                        // RCLCPP_INFO(this->get_logger(), "  position %d: %f", (int)j, msg->points[i].positions[j]);
                        desired_pos_[msg->joint_names[j]] = msg->points[i].positions[j];
                    }
                }
            }
        }

        void callback_js(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            for(size_t i=0; i<msg->name.size();i++)
            {
                if  (msg->name[i] == "lf_upper_leg_joint")
                {
                    // RCLCPP_INFO(this->get_logger(), "joint %s: position = %f", msg->name[i].c_str(), msg->position[i]);
                    actual_pos_[msg->name[i]] = msg->position[i];
                }
            }
        }

        void timer_callback()
        {
            std::string joint = "lf_upper_leg_joint";
        
            if(desired_pos_.count(joint) &&
            actual_pos_.count(joint))
            {
                double qd = desired_pos_[joint];
                double q  = actual_pos_[joint];

                double error = qd - q;

                RCLCPP_INFO(
                    this->get_logger(),
                    "%s | qd=%.4f q=%.4f err=%.4f",
                    joint.c_str(),
                    qd,
                    q,
                    error);
            }
        }

        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
        rclcpp::TimerBase::SharedPtr timer_;
};  

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<trajectory_inspector>());
    rclcpp::shutdown();
    return 0;
}