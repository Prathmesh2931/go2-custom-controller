#include<rclcpp/rclcpp.hpp>
#include<trajectory_msgs/msg/joint_trajectory.hpp>
#include<trajectory_msgs/msg/joint_trajectory_point.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>   

class fk_node : public rclcpp ::Node
{
    public:
        fk_node() : Node("fk_node")
        {
            sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
                    std::bind(&fk_node::callback_js, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&fk_node::timer_callback, this));
        }

    private:
        std::unordered_map<std::string,double> actual_pos_;
        double L1 = 0.213;
        double L2 = 0.213;
        double hip_lateral = 0.0955; // fixed Y offset from hip to thigh


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
            double q0 = actual_pos_["lf_hip_joint"];
            double q1 = actual_pos_["lf_upper_leg_joint"];
            double q2 = actual_pos_["lf_lower_leg_joint"];

            double x = L1*sin(q1) + L2*sin(q1+q2);
            double z = -L1*cos(q1) - L2*cos(q1+q2);

            double y_full = hip_lateral*cos(q0) + z*sin(q0);
            double z_full = -hip_lateral*sin(q0) + z*cos(q0);

            RCLCPP_INFO(this->get_logger(), "End effector position: x = %f, Y = %f, z = %f", x, y_full, z_full);

        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
        rclcpp::TimerBase::SharedPtr timer_;
            
            
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fk_node>());
    rclcpp::shutdown();
    return 0;
}