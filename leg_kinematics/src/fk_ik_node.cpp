#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <cmath>

class fk_ik_node : public rclcpp::Node
{
public:
    fk_ik_node() : Node("fk_ik_node")
    {
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&fk_ik_node::callback_js, this, std::placeholders::_1));

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_group_effort_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&fk_ik_node::timer_callback, this));
    }

private:
    //  Constants 
    const double L0 = 0.0955;   // hip lateral offset
    const double L1 = 0.213;    // thigh length
    const double L2 = 0.213;    // calf length

    std::unordered_map<std::string, double> actual_pos_;

    //  Structs 
    struct FootPos   { double x, y, z; };
    struct LegAngles { double hip, thigh, calf; };

    //  FK 
    FootPos fk(double q0, double q1, double q2)
    {
        double x    =  L1*sin(q1) + L2*sin(q1+q2);
        double z2d  = -L1*cos(q1) - L2*cos(q1+q2);

        double y_full = L0*cos(q0)  + z2d*sin(q0);
        double z_full = -L0*sin(q0) + z2d*cos(q0);

        return {x, y_full, z_full};
    }

    //  IK 
    LegAngles ik(double x, double y, double z)
    {
        const double knee_dir = -1.0;

        // Step 1: hip joint + 3D→2D
        double hip  = atan2(y, -z) - atan2(L0, sqrt(y*y + z*z - L0*L0));  
        double z2d  = sqrt(y*y + z*z - L0*L0);

        // Step 2: law of cosines
        double calf  = knee_dir * acos((x*x + z2d*z2d - L1*L1 - L2*L2) / (2*L1*L2));
        double thigh = atan2(x, z2d) - atan2(L2*sin(calf), L1 + L2*cos(calf)); 

        return {hip, thigh, calf};
    }

    //  Callbacks 
    void callback_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for(size_t i = 0; i < msg->name.size(); i++)
            actual_pos_[msg->name[i]] = msg->position[i];
    }

    void timer_callback()
    {
        if(actual_pos_.empty()){
            RCLCPP_WARN(this->get_logger(), "Waiting for joint states...");
            return;
        }

        double q0 = actual_pos_["FL_hip_joint"];
        double q1 = actual_pos_["FL_thigh_joint"];
        double q2 = actual_pos_["FL_calf_joint"];

        //  FK: angles → foot position 
        FootPos fp = fk(q0, q1, q2);
        RCLCPP_INFO(this->get_logger(),
            "FK  →  x: %.4f  y: %.4f  z: %.4f", fp.x, fp.y, fp.z);

        //  IK: foot position → angles (round-trip check) 
        LegAngles ik_result = ik(fp.x, fp.y, fp.z);
        RCLCPP_INFO(this->get_logger(),
            "IK  →  hip: %.4f(%.4f)  thigh: %.4f(%.4f)  calf: %.4f(%.4f)",
            ik_result.hip,   q0,
            ik_result.thigh, q1,
            ik_result.calf,  q2);

        //  Error check 
        double err = fabs(ik_result.thigh - q1) + fabs(ik_result.calf - q2);
        if(err > 0.01)
            RCLCPP_WARN(this->get_logger(), "FK/IK mismatch! err = %.4f", err);
        else
            RCLCPP_INFO(this->get_logger(), "FK/IK consistent ✅ err = %.6f", err);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fk_ik_node>());
    rclcpp::shutdown();
    return 0;
}