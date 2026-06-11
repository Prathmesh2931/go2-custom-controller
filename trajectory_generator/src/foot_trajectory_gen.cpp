#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <cmath>

class foot_trajectory_gen : public rclcpp::Node
{
public:
    foot_trajectory_gen() : Node("foot_trajectory_gen")
    {
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&foot_trajectory_gen::callback_js, this, std::placeholders::_1));
        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_group_effort_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&foot_trajectory_gen::timer_callback, this));
    }

private:
    const double L0 = 0.0955;
    const double L1 = 0.213;
    const double L2 = 0.213;
    const double STAND_THIGH =  0.7;
    const double STAND_CALF  = -1.4;

    bool standing_   = false;
    int  stand_ticks_= 0;
    const int STAND_DURATION = 30;   // 3 sec
    double start_time_ = 0.0;

    struct LegAngles { double hip, thigh, calf; };
    std::unordered_map<std::string, double> actual_pos_;

    LegAngles ik(double x, double y, double z)
    {
        const double knee_dir = -1.0;
        double hip = atan2(y, -z) - atan2(L0, sqrt(y*y + z*z - L0*L0));
        double z2d = sqrt(y*y + z*z - L0*L0);
        double D   = std::clamp((x*x + z2d*z2d - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0);
        double calf  = knee_dir * acos(D);
        double thigh = atan2(x, z2d) - atan2(L2*sin(calf), L1 + L2*cos(calf));
        return {hip, thigh, calf};
    }

    void publish_traj(std::vector<double> positions)
    {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.nanosec = 100000000;
        msg.points.push_back(point);
        pub_->publish(msg);
    }

    void callback_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for(size_t i = 0; i < msg->name.size(); i++)
            actual_pos_[msg->name[i]] = msg->position[i];
    }

    void timer_callback()
    {
        if(actual_pos_.empty()) return;

        // ── Phase 1: stand still for 3 seconds ──
        if(!standing_)
        {
            publish_traj({
                0.0, STAND_THIGH, STAND_CALF,
                0.0, STAND_THIGH, STAND_CALF,
                0.0, STAND_THIGH, STAND_CALF,
                0.0, STAND_THIGH, STAND_CALF
            });
            stand_ticks_++;
            if(stand_ticks_ >= STAND_DURATION){
                standing_   = true;
                start_time_ = this->now().seconds();
                RCLCPP_INFO(this->get_logger(), "✅ Standing stable — starting circle");
            }
            return;
        }

        // ── Phase 2: FL leg circle, others hold stand ──
        double t = (this->now().seconds() - start_time_) * 0.5; // half speed

        double x =  0.04 * cos(t);           // small 3cm swing
        double y =  L0;
        double z = -0.25 + 0.04 * sin(t);    // small 4cm lift

        LegAngles a = ik(x, y, z);

        RCLCPP_INFO(this->get_logger(),
            "foot(%.3f,%.3f,%.3f) → hip:%.3f thigh:%.3f calf:%.3f",
            x, y, z, a.hip, a.thigh, a.calf);

        publish_traj({
            a.hip, a.thigh, a.calf,       // FL: circle
            0.0, STAND_THIGH, STAND_CALF, // FR: hold
            0.0, STAND_THIGH, STAND_CALF, // RL: hold
            0.0, STAND_THIGH, STAND_CALF  // RR: hold
        });
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<foot_trajectory_gen>());
    rclcpp::shutdown();
    return 0;
}