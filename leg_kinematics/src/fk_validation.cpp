// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>

// #include "go2_mpc_controller/robot_dynamics.hpp"

// class FKValidator : public rclcpp::Node
// {
// public:

//     FKValidator()
//     : Node("fk_validator")
//     {
//         joint_sub_ =
//             create_subscription<sensor_msgs::msg::JointState>(
//                 "/joint_states",
//                 10,
//                 std::bind(
//                     &FKValidator::jointCallback,
//                     this,
//                     std::placeholders::_1));

//         RCLCPP_INFO(
//             get_logger(),
//             "FK VALIDATOR STARTED");
//     }

// private:

//     void jointCallback(
//         const sensor_msgs::msg::JointState::SharedPtr msg)
//     {
//         static int counter = 0;

//         if(++counter < 50)
//             return;

//         counter = 0;

//         validateLeg(msg, "lf", 0);
//         validateLeg(msg, "rf", 1);
//         validateLeg(msg, "lh", 2);
//         validateLeg(msg, "rh", 3);

//         RCLCPP_INFO(
//             get_logger(),
//             "--------------------------------");
//     }

//     void validateLeg(
//         const sensor_msgs::msg::JointState::SharedPtr msg,
//         const std::string& leg,
//         int leg_index)
//     {
//         double q1 = getJoint(msg, leg + "_hip_joint");
//         double q2 = getJoint(msg, leg + "_upper_leg_joint");
//         double q3 = getJoint(msg, leg + "_lower_leg_joint");

//         Eigen::Vector3d foot =
//             go2_physics::calcFootPosition(
//                 q1,
//                 q2,
//                 q3,
//                 leg_index);

//         RCLCPP_INFO(
//             get_logger(),
//             "%s | q=[%.3f %.3f %.3f]  FK=[%.3f %.3f %.3f]",
//             leg.c_str(),
//             q1,
//             q2,
//             q3,
//             foot.x(),
//             foot.y(),
//             foot.z());
//     }

//     double getJoint(
//         const sensor_msgs::msg::JointState::SharedPtr msg,
//         const std::string& name)
//     {
//         auto it =
//             std::find(
//                 msg->name.begin(),
//                 msg->name.end(),
//                 name);

//         if(it == msg->name.end())
//             return 0.0;

//         size_t idx =
//             std::distance(
//                 msg->name.begin(),
//                 it);

//         return msg->position[idx];
//     }

//     rclcpp::Subscription<
//         sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
// };

int main(int argc,char** argv)
{
    // rclcpp::init(argc,argv);

    // rclcpp::spin(
    //     std::make_shared<FKValidator>());

    // rclcpp::shutdown();

    return 0;
}