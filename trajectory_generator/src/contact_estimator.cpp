#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#include "go2_mpc_controller/robot_model.hpp"

using namespace go2_physics;

class ContactEstimator : public rclcpp::Node {
public:
    ContactEstimator() : Node("contact_estimator") {
        
        sub_lf_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "/gazebo/lf_bumper", 10, std::bind(&ContactEstimator::lf_cb, this, std::placeholders::_1));
        sub_rf_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "/gazebo/rf_bumper", 10, std::bind(&ContactEstimator::rf_cb, this, std::placeholders::_1));
        sub_lh_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "/gazebo/lh_bumper", 10, std::bind(&ContactEstimator::lh_cb, this, std::placeholders::_1));
        sub_rh_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
            "/gazebo/rh_bumper", 10, std::bind(&ContactEstimator::rh_cb, this, std::placeholders::_1));

        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ContactEstimator::js_cb, this, std::placeholders::_1));
        sub_gait_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/gait/planned_contacts", 10, std::bind(&ContactEstimator::gait_cb, this, std::placeholders::_1));

        pub_states_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/state_estimator/contact_states", 10);
        pub_probs_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_estimator/contact_probs", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4), std::bind(&ContactEstimator::tick, this));

        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };
            
        RCLCPP_INFO(this->get_logger(), "SHADOW ESTIMATOR ONLINE: KINEMATIC BOUNDARIES ACTIVE.");
    }

private:
    std::mutex mtx_;
    std::vector<int> gt_contacts_ = std::vector<int>(4, 1);        
    std::vector<int> planned_contacts_ = std::vector<int>(4, 1);   
    
    sensor_msgs::msg::JointState last_js_;
    bool js_received_ = false;

    // Initialize arrays correctly to avoid the garbage memory bug!
    std::vector<double> filtered_dq_ = std::vector<double>(12, 0.0);
    std::vector<double> log_odds_ = std::vector<double>(4, 0.0);
    std::vector<double> probs_ = std::vector<double>(4, 0.5);
    std::vector<double> latest_slip_ = std::vector<double>(4, 0.0);

    int log_counter_ = 0;
    std::vector<std::string> joint_names_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_probs_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr sub_lf_, sub_rf_, sub_lh_, sub_rh_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_gait_;

    void lf_cb(const gazebo_msgs::msg::ContactsState::SharedPtr msg) { std::lock_guard<std::mutex> lock(mtx_); gt_contacts_[0] = !msg->states.empty() ? 1 : 0; }
    void rf_cb(const gazebo_msgs::msg::ContactsState::SharedPtr msg) { std::lock_guard<std::mutex> lock(mtx_); gt_contacts_[1] = !msg->states.empty() ? 1 : 0; }
    void lh_cb(const gazebo_msgs::msg::ContactsState::SharedPtr msg) { std::lock_guard<std::mutex> lock(mtx_); gt_contacts_[2] = !msg->states.empty() ? 1 : 0; }
    void rh_cb(const gazebo_msgs::msg::ContactsState::SharedPtr msg) { std::lock_guard<std::mutex> lock(mtx_); gt_contacts_[3] = !msg->states.empty() ? 1 : 0; }

    void js_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        last_js_ = *msg;
        js_received_ = true;
    }

    void gait_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) {
            std::lock_guard<std::mutex> lock(mtx_);
            for (int i=0; i<4; i++) planned_contacts_[i] = msg->data[i];
        }
    }

    void tick() {
        std::vector<int> true_contacts(4, 1);
        std::vector<int> prior_contacts(4, 1);
        sensor_msgs::msg::JointState js;
        bool has_js;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            true_contacts = gt_contacts_;
            prior_contacts = planned_contacts_;
            js = last_js_;
            has_js = js_received_;
        }

        // =================================================================================
        // THE LIFELINE: Pass pure perfect Gazebo truth directly to the PD Tracker
        // =================================================================================
        std_msgs::msg::Int32MultiArray state_msg;
        state_msg.data = true_contacts;
        pub_states_->publish(state_msg);

        if (!has_js || js.name.empty() || js.position.size() < 12 || js.velocity.size() < 12) return;

        std::vector<double> q(12, 0.0);
        for (int i = 0; i < 12; ++i) {
            auto it = std::find(js.name.begin(), js.name.end(), joint_names_[i]);
            if (it != js.name.end()) {
                int idx = std::distance(js.name.begin(), it);
                q[i] = js.position[idx];
                
                // Tight filter to crush the 4ms Gazebo physics vibration
                filtered_dq_[i] = (0.92 * filtered_dq_[i]) + (0.08 * js.velocity[idx]);
            }
        }

        Eigen::Vector3d v_body_ego = Eigen::Vector3d::Zero();
        double weight_sum = 0.0;
        
        std::vector<Eigen::Vector3d> v_foot_kin(4);
        std::vector<Eigen::Vector3d> p_foot_kin(4);

        for (int i = 0; i < 4; ++i) {
            v_foot_kin[i] = RobotModel::calcFootVelocity(
                q[i*3], q[i*3+1], q[i*3+2], 
                filtered_dq_[i*3], filtered_dq_[i*3+1], filtered_dq_[i*3+2], 
                static_cast<Leg>(i));
            
            p_foot_kin[i] = RobotModel::calcForwardKinematics(q[i*3], q[i*3+1], q[i*3+2], static_cast<Leg>(i));
            
            double w = probs_[i]; 
            v_body_ego += w * (-v_foot_kin[i]);
            weight_sum += w;
        }

        if (weight_sum > 0.1) v_body_ego /= weight_sum;
        else                  v_body_ego.setZero();

        double sigma_planted = 0.45; 
        double sigma_swing = 1.50;
        double prior_weight = 1.5;   
        double forgetting_factor = 0.90; 
        double clamp_val = 5.0;

        double log_ratio = 3.0 * std::log(sigma_swing / sigma_planted);

        std_msgs::msg::Float64MultiArray prob_msg;
        prob_msg.data.resize(4, 0.0);

        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d v_slip = v_body_ego + v_foot_kin[i];
            latest_slip_[i] = v_slip.norm();

            // EFFECTIVE SLIP SCALING:
            // X and Y are multiplied by 0.5 to cut the Gazebo impact sliding penalty in half.
            // Z is multiplied by 0.05 to completely ignore Gazebo floor vibrations.
            double v_sq = (v_slip.x() * v_slip.x() * 0.5) +  
                          (v_slip.y() * v_slip.y() * 0.5) +  
                          (v_slip.z() * v_slip.z() * 0.05);   

            double delta_L = log_ratio - (v_sq / (2.0 * sigma_planted * sigma_planted)) 
                                       + (v_sq / (2.0 * sigma_swing * sigma_swing));

            delta_L += (prior_contacts[i] == 1) ? prior_weight : -prior_weight;

            // --- ZERO-NOISE KINEMATIC BOUNDARIES ---
            if (p_foot_kin[i].z() > -0.215) {
                // Leg is physically folded up. It MUST be in the air.
                delta_L -= 10.0; 
            } else if (p_foot_kin[i].z() < -0.235) {
                // Leg is deeply extended and bearing chassis weight. It MUST be planted.
                delta_L += 2.0;
            }

            log_odds_[i] = (forgetting_factor * log_odds_[i]) + delta_L;
            log_odds_[i] = std::clamp(log_odds_[i], -clamp_val, clamp_val);

            probs_[i] = 1.0 / (1.0 + std::exp(-log_odds_[i]));
            prob_msg.data[i] = probs_[i];
        }

        pub_probs_->publish(prob_msg);

        if (++log_counter_ >= 250) {
            log_counter_ = 0;
            std::cout << "\n=======================================================\n";
            std::cout << "SHADOW MODE TELEMETRY | MATHEMATICS \n";
            for (int i = 0; i < 4; ++i) {
                std::string gt_str = (true_contacts[i] == 1) ? "[GROUND]" : "[ AIR  ]";
                // std::string math_str = (probs_[i] > 0.5) ? "[GROUND]" : "[ AIR  ]";
                // std::string match = (gt_str == math_str) ? "OK " : "ERR";

                std::cout << "LEG " << i << " | TRUTH: " << gt_str 
                        //   << " | MATH PREDICTS: " << std::setw(8) << std::fixed << std::setprecision(3) << probs_[i] 
                          << " " << " | SLIP: " << std::setw(5) << latest_slip_[i] << " m/s | "  << "\n";
            }
            // std::cout << "=======================================================\n";
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContactEstimator>());
    rclcpp::shutdown();
    return 0;
}