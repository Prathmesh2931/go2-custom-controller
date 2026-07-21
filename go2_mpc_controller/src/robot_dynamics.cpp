#include "go2_mpc_controller/robot_dynamics.hpp"
#include <algorithm>

namespace go2_physics {

    Eigen::Matrix3d RobotModel::getInertiaTensor() {
        Eigen::Matrix3d I;
        // EXACT URDF VALUES RESTORED
        I << 0.02448,    0.00012166, 0.0014849,
             0.00012166, 0.098077,  -0.0000312,
             0.0014849, -0.0000312,  0.107;
        return I;
    }

    Eigen::Vector3d RobotModel::getCoMOffset() {
        // EXACT URDF VALUES RESTORED
        return Eigen::Vector3d(0.021112, 0.0, -0.005366);
    }

    Eigen::Vector3d RobotModel::getHipOffset(int leg_idx) {
        double hx = (leg_idx == 0 || leg_idx == 1) ? HIP_X : -HIP_X;
        double hy = (leg_idx == 0 || leg_idx == 2) ? HIP_Y : -HIP_Y;
        return Eigen::Vector3d(hx, hy, 0.0);
    }

    Eigen::Vector3d RobotModel::calcForwardKinematics(double q1, double q2, double q3, int leg_idx) {
        double l1 = (leg_idx == 0 || leg_idx == 2) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;

        double s1 = std::sin(q1);
        double c1 = std::cos(q1);
        double s2 = std::sin(q2);
        double c2 = std::cos(q2);
        double s23 = std::sin(q2 + q3);
        double c23 = std::cos(q2 + q3);

        Eigen::Vector3d pos;
        // X
        pos(0) = -l2 * s2 - l3 * s23;
        // Y
        pos(1) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
        // Z
        pos(2) = l1 * s1 - c1 * (l2 * c2 + l3 * c23);
        
        return pos;
    }

    Eigen::Vector3d RobotModel::calcInverseKinematics(const Eigen::Vector3d& pos, int leg_idx) {
        double x = pos(0);
        double y = pos(1);
        double z = pos(2);
        
        double hip_sign = (leg_idx == 0 || leg_idx == 2) ? 1.0 : -1.0;
        
        double r2d = std::sqrt(y*y + z*z);
        if (r2d < HIP_OFFSET) r2d = HIP_OFFSET + 0.001; 
        
        double inner = std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET);
        double L2_proj = std::sqrt(inner);

        double q_hip = std::atan2(y, -z) - hip_sign * std::atan2(HIP_OFFSET, L2_proj);

        double leg_distance = std::sqrt(x*x + L2_proj*L2_proj); 
        leg_distance = std::clamp(leg_distance, 0.01, THIGH_LEN + CALF_LEN - 0.001);
        
        double cos_calf = (leg_distance*leg_distance - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * CALF_LEN);
        double q_calf = -std::acos(std::clamp(cos_calf, -1.0, 1.0)); 

        double alpha = std::atan2(x, L2_proj);
        double cos_beta = (THIGH_LEN*THIGH_LEN + leg_distance*leg_distance - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * leg_distance);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        
        double q_thigh = beta - alpha;

        return Eigen::Vector3d(q_hip, q_thigh, q_calf);
    }

    Eigen::Matrix3d RobotModel::calcAnalyticalJacobian(double q1, double q2, double q3, int leg_idx) {
        double l1 = (leg_idx == 0 || leg_idx == 2) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;

        double s1 = std::sin(q1);
        double c1 = std::cos(q1);
        double s2 = std::sin(q2);
        double c2 = std::cos(q2);
        double s23 = std::sin(q2 + q3);
        double c23 = std::cos(q2 + q3);

        Eigen::Matrix3d J;
        
        // Row 0: X derivatives
        J(0, 0) = 0.0;
        J(0, 1) = -l2 * c2 - l3 * c23;
        J(0, 2) = -l3 * c23;
        
        // Row 1: Y derivatives
        J(1, 0) = -l1 * s1 + c1 * (l2 * c2 + l3 * c23);
        J(1, 1) = s1 * (-l2 * s2 - l3 * s23);
        J(1, 2) = s1 * (-l3 * s23);
        
        // Row 2: Z derivatives (Cleaned to exactly match original math)
        J(2, 0) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
        J(2, 1) = c1 * (l2 * s2 + l3 * s23);
        J(2, 2) = c1 * (l3 * s23);

        return J;
    }

} // namespace go2_physics