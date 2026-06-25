#include "go2_mpc_controller/robot_dynamics.hpp"
#include <cmath>

namespace go2_physics {

Eigen::Vector3d getCoMOffset() {
    return Eigen::Vector3d(0.021112, 0.0, -0.005366);
}

Eigen::Matrix3d getInertiaTensor() {
    Eigen::Matrix3d I;
    I << 0.02448,   0.00012166, 0.0014849,
         0.00012166, 0.098077,  -0.0000312,
         0.0014849, -0.0000312,  0.107;
    return I;
}

Eigen::Matrix3d calcLegJacobian(double q1, double q2, double q3, int leg_index) {
    // Left legs (0, 2) have positive hip offset, Right legs (1, 3) have negative
    double l1 = (leg_index == 0 || leg_index == 2) ? HIP_OFFSET : -HIP_OFFSET;
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

    // Row 2: Z derivatives
    J(2, 0) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
    J(2, 1) = c1 * (l2 * s2 + l3 * s23);
    J(2, 2) = c1 * (l3 * s23);

    return J;
}

Eigen::Vector3d calcFootPosition(double q1, double q2, double q3, int leg_index) {
    double l1 = (leg_index == 0 || leg_index == 2) ? HIP_OFFSET : -HIP_OFFSET;
    double l2 = THIGH_LEN;
    double l3 = CALF_LEN;

    double s1 = std::sin(q1);
    double c1 = std::cos(q1);
    double s2 = std::sin(q2);
    double c2 = std::cos(q2);
    double s23 = std::sin(q2 + q3);
    double c23 = std::cos(q2 + q3);

    Eigen::Vector3d pos;
    pos(0) = -l2 * s2 - l3 * s23;
    pos(1) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
    pos(2) = l1 * s1 - c1 * (l2 * c2 + l3 * c23);
    return pos;
}

} // namespace go2_physics