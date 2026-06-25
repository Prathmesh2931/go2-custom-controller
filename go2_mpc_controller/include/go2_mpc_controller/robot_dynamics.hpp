#pragma once
#include <Eigen/Dense>

namespace go2_physics {

    // --- Go2 URDF Constants ---
    constexpr double MASS = 15.019; // Total extracted mass
    
    // Leg link lengths (meters)
    constexpr double HIP_OFFSET = 0.0955;
    constexpr double THIGH_LEN = 0.213;
    constexpr double CALF_LEN = 0.213;

    // Center of Mass Offset from the base link origin
    Eigen::Vector3d getCoMOffset();

    // The 3x3 Inertia Tensor (I_G) extracted from your URDF
    Eigen::Matrix3d getInertiaTensor();

    // Analytical 3x3 Jacobian for mapping forces to torques
    // leg_index: 0=LF, 1=RF, 2=LH, 3=RH
    Eigen::Matrix3d calcLegJacobian(double q_hip, double q_thigh, double q_calf, int leg_index);

    // Forward Kinematics: Gets the foot position relative to the hip
    Eigen::Vector3d calcFootPosition(double q_hip, double q_thigh, double q_calf, int leg_index);
}