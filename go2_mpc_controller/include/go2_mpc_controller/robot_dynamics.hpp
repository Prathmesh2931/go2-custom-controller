#ifndef GO2_ROBOT_MODEL_HPP
#define GO2_ROBOT_MODEL_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace go2_physics {

    // --- Physical Dimensions ---
    constexpr double MASS = 15.019; // Verified from URDF
    constexpr double HIP_OFFSET = 0.0955;
    constexpr double THIGH_LEN = 0.213;
    constexpr double CALF_LEN = 0.213;

    // Nominal Hip Attachments (Local Frame)
    constexpr double HIP_X = 0.1934;
    constexpr double HIP_Y = 0.0465; 

    class RobotModel {
    public:
        // Returns the static Inertia Tensor for the Go2 chassis
        static Eigen::Matrix3d getInertiaTensor();
        
        // Returns the Center of Mass offset relative to the base_link
        static Eigen::Vector3d getCoMOffset();

        // --- Kinematics ---
        
        // Calculates Forward Kinematics (Foot position relative to the hip joint)
        // leg_idx: 0=LF, 1=RF, 2=LH, 3=RH
        static Eigen::Vector3d calcForwardKinematics(double q_hip, double q_thigh, double q_calf, int leg_idx);

        // Calculates Inverse Kinematics (Joint angles required to reach a local foot position)
        // pos: (x, y, z) target relative to the hip
        // Returns: [q_hip, q_thigh, q_calf]
        static Eigen::Vector3d calcInverseKinematics(const Eigen::Vector3d& pos, int leg_idx);

        // Calculates the 3x3 Analytical Jacobian mapping Joint Velocities to Cartesian Foot Velocities
        static Eigen::Matrix3d calcAnalyticalJacobian(double q_hip, double q_thigh, double q_calf, int leg_idx);

        // Get the absolute offset of the hip from the CoM
        static Eigen::Vector3d getHipOffset(int leg_idx);
    };

} // namespace go2_physics

#endif // GO2_ROBOT_MODEL_HPP