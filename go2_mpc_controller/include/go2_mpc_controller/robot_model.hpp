#ifndef GO2_ROBOT_MODEL_HPP
#define GO2_ROBOT_MODEL_HPP

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <cmath>
#include <string>

namespace go2_physics {

    // --- Strongly Typed Leg Enum ---
    enum class Leg {
        LF = 0, // Left Front
        RF = 1, // Right Front
        LH = 2, // Left Hind
        RH = 3  // Right Hind
    };

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
        // --- Initialization ---
        // MUST BE CALLED ONCE AT STARTUP with the path to the Go2 URDF file!
        // This builds the Pinocchio Rigid Body Dynamics engine in the background.
        static void initialize(const std::string& urdf_path);

        // Returns the static Inertia Tensor for the Go2 chassis
        static Eigen::Matrix3d getInertiaTensor();
        
        // Returns the Center of Mass offset relative to the base_link
        static Eigen::Vector3d getCoMOffset();

        // Get the absolute offset of the hip from the CoM
        static Eigen::Vector3d getHipOffset(Leg leg);

        // --- Kinematics ---
        
        // Calculates Forward Kinematics (Foot position relative to the hip joint)
        static Eigen::Vector3d calcForwardKinematics(double q_hip, double q_thigh, double q_calf, Leg leg);

        // Calculates Inverse Kinematics (Joint angles required to reach a local foot position)
        static Eigen::Vector3d calcInverseKinematics(const Eigen::Vector3d& pos, Leg leg);

        // Calculates the 3x3 Analytical Jacobian mapping Joint Velocities to Cartesian Foot Velocities
        static Eigen::Matrix3d calcAnalyticalJacobian(double q_hip, double q_thigh, double q_calf, Leg leg);

        // Calculates the Cartesian Foot Velocity (J * dq)
        static Eigen::Vector3d calcFootVelocity(double q_hip, double q_thigh, double q_calf, 
                                                double dq_hip, double dq_thigh, double dq_calf, Leg leg);

        // --- Robot Configurations & Limits ---

        static std::array<double, 12> getDefaultStandPose();
        static Eigen::Vector3d getJointLimitsMin();
        static Eigen::Vector3d getJointLimitsMax();

        // --- Rigid Body Dynamics (Powered by Pinocchio) ---
        
        // Joint-space Mass/Inertia Matrix for a single leg: M(q) [3x3]
        static Eigen::Matrix3d calcLegMassMatrix(double q_hip, double q_thigh, double q_calf, Leg leg);

        // Coriolis and Centrifugal forces for a single leg: C(q, dq) * dq [3x1]
        static Eigen::Vector3d calcLegCoriolis(double q_hip, double q_thigh, double q_calf, 
                                               double dq_hip, double dq_thigh, double dq_calf, Leg leg);

        // Joint-space Gravity vector for a single leg: g(q) [3x1]
        static Eigen::Vector3d calcLegGravity(double q_hip, double q_thigh, double q_calf, Leg leg);

        // --- Cross-Validation (Pinocchio ground truth) ---
        static Eigen::Vector3d calcPinocchioFootPosition(double q_hip, double q_thigh, double q_calf, Leg leg);

        static Eigen::Vector3d calcLegRNEA(double q_hip, double q_thigh, double q_calf,
                                            double dq_hip, double dq_thigh, double dq_calf,
                                            double ddq_hip, double ddq_thigh, double ddq_calf, Leg leg);

        static void debugPrintFrames();
    };

} // namespace go2_physics

#endif // GO2_ROBOT_MODEL_HPP