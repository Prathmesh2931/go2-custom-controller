#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>

class JacobianValidator : public rclcpp::Node
{
public:
    JacobianValidator() : Node("jacobian_validator")
    {
        RCLCPP_INFO(get_logger(), "Initiating Numerical vs Analytical Jacobian Audit...");
        run_validation_suite();
    }

private:
    static constexpr double HIP_OFFSET = 0.0955;
    static constexpr double THIGH_LEN  = 0.213;
    static constexpr double CALF_LEN   = 0.213;
    static constexpr double EPSILON    = 1e-5; // Central difference perturbation step size

    // Ground-truth geometric Forward Kinematics chain
    Eigen::Vector3d forward_kinematics(double q_hip, double q_thigh, double q_calf)
    {
        Eigen::Vector3d p_foot;
        double s1 = std::sin(q_hip);   double c1 = std::cos(q_hip);
        double s2 = std::sin(q_thigh); double c2 = std::cos(q_thigh);
        double s23 = std::sin(q_thigh + q_calf); double c23 = std::cos(q_thigh + q_calf);

        p_foot(0) = THIGH_LEN * s2 + CALF_LEN * s23;
        p_foot(1) = HIP_OFFSET * c1 - (THIGH_LEN * c2 + CALF_LEN * c23) * s1;
        p_foot(2) = -HIP_OFFSET * s1 - (THIGH_LEN * c2 + CALF_LEN * c23) * c1;
        return p_foot;
    }

    // Your analytical partial derivatives model
    Eigen::Matrix3d compute_analytical_jacobian(double q_hip, double q_thigh, double q_calf)
    {
        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
        double s1 = std::sin(q_hip);   double c1 = std::cos(q_hip);
        double s2 = std::sin(q_thigh); double c2 = std::cos(q_thigh);
        double s23 = std::sin(q_thigh + q_calf); double c23 = std::cos(q_thigh + q_calf);

        J(0,1) = THIGH_LEN * c2 + CALF_LEN * c23;
        J(0,2) = CALF_LEN * c23;
        J(1,0) = -HIP_OFFSET * s1 - (THIGH_LEN * c2 + CALF_LEN * c23) * c1;
        J(1,1) = (THIGH_LEN * s2 + CALF_LEN * s23) * s1;
        J(1,2) = CALF_LEN * s23 * s1;
        J(2,0) = -HIP_OFFSET * c1 + (THIGH_LEN * c2 + CALF_LEN * c23) * s1;
        J(2,1) = (THIGH_LEN * s2 + CALF_LEN * s23) * c1;
        J(2,2) = CALF_LEN * s23 * c1;
        return J;
    }

    // Model-free central difference estimator loop
    Eigen::Matrix3d compute_numerical_jacobian(double q_hip, double q_thigh, double q_calf)
    {
        Eigen::Matrix3d J_num = Eigen::Matrix3d::Zero();
        Eigen::Vector3d q(q_hip, q_thigh, q_calf);

        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d q_plus = q;  q_plus(i) += EPSILON;
            Eigen::Vector3d q_minus = q; q_minus(i) -= EPSILON;

            Eigen::Vector3d p_plus = forward_kinematics(q_plus(0), q_plus(1), q_plus(2));
            Eigen::Vector3d p_minus = forward_kinematics(q_minus(0), q_minus(1), q_minus(2));

            J_num.col(i) = (p_plus - p_minus) / (2.0 * EPSILON);
        }
        return J_num;
    }

    void run_validation_suite()
    {
        std::mt19937 gen(42); // Seeded random generator
        std::uniform_real_distribution<> hip_dist(-0.5, 0.5);
        std::uniform_real_distribution<> thigh_dist(0.2, 1.2);
        std::uniform_real_distribution<> calf_dist(-1.8, -0.8);

        double max_error = 0.0;
        int failures = 0;

        for (int test = 0; test < 1000; ++test) {
            double q0 = hip_dist(gen); double q1 = thigh_dist(gen); double q2 = calf_dist(gen);

            Eigen::Matrix3d Ja = compute_analytical_jacobian(q0, q1, q2);
            Eigen::Matrix3d Jn = compute_numerical_jacobian(q0, q1, q2);

            Eigen::Matrix3d error_matrix = (Ja - Jn).cwiseAbs();
            double local_max = error_matrix.maxCoeff();
            if (local_max > max_error) max_error = local_max;

            if (local_max > 1e-4) failures++;
        }

        std::cout << "\n=============================================\n";
        std::cout << "          JACOBIAN VALIDATION REPORT         \n";
        std::cout << "=============================================\n";
        std::cout << "Total Random Workspace Sweeps: 1000\n";
        std::cout << "Maximum Absolute Discrepancy Error: " << max_error << "\n";
        if (failures == 0) {
            std::cout << "AUDIT STATUS: VERIFIED [PASSED]\n";
        } else {
            std::cout << "AUDIT STATUS: FAIL [" << failures << " AXIS MISMATCHES DETECTED]\n";
        }
        std::cout << "=============================================\n\n";
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<JacobianValidator>()); rclcpp::shutdown(); return 0;
}