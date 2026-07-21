#ifndef MPC_SOLVER_HPP
#define MPC_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <qpOASES.hpp>

class MpcSolver {
public:
    static constexpr int N = 10; // Prediction horizon

    using StateVector = Eigen::Matrix<double, 13, 1>;
    using ForceVector = Eigen::Matrix<double, 12, 1>;

    MpcSolver();
    ~MpcSolver();

    ForceVector solve(const StateVector& current_state, 
                      const std::vector<Eigen::Vector3d>& foot_positions, 
                      double target_z, 
                      const std::vector<int>& contact_state,
                      double cmd_vx, double cmd_vy, double cmd_wz);

private:
    Eigen::Matrix<double, 13, 13> A_c;
    Eigen::Matrix<double, 13, 12> B_c;
    Eigen::Matrix<double, 13, 13> A_d;
    Eigen::Matrix<double, 13, 12> B_d;
    Eigen::Matrix<double, 13, 13> Q;
    Eigen::Matrix<double, 12, 12> R;

    // THE FIX: SQProblem (Sequential QP) natively supports matrix-varying hotstarts!
    qpOASES::SQProblem qp_problem_;
    bool qp_initialized_ = false;

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
    void buildContinuousMatrices(double yaw, const std::vector<Eigen::Vector3d>& foot_positions);
};

#endif