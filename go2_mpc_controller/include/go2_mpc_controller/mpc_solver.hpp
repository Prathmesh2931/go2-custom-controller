#pragma once
#include <Eigen/Dense>
#include <vector>
#include <qpOASES.hpp>

class MpcSolver {
public:
    MpcSolver();
    ~MpcSolver();

    // 13-State: [Roll, Pitch, Yaw, pX, pY, pZ, wX, wY, wZ, vX, vY, vZ, Gravity]
    using StateVector = Eigen::Matrix<double, 13, 1>;
    using ForceVector = Eigen::Matrix<double, 12, 1>;

    ForceVector solve(const StateVector& current_state, const std::vector<Eigen::Vector3d>& foot_positions, double target_z);

private:
    // MPC Prediction Parameters
    const int N = 10;            // Horizon length (Look 10 steps into the future)
    const double dt = 0.03;      // 30ms per step (Predicting 0.3 seconds ahead)
    const double mu = 0.5;       // Friction coefficient
    const double f_max = 37.0;  // Maximum force per foot in Newtons

    // Tuning Weights
    Eigen::Matrix<double, 13, 13> Q; 
    Eigen::Matrix<double, 12, 12> R; 

    // Physics Matrices
    Eigen::Matrix<double, 13, 13> A_c;
    Eigen::Matrix<double, 13, 12> B_c;
    Eigen::Matrix<double, 13, 13> A_d;
    Eigen::Matrix<double, 13, 12> B_d;

    void buildContinuousMatrices(double yaw, const std::vector<Eigen::Vector3d>& foot_positions);
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
};