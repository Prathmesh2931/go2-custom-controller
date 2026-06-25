#include "go2_mpc_controller/mpc_solver.hpp"
#include "go2_mpc_controller/robot_dynamics.hpp"
#include <iostream>

MpcSolver::MpcSolver() {
    A_c.setZero();
    B_c.setZero();
    
    // Continuous A Matrix static parts
    A_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity(); // p_dot = v
    A_c(11, 12) = 1.0; // velocity derivative in Z is affected by gravity

    // MPC Tuning Weights
    Q.setZero();
    Q.diagonal() << 50.0, 50.0, 20.0,   // RPY
                    2.0,   2.0,   50.0,  // XYZ Position
                    1.0,   1.0,   1.0,    // Angular Velocity
                    1.0,   1.0,   1.0,    // Linear Velocity
                    0.0;                  // Gravity

    R.setIdentity();
    for(int i=0; i<4; i++) {
        R(i*3+0, i*3+0) = 1e-2; // Fx penalty (Expensive: Stop pushing forward/backward)
        R(i*3+1, i*3+1) = 1e-2; // Fy penalty (Expensive: Stop splaying outward)
        R(i*3+2, i*3+2) = 1e-4; // Fz penalty (Cheap: Use vertical force to balance!)
    }
}

MpcSolver::~MpcSolver() {}

Eigen::Matrix3d MpcSolver::skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<  0,   -v(2),  v(1),
          v(2), 0,    -v(0),
         -v(1), v(0),  0;
    return m;
}

void MpcSolver::buildContinuousMatrices(double yaw, const std::vector<Eigen::Vector3d>& foot_positions) {
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    Eigen::Matrix3d Rz;
    Rz << c, -s, 0,
          s,  c, 0,
          0,  0, 1;
          
    A_c.block<3, 3>(0, 6) = Rz;

    double mass = go2_physics::MASS;
    Eigen::Matrix3d I_G_body = go2_physics::getInertiaTensor();
    Eigen::Vector3d p_com_body = go2_physics::getCoMOffset();

    Eigen::Matrix3d I_G_yaw = Rz * I_G_body * Rz.transpose();
    Eigen::Matrix3d I_G_inv = I_G_yaw.inverse();

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d r_body = foot_positions[i] - p_com_body;
        Eigen::Vector3d r_yaw = Rz * r_body;

        B_c.block<3, 3>(6, i * 3) = I_G_inv * skewSymmetric(r_yaw);
        B_c.block<3, 3>(9, i * 3) = Eigen::Matrix3d::Identity() / mass;
    }
}

MpcSolver::ForceVector MpcSolver::solve(const StateVector& current_state, const std::vector<Eigen::Vector3d>& foot_positions, double target_z) {
    buildContinuousMatrices(current_state(2), foot_positions);

    // CRITICAL BUG FIX: Enforce core physics constants so matrices don't multiply by 0.0
    double dt_safe = 0.03;
    double mu_safe = 0.6;
    double f_max_safe = 100.0;

    // Forward Euler Discretization
    A_d = Eigen::Matrix<double, 13, 13>::Identity() + A_c * dt_safe;
    B_d = B_c * dt_safe;

    Eigen::Matrix<double, Eigen::Dynamic, 13> A_qp(13 * N, 13);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_qp(13 * N, 12 * N);
    B_qp.setZero();

    Eigen::Matrix<double, 13, 13> A_p = A_d;
    for (int i = 0; i < N; ++i) {
        A_qp.block(i * 13, 0, 13, 13) = A_p;
        A_p = A_p * A_d;
    }

    for (int i = 0; i < N; ++i) {
        Eigen::Matrix<double, 13, 13> A_p_b = Eigen::Matrix<double, 13, 13>::Identity();
        for (int j = i; j < N; ++j) {
            B_qp.block(j * 13, i * 12, 13, 12) = A_p_b * B_d;
            A_p_b = A_p_b * A_d;
        }
    }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q_bar(13 * N, 13 * N);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_bar(12 * N, 12 * N);
    Q_bar.setZero();
    R_bar.setZero();
    for (int i = 0; i < N; ++i) {
        Q_bar.block(i * 13, i * 13, 13, 13) = Q;
        R_bar.block(i * 12, i * 12, 12, 12) = R;
    }

    Eigen::Matrix<double, Eigen::Dynamic, 1> X_ref(13 * N, 1);
    X_ref.setZero();
    for(int i = 0; i < N; ++i) {
        X_ref(i * 13 + 5) = target_z; 
        X_ref(i * 13 + 12) = -9.81; 
    }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    H = 2.0 * (B_qp.transpose() * Q_bar * B_qp + R_bar);
    H = 0.5 * (H + H.transpose().eval()); // Force Symmetry
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> g;
    g = 2.0 * B_qp.transpose() * Q_bar * (A_qp * current_state - X_ref);

    int num_constraints = 20 * N;
    int num_variables = 12 * N;
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> C_mat(num_constraints, num_variables);
    Eigen::Matrix<double, Eigen::Dynamic, 1> lbA(num_constraints);
    Eigen::Matrix<double, Eigen::Dynamic, 1> ubA(num_constraints);
    C_mat.setZero();

    int c_idx = 0;
    for (int step = 0; step < N; ++step) {
        for (int leg = 0; leg < 4; ++leg) {
            int v_idx = step * 12 + leg * 3;
            
            // Apply mu_safe and f_max_safe so the bounds are physically possible
            C_mat(c_idx, v_idx) = 1.0;  C_mat(c_idx, v_idx + 2) = -mu_safe; lbA(c_idx) = -1e5; ubA(c_idx) = 0.0; c_idx++;
            C_mat(c_idx, v_idx) = -1.0; C_mat(c_idx, v_idx + 2) = -mu_safe; lbA(c_idx) = -1e5; ubA(c_idx) = 0.0; c_idx++;
            
            C_mat(c_idx, v_idx + 1) = 1.0;  C_mat(c_idx, v_idx + 2) = -mu_safe; lbA(c_idx) = -1e5; ubA(c_idx) = 0.0; c_idx++;
            C_mat(c_idx, v_idx + 1) = -1.0; C_mat(c_idx, v_idx + 2) = -mu_safe; lbA(c_idx) = -1e5; ubA(c_idx) = 0.0; c_idx++;
            
            C_mat(c_idx, v_idx + 2) = 1.0; lbA(c_idx) = 0.0; ubA(c_idx) = f_max_safe; c_idx++;
        }
    }

    int nWSR = 10000; 

    qpOASES::QProblem qp_problem(num_variables, num_constraints);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE; 
    qp_problem.setOptions(options);

    qpOASES::returnValue status = qp_problem.init(
        H.data(), g.data(), C_mat.data(), 
        nullptr, nullptr, 
        lbA.data(), ubA.data(), nWSR
    );

    ForceVector optimal_forces;
    optimal_forces.setZero();

    if (status == qpOASES::SUCCESSFUL_RETURN) {
        Eigen::Matrix<double, Eigen::Dynamic, 1> U_opt(num_variables);
        qp_problem.getPrimalSolution(U_opt.data());
        optimal_forces = U_opt.head<12>(); 
    }

    return optimal_forces;
}