#include <iostream>
#include <iomanip>
#include "go2_mpc_controller/robot_dynamics.hpp"
#include "go2_mpc_controller/mpc_solver.hpp" // <-- THE MISSING BLUEPRINTS!

int main() {
    std::cout << "--- GO2 PHYSICS DEBUGGER ---\n\n";
    
    std::cout << "1. Total URDF Mass: " << go2_physics::MASS << " kg\n";
    std::cout << "2. CoM Offset:\n" << go2_physics::getCoMOffset().transpose() << "\n\n";
    std::cout << "3. Base Inertia Tensor:\n" << go2_physics::getInertiaTensor() << "\n\n";

    // Test the Jacobian for the Left Front leg (index 0) in a standard standing pose
    double q_hip = 0.0;
    double q_thigh = 0.67;
    double q_calf = -1.30;
    
    Eigen::Matrix3d J = go2_physics::calcLegJacobian(q_hip, q_thigh, q_calf, 0);
    
    std::cout << "4. LF Leg Jacobian (q = [0, 0.67, -1.30]):\n" << J << "\n\n";

    // Test the MPC Solver
    MpcSolver solver;
    MpcSolver::StateVector dummy_state = MpcSolver::StateVector::Zero();
    dummy_state(5) = 0.30; // Current height is slightly below target (0.32m)
    dummy_state(12) = -9.81; // Gravity

    std::vector<Eigen::Vector3d> dummy_feet(4);
    dummy_feet[0] << 0.18, 0.13, -0.30;  // LF
    dummy_feet[1] << 0.18, -0.13, -0.30; // RF
    dummy_feet[2] << -0.18, 0.13, -0.30; // LH
    dummy_feet[3] << -0.18, -0.13, -0.30;// RH

    MpcSolver::ForceVector forces = solver.solve(dummy_state, dummy_feet, 0.34);
    std::cout << "5. MPC Generated Forces (LF, RF, LH, RH):\n" << forces.transpose() << "\n";

    return 0;
}