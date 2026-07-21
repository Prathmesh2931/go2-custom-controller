#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>
#include <string>

#include "go2_mpc_controller/robot_model.hpp"

using namespace go2_physics;

struct TestStats {
    std::string name;
    int passed = 0;
    int failed = 0;
    double max_error = 0.0;
    double sum_error = 0.0;

    void addResult(bool ok, double error) {
        if (ok) passed++;
        else failed++;
        if (error > max_error) max_error = error;
        sum_error += error;
    }

    void printSummary(int iterations) {
        double mean_error = sum_error / iterations;
        std::cout << std::left << std::setw(40) << name 
                  << " | Pass: " << std::setw(6) << passed 
                  << " | Fail: " << std::setw(6) << failed 
                  << " | Max Err: " << std::scientific << std::setprecision(2) << max_error 
                  << " | Mean Err: " << mean_error << "\n";
    }
};

class RobotModelValidator {
public:
    RobotModelValidator() {
        std::random_device rd;
        rng_.seed(rd());
    }

    void runAllTests(int iterations = 10000) {
        std::cout << "\n==================================================================================\n";
        std::cout << "               RIGOROUS ROBOT_MODEL VALIDATION SUITE (PROD-GRADE)\n";
        std::cout << "               Iterations per test: " << iterations << "\n";
        std::cout << "==================================================================================\n";

        runPinocchioSanityCheck();
        debugIsolateFKError();
        debugHipComponentwise();

        TestStats t1 = {"1. FK <-> IK Roundtrip (Workspace)"};
        TestStats t2 = {"2. Jacobian Central Finite Diff"};
        TestStats t3 = {"3. Mass Matrix Pos-Def & Symmetry"};
        TestStats t4 = {"4. Gravity Left/Right Mirror Sym."};
        TestStats t5 = {"5. Velocity Consistency (J * dq)"};
        TestStats t6 = {"6. IK Joint Limit Enforcement"};
        TestStats t7 = {"7. Pinocchio FK Cross-Check"};
        TestStats t8 = {"8. Dynamics ID (M*ddq + C*dq + G == RNEA)"};

        for (int i = 0; i < iterations; ++i) {
            Leg leg = static_cast<Leg>(i % 4);
            
            testIKFKConsistency(leg, t1);
            testJacobianFiniteDifference(leg, t2);
            testMassMatrixProperties(leg, t3);
            testGravitySymmetry(t4);
            testVelocityConsistency(leg, t5);
            testIKLimits(leg, t6);
            testPinocchioFK(leg, t7);
            testDynamicsConsistency(leg, t8);
        }

        t1.printSummary(iterations);
        t2.printSummary(iterations);
        t3.printSummary(iterations);
        t4.printSummary(iterations);
        t5.printSummary(iterations);
        t6.printSummary(iterations);
        t7.printSummary(iterations);
        t8.printSummary(iterations);

        std::cout << "\n==================================================================================\n";
        std::cout << "                             PERFORMANCE BENCHMARK\n";
        std::cout << "==================================================================================\n";
        runPerformanceBenchmark(iterations);

        std::cout << "==================================================================================\n\n";
    }

private:
    std::mt19937 rng_;

    void debugIsolateFKError() {
        std::cout << "\n--- FK Axis Isolation Debug ---\n";
        const char* leg_names[4] = {"LF", "RF", "LH", "RH"};

        for (int l = 0; l < 4; ++l) {
            Leg leg = static_cast<Leg>(l);

            // Vary only hip
            Eigen::Vector3d p_hand_hip = RobotModel::calcForwardKinematics(0.3, 0.0, -1.5, leg);
            Eigen::Vector3d p_pin_hip  = RobotModel::calcPinocchioFootPosition(0.3, 0.0, -1.5, leg);

            // Vary only thigh
            Eigen::Vector3d p_hand_thigh = RobotModel::calcForwardKinematics(0.0, 0.8, -1.5, leg);
            Eigen::Vector3d p_pin_thigh  = RobotModel::calcPinocchioFootPosition(0.0, 0.8, -1.5, leg);

            // Vary only calf
            Eigen::Vector3d p_hand_calf = RobotModel::calcForwardKinematics(0.0, 0.0, -2.0, leg);
            Eigen::Vector3d p_pin_calf  = RobotModel::calcPinocchioFootPosition(0.0, 0.0, -2.0, leg);

            std::cout << leg_names[l] << " hip-only diff:   " << std::fixed << std::setprecision(8) << (p_hand_hip - p_pin_hip).norm() << "\n";
            std::cout << leg_names[l] << " thigh-only diff: " << (p_hand_thigh - p_pin_thigh).norm() << "\n";
            std::cout << leg_names[l] << " calf-only diff:  " << (p_hand_calf - p_pin_calf).norm() << "\n";
        }
        std::cout << "-------------------------------\n";
    }

    void debugHipComponentwise() {
        std::cout << "\n--- Hip Rotation Componentwise Debug (LF leg) ---\n";
        for (double q1 : {-0.3, -0.1, 0.0, 0.1, 0.3}) {
            Eigen::Vector3d p_hand = RobotModel::calcForwardKinematics(q1, 0.0, -1.5, Leg::LF);
            Eigen::Vector3d p_pin  = RobotModel::calcPinocchioFootPosition(q1, 0.0, -1.5, Leg::LF);
            Eigen::Vector3d diff = p_hand - p_pin;
            
            std::cout << "q1=" << std::setw(6) << std::fixed << std::setprecision(3) << q1
                      << "  hand=(" << std::fixed << std::setprecision(4) << p_hand(0) << ", " << p_hand(1) << ", " << p_hand(2) << ")"
                      << "  pin=(" << p_pin(0) << ", " << p_pin(1) << ", " << p_pin(2) << ")"
                      << "  diff=(" << diff(0) << ", " << diff(1) << ", " << diff(2) << ")\n";
        }
        std::cout << "-----------------------------------------------\n";
    }

    Eigen::Vector3d getRandomJoints() {
        Eigen::Vector3d min_lim = RobotModel::getJointLimitsMin();
        Eigen::Vector3d max_lim = RobotModel::getJointLimitsMax();
        std::uniform_real_distribution<double> dist_hip(min_lim(0), max_lim(0));
        std::uniform_real_distribution<double> dist_thigh(min_lim(1), max_lim(1));
        std::uniform_real_distribution<double> dist_calf(min_lim(2), max_lim(2));
        return Eigen::Vector3d(dist_hip(rng_), dist_thigh(rng_), dist_calf(rng_));
    }

    Eigen::Vector3d getOperationalJoints() {
        std::uniform_real_distribution<double> dist_hip(-0.5, 0.5);         
        std::uniform_real_distribution<double> dist_thigh(0.0, 1.5);        
        std::uniform_real_distribution<double> dist_calf(-2.5, -1.0);       
        return Eigen::Vector3d(dist_hip(rng_), dist_thigh(rng_), dist_calf(rng_));
    }

    void testIKFKConsistency(Leg leg, TestStats& stats) {
        Eigen::Vector3d q_orig = getOperationalJoints();
        Eigen::Vector3d p_target = RobotModel::calcForwardKinematics(q_orig(0), q_orig(1), q_orig(2), leg);
        
        Eigen::Vector3d q_ik = RobotModel::calcInverseKinematics(p_target, leg);
        Eigen::Vector3d p_calc = RobotModel::calcForwardKinematics(q_ik(0), q_ik(1), q_ik(2), leg);
        
        double error = (p_target - p_calc).norm();
        stats.addResult(error < 1e-4, error);
    }

    void testJacobianFiniteDifference(Leg leg, TestStats& stats) {
        Eigen::Vector3d q = getRandomJoints();
        Eigen::Matrix3d J_ana = RobotModel::calcAnalyticalJacobian(q(0), q(1), q(2), leg);

        Eigen::Matrix3d J_num;
        const double eps = 1e-5;

        for (int j = 0; j < 3; ++j) {
            Eigen::Vector3d q_plus = q;
            Eigen::Vector3d q_minus = q;
            q_plus(j) += eps;
            q_minus(j) -= eps;
            
            Eigen::Vector3d p_plus = RobotModel::calcForwardKinematics(q_plus(0), q_plus(1), q_plus(2), leg);
            Eigen::Vector3d p_minus = RobotModel::calcForwardKinematics(q_minus(0), q_minus(1), q_minus(2), leg);
            
            J_num.col(j) = (p_plus - p_minus) / (2.0 * eps);
        }

        double error = (J_ana - J_num).norm();
        stats.addResult(error < 1e-3, error);
    }

    void testMassMatrixProperties(Leg leg, TestStats& stats) {
        Eigen::Vector3d q = getRandomJoints();
        Eigen::Matrix3d M = RobotModel::calcLegMassMatrix(q(0), q(1), q(2), leg);

        double sym_error = (M - M.transpose()).norm();
        bool is_sym = (sym_error < 1e-6);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
        double min_eig = solver.eigenvalues().minCoeff();
        double max_eig = solver.eigenvalues().maxCoeff();
        bool is_pd = (min_eig > 0.0);
        
        double cond = std::abs(max_eig / min_eig);
        bool is_stable = (cond < 1e6);

        stats.addResult(is_sym && is_pd && is_stable, sym_error);
    }

    void testGravitySymmetry(TestStats& stats) {
        Eigen::Vector3d q_LF = getRandomJoints();
        Eigen::Vector3d q_RF(-q_LF(0), q_LF(1), q_LF(2)); 

        Eigen::Vector3d g_LF = RobotModel::calcLegGravity(q_LF(0), q_LF(1), q_LF(2), Leg::LF);
        Eigen::Vector3d g_RF = RobotModel::calcLegGravity(q_RF(0), q_RF(1), q_RF(2), Leg::RF);

        Eigen::Vector3d g_RF_expected(-g_LF(0), g_LF(1), g_LF(2));
        double error = (g_RF - g_RF_expected).norm();
        
        stats.addResult(error < 1e-4, error);
    }

    void testVelocityConsistency(Leg leg, TestStats& stats) {
        Eigen::Vector3d q = getRandomJoints();
        
        std::uniform_real_distribution<double> dist_dq(-5.0, 5.0);
        Eigen::Vector3d dq(dist_dq(rng_), dist_dq(rng_), dist_dq(rng_));

        Eigen::Vector3d v_ana = RobotModel::calcFootVelocity(q(0), q(1), q(2), dq(0), dq(1), dq(2), leg);

        const double eps = 1e-5;
        Eigen::Vector3d p_plus = RobotModel::calcForwardKinematics(q(0) + eps*dq(0), q(1) + eps*dq(1), q(2) + eps*dq(2), leg);
        Eigen::Vector3d p_minus = RobotModel::calcForwardKinematics(q(0) - eps*dq(0), q(1) - eps*dq(1), q(2) - eps*dq(2), leg);
        Eigen::Vector3d v_num = (p_plus - p_minus) / (2.0 * eps);

        double error = (v_ana - v_num).norm();
        stats.addResult(error < 1e-3, error);
    }

    void testIKLimits(Leg leg, TestStats& stats) {
        Eigen::Vector3d q_orig = getOperationalJoints();
        Eigen::Vector3d p_target = RobotModel::calcForwardKinematics(q_orig(0), q_orig(1), q_orig(2), leg);
        
        Eigen::Vector3d q_ik = RobotModel::calcInverseKinematics(p_target, leg);
        Eigen::Vector3d min_lim = RobotModel::getJointLimitsMin();
        Eigen::Vector3d max_lim = RobotModel::getJointLimitsMax();
        
        bool valid = true;
        double max_viol = 0.0;
        for(int i=0; i<3; i++) {
            if (q_ik(i) < min_lim(i) - 1e-3 || q_ik(i) > max_lim(i) + 1e-3) {
                valid = false;
                max_viol = std::max(max_viol, std::max(min_lim(i) - q_ik(i), q_ik(i) - max_lim(i)));
            }
        }
        stats.addResult(valid, max_viol);
    }

    void runPinocchioSanityCheck() {
        std::cout << "\n--- Pinocchio FK Frame Sanity Check (q=0, LF leg) ---\n";
        Eigen::Vector3d p_hand = RobotModel::calcForwardKinematics(0, 0, 0, Leg::LF);
        Eigen::Vector3d p_pin  = RobotModel::calcPinocchioFootPosition(0, 0, 0, Leg::LF);
        Eigen::Vector3d diff   = p_hand - p_pin;

        std::cout << "Hand : " << p_hand.transpose() << "\n";
        std::cout << "Pin  : " << p_pin.transpose()  << "\n";
        std::cout << "Diff : " << std::fixed << std::setprecision(8) << diff.transpose()   << "\n";

        if (diff.norm() > 1e-6) {
            std::cout << "WARNING: Hand/Pinocchio FK mismatch at q=0 ("
                    << diff.norm() << ").\n";
        } else {
            std::cout << "OK: Hand and Pinocchio FK agree at q=0. Frame convention verified.\n";
        }
        std::cout << "-------------------------------------------------------\n";
    }

    void testPinocchioFK(Leg leg, TestStats& stats) {
        Eigen::Vector3d q = getOperationalJoints();
        Eigen::Vector3d p_hand = RobotModel::calcForwardKinematics(q(0), q(1), q(2), leg);
        Eigen::Vector3d p_pin  = RobotModel::calcPinocchioFootPosition(q(0), q(1), q(2), leg);

        double error = (p_hand - p_pin).norm();
        stats.addResult(error < 1e-6, error); 
    }

    // --- NEW: Test 8 - Dynamics Consistency Verification ---
    void testDynamicsConsistency(Leg leg, TestStats& stats) {
        Eigen::Vector3d q = getRandomJoints();
        
        std::uniform_real_distribution<double> dist_dq(-5.0, 5.0);
        Eigen::Vector3d dq(dist_dq(rng_), dist_dq(rng_), dist_dq(rng_));
        Eigen::Vector3d ddq(dist_dq(rng_), dist_dq(rng_), dist_dq(rng_));

        Eigen::Matrix3d M = RobotModel::calcLegMassMatrix(q(0), q(1), q(2), leg);
        Eigen::Vector3d C = RobotModel::calcLegCoriolis(q(0), q(1), q(2), dq(0), dq(1), dq(2), leg);
        Eigen::Vector3d G = RobotModel::calcLegGravity(q(0), q(1), q(2), leg);
        
        Eigen::Vector3d tau_rnea = RobotModel::calcLegRNEA(q(0), q(1), q(2), dq(0), dq(1), dq(2), ddq(0), ddq(1), ddq(2), leg);
        
        Eigen::Vector3d tau_comp = M * ddq + C + G;
        double error = (tau_rnea - tau_comp).norm();
        stats.addResult(error < 1e-6, error);
    }

    void runPerformanceBenchmark(int iterations) {
        using namespace std::chrono;
        
        Eigen::Vector3d q = getRandomJoints();
        Eigen::Vector3d dq(1.0, 1.0, 1.0);
        Eigen::Vector3d ddq(0.5, 0.5, 0.5);
        Leg leg = Leg::LF;
        Eigen::Vector3d p_target = RobotModel::calcForwardKinematics(q(0), q(1), q(2), leg);

        // Volatile checksum absolutely forces the compiler to run the math in the loop
        // Otherwise `-O3` flags will eliminate the loop because the result wasn't being used.
        volatile double checksum = 0.0;

        auto benchmark = [&](const std::string& name, auto func) {
            auto start = high_resolution_clock::now();
            for(int i=0; i<iterations; i++) {
                checksum += func();
            }
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            std::cout << std::left << std::setw(35) << name 
                      << ": " << std::fixed << std::setprecision(2) << (double)duration.count() / iterations << " us per call\n";
        };

        // Return the norm so it's added to the checksum, ensuring no dead-code elimination
        benchmark("Analytical Forward Kinematics", [&](){ return RobotModel::calcForwardKinematics(q(0), q(1), q(2), leg).norm(); });
        benchmark("Analytical Inverse Kinematics", [&](){ return RobotModel::calcInverseKinematics(p_target, leg).norm(); });
        benchmark("Analytical Jacobian", [&](){ return RobotModel::calcAnalyticalJacobian(q(0), q(1), q(2), leg).norm(); });
        benchmark("Pinocchio Mass Matrix (CRBA)", [&](){ return RobotModel::calcLegMassMatrix(q(0), q(1), q(2), leg).norm(); });
        benchmark("Pinocchio Gravity (RNEA)", [&](){ return RobotModel::calcLegGravity(q(0), q(1), q(2), leg).norm(); });
        benchmark("Pinocchio Coriolis (RNEA)", [&](){ return RobotModel::calcLegCoriolis(q(0), q(1), q(2), dq(0), dq(1), dq(2), leg).norm(); });
        benchmark("Pinocchio Full RNEA", [&](){ return RobotModel::calcLegRNEA(q(0), q(1), q(2), dq(0), dq(1), dq(2), ddq(0), ddq(1), ddq(2), leg).norm(); });
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("go2_description") + "/urdf/go2_description.urdf";
        RobotModel::initialize(urdf_path);
        
        RobotModelValidator validator;
        validator.runAllTests(10000); 
        
    } catch (const std::exception& e) {
        std::cerr << "\n[FATAL ERROR] " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}