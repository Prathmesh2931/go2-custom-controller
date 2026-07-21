#include "go2_mpc_controller/robot_model.hpp"
#include <algorithm>
#include <stdexcept>
#include <array>
#include <iostream>   // ADD THIS

// --- PINOCCHIO INCLUDES ---
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace go2_physics {

    // --- HIDDEN PINOCCHIO ENGINE ---
    static pinocchio::Model pin_model;
    static pinocchio::Data pin_data;
    static bool is_initialized = false;

    // Fast O(1) cache for joint indices to avoid string lookups in the control loop
    struct LegJointIndices {
        int hip_q, hip_v;
        int thigh_q, thigh_v;
        int calf_q, calf_v;
    };
    struct LegFrameIndices {
        pinocchio::FrameIndex hip;
        pinocchio::FrameIndex foot;
    };
    static std::array<LegFrameIndices, 4> cached_leg_frames;
    static std::array<LegJointIndices, 4> cached_leg_indices;

    static pinocchio::FrameIndex base_frame_id_;

    static pinocchio::FrameIndex resolveFrameId(const std::string& n1, const std::string& n2) {
        if (pin_model.existFrame(n1, pinocchio::BODY)) return pin_model.getFrameId(n1, pinocchio::BODY);
        if (pin_model.existFrame(n2, pinocchio::BODY)) return pin_model.getFrameId(n2, pinocchio::BODY);

        std::string avail = "";
        for (const auto& f : pin_model.frames) avail += f.name + " ";
        throw std::runtime_error("BODY frame " + n1 + " or " + n2 + " not found! Available: " + avail);
    }

    void RobotModel::initialize(const std::string& urdf_path) {
        if (is_initialized) return;
        
        // Build a fixed-base model (ideal for decoupled leg swing dynamics)
        pinocchio::urdf::buildModel(urdf_path, pin_model);
        pin_data = pinocchio::Data(pin_model);

        if (pin_model.existFrame("base", pinocchio::BODY)) {
            base_frame_id_ = pin_model.getFrameId("base", pinocchio::BODY);
        } else if (pin_model.existFrame("base_link", pinocchio::BODY)) {
            base_frame_id_ = pin_model.getFrameId("base_link", pinocchio::BODY);
        } else if (pin_model.existFrame("base")) {
            base_frame_id_ = pin_model.getFrameId("base");
        } else if (pin_model.existFrame("base_link")) {
            base_frame_id_ = pin_model.getFrameId("base_link");
        } else {
            base_frame_id_ = 0; 
        }

        // Pre-cache the joint indices for all 4 legs ONCE during startup
        auto get_idx = [&](const std::string& n1, const std::string& n2, int& q, int& v) {
            if (pin_model.existJointName(n1)) {
                q = pin_model.joints[pin_model.getJointId(n1)].idx_q();
                v = pin_model.joints[pin_model.getJointId(n1)].idx_v();
                return;
            }
            if (pin_model.existJointName(n2)) {
                q = pin_model.joints[pin_model.getJointId(n2)].idx_q();
                v = pin_model.joints[pin_model.getJointId(n2)].idx_v();
                return;
            }
            std::string avail = "";
            for(size_t i=0; i<pin_model.names.size(); ++i) avail += pin_model.names[i] + " ";
            throw std::runtime_error("Joint " + n1 + " or " + n2 + " not found! Available: " + avail);
        };

        const std::string prefixes[4] = {"lf_", "rf_", "lh_", "rh_"};
        const std::string prefixes_alt[4] = {"FL_", "FR_", "RL_", "RR_"};

        for (int i = 0; i < 4; ++i) {
            get_idx(prefixes[i] + "hip_joint", prefixes_alt[i] + "hip_joint", 
                    cached_leg_indices[i].hip_q, cached_leg_indices[i].hip_v);
            get_idx(prefixes[i] + "upper_leg_joint", prefixes_alt[i] + "thigh_joint", 
                    cached_leg_indices[i].thigh_q, cached_leg_indices[i].thigh_v);
            get_idx(prefixes[i] + "lower_leg_joint", prefixes_alt[i] + "calf_joint", 
                    cached_leg_indices[i].calf_q, cached_leg_indices[i].calf_v);

            cached_leg_frames[i].hip  = resolveFrameId(prefixes[i] + "hip",  prefixes_alt[i] + "hip");
            cached_leg_frames[i].foot = resolveFrameId(prefixes[i] + "foot", prefixes_alt[i] + "foot");
        }

        is_initialized = true;
    }

    void RobotModel::debugPrintFrames() {
        std::cout << "--- Available Pinocchio frames ---\n";
        for (const auto& f : pin_model.frames) {
            std::cout << f.name << "\n";
        }
        std::cout << "-----------------------------------\n";
        std::cout << "LF hip_frame idx  = " << cached_leg_frames[0].hip  << "\n";
        std::cout << "LF foot_frame idx = " << cached_leg_frames[0].foot << "\n";
        if (cached_leg_frames[0].hip == cached_leg_frames[0].foot) {
            std::cout << "WARNING: hip and foot frame resolved to the SAME index.\n";
        }
    }

    // ---------------------------------------------------------
    // STATIC CONSTANTS & KINEMATICS
    // ---------------------------------------------------------

    Eigen::Matrix3d RobotModel::getInertiaTensor() {
        Eigen::Matrix3d I;
        // EXACT URDF VALUES (Go2 Base)
        I << 0.02448,    0.00012166, 0.0014849,
             0.00012166, 0.098077,  -0.0000312,
             0.0014849, -0.0000312,  0.107;
        return I;
    }

    Eigen::Vector3d RobotModel::getCoMOffset() {
        return Eigen::Vector3d(0.021112, 0.0, -0.005366);
    }

    Eigen::Vector3d RobotModel::getHipOffset(Leg leg) {
        double hx = (leg == Leg::LF || leg == Leg::RF) ? HIP_X : -HIP_X;
        double hy = (leg == Leg::LF || leg == Leg::LH) ? HIP_Y : -HIP_Y;
        return Eigen::Vector3d(hx, hy, 0.0);
    }

    Eigen::Vector3d RobotModel::calcForwardKinematics(double q1, double q2, double q3, Leg leg) {
        double l1 = (leg == Leg::LF || leg == Leg::LH) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;

        double s1 = std::sin(q1); double c1 = std::cos(q1);
        double s2 = std::sin(q2); double c2 = std::cos(q2);
        double s23 = std::sin(q2 + q3); double c23 = std::cos(q2 + q3);

        Eigen::Vector3d pos;
        pos(0) = -l2 * s2 - l3 * s23;
        pos(1) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
        pos(2) = l1 * s1 - c1 * (l2 * c2 + l3 * c23);
        return pos;
    }

    Eigen::Vector3d RobotModel::calcInverseKinematics(const Eigen::Vector3d& pos, Leg leg) {
        double x = pos(0); double y = pos(1); double z = pos(2);
        double hip_sign = (leg == Leg::LF || leg == Leg::LH) ? 1.0 : -1.0;
        
        double r2d = std::sqrt(y*y + z*z);
        if (r2d < HIP_OFFSET) r2d = HIP_OFFSET + 0.001; 
        
        double inner = std::max(0.0, r2d*r2d - HIP_OFFSET*HIP_OFFSET);
        double L2_proj = std::sqrt(inner);

        double q_hip = std::atan2(y, -z) - hip_sign * std::atan2(HIP_OFFSET, L2_proj);

        double leg_distance = std::sqrt(x*x + L2_proj*L2_proj); 
        leg_distance = std::clamp(leg_distance, 0.01, THIGH_LEN + CALF_LEN - 0.001);
        
        double cos_calf = (leg_distance*leg_distance - THIGH_LEN*THIGH_LEN - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * CALF_LEN);
        double q_calf = -std::acos(std::clamp(cos_calf, -1.0, 1.0)); 

        double alpha = std::atan2(x, L2_proj);
        double cos_beta = (THIGH_LEN*THIGH_LEN + leg_distance*leg_distance - CALF_LEN*CALF_LEN) / (2.0 * THIGH_LEN * leg_distance);
        double beta = std::acos(std::clamp(cos_beta, -1.0, 1.0));
        
        double q_thigh = beta - alpha;

        return Eigen::Vector3d(q_hip, q_thigh, q_calf);
    }

    Eigen::Matrix3d RobotModel::calcAnalyticalJacobian(double q1, double q2, double q3, Leg leg) {
        double l1 = (leg == Leg::LF || leg == Leg::LH) ? HIP_OFFSET : -HIP_OFFSET;
        double l2 = THIGH_LEN;
        double l3 = CALF_LEN;

        double s1 = std::sin(q1); double c1 = std::cos(q1);
        double s2 = std::sin(q2); double c2 = std::cos(q2);
        double s23 = std::sin(q2 + q3); double c23 = std::cos(q2 + q3);

        Eigen::Matrix3d J;
        J(0, 0) = 0.0;
        J(0, 1) = -l2 * c2 - l3 * c23;
        J(0, 2) = -l3 * c23;
        
        J(1, 0) = -l1 * s1 + c1 * (l2 * c2 + l3 * c23);
        J(1, 1) = s1 * (-l2 * s2 - l3 * s23);
        J(1, 2) = s1 * (-l3 * s23);
        
        J(2, 0) = l1 * c1 + s1 * (l2 * c2 + l3 * c23);
        J(2, 1) = c1 * (l2 * s2 + l3 * s23);
        J(2, 2) = c1 * (l3 * s23);
        return J;
    }

    Eigen::Vector3d RobotModel::calcFootVelocity(double q_hip, double q_thigh, double q_calf, 
                                                 double dq_hip, double dq_thigh, double dq_calf, Leg leg) {
        Eigen::Matrix3d J = calcAnalyticalJacobian(q_hip, q_thigh, q_calf, leg);
        return J * Eigen::Vector3d(dq_hip, dq_thigh, dq_calf);
    }

    std::array<double, 12> RobotModel::getDefaultStandPose() {
        return { 0.0, 0.75, -1.5,   0.0, 0.75, -1.5,   0.0, 0.75, -1.5,   0.0, 0.75, -1.5 };
    }

    Eigen::Vector3d RobotModel::getJointLimitsMin() {
        return Eigen::Vector3d(-1.0472, -1.5708, -2.7227);
    }

    Eigen::Vector3d RobotModel::getJointLimitsMax() {
        return Eigen::Vector3d(1.0472, 3.4907, -0.83776);
    }

    // ---------------------------------------------------------
    // PINOCCHIO RIGID BODY DYNAMICS
    // ---------------------------------------------------------

    Eigen::Matrix3d RobotModel::calcLegMassMatrix(double q_hip, double q_thigh, double q_calf, Leg leg) {
        if (!is_initialized) throw std::logic_error("Call RobotModel::initialize(urdf) first!");
        
        // Fast O(1) lookup
        const LegJointIndices& idx = cached_leg_indices[static_cast<int>(leg)];
        Eigen::VectorXd q = Eigen::VectorXd::Zero(pin_model.nq);
        
        q(idx.hip_q) = q_hip; 
        q(idx.thigh_q) = q_thigh; 
        q(idx.calf_q) = q_calf;

        // Composite Rigid Body Algorithm (calculates full M matrix)
        pinocchio::crba(pin_model, pin_data, q);
        
        // Pinocchio only fills the upper triangle for efficiency, copy to lower
        pin_data.M.triangularView<Eigen::StrictlyLower>() = pin_data.M.transpose().triangularView<Eigen::StrictlyLower>();

        // Safely extract the exact 3x3 block using stack array (no dynamic memory)
        Eigen::Matrix3d M_leg;
        std::array<int, 3> v_idx = {idx.hip_v, idx.thigh_v, idx.calf_v};
        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                M_leg(r,c) = pin_data.M(v_idx[r], v_idx[c]);
            }
        }
        return M_leg;
    }

    Eigen::Vector3d RobotModel::calcLegCoriolis(double q_hip, double q_thigh, double q_calf, 
                                                double dq_hip, double dq_thigh, double dq_calf, Leg leg) {
        if (!is_initialized) throw std::logic_error("Call RobotModel::initialize(urdf) first!");

        const LegJointIndices& idx = cached_leg_indices[static_cast<int>(leg)];
        Eigen::VectorXd q = Eigen::VectorXd::Zero(pin_model.nq);
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(pin_model.nv);
        
        q(idx.hip_q) = q_hip; q(idx.thigh_q) = q_thigh; q(idx.calf_q) = q_calf;
        dq(idx.hip_v) = dq_hip; dq(idx.thigh_v) = dq_thigh; dq(idx.calf_v) = dq_calf;

        // Compute Coriolis matrix C(q, dq) using RNEA
        pinocchio::computeCoriolisMatrix(pin_model, pin_data, q, dq);

        Eigen::Matrix3d C_leg;
        std::array<int, 3> v_idx = {idx.hip_v, idx.thigh_v, idx.calf_v};
        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                C_leg(r,c) = pin_data.C(v_idx[r], v_idx[c]);
            }
        }
        // Returns the actual Coriolis Force Vector: C(q, dq) * dq
        return C_leg * Eigen::Vector3d(dq_hip, dq_thigh, dq_calf);
    }

    Eigen::Vector3d RobotModel::calcLegGravity(double q_hip, double q_thigh, double q_calf, Leg leg) {
        if (!is_initialized) throw std::logic_error("Call RobotModel::initialize(urdf) first!");

        const LegJointIndices& idx = cached_leg_indices[static_cast<int>(leg)];
        Eigen::VectorXd q = Eigen::VectorXd::Zero(pin_model.nq);
        
        q(idx.hip_q) = q_hip; 
        q(idx.thigh_q) = q_thigh; 
        q(idx.calf_q) = q_calf;

        // Compute generalized gravity vector g(q)
        pinocchio::computeGeneralizedGravity(pin_model, pin_data, q);

        Eigen::Vector3d g_leg;
        g_leg(0) = pin_data.g(idx.hip_v);
        g_leg(1) = pin_data.g(idx.thigh_v);
        g_leg(2) = pin_data.g(idx.calf_v);
        return g_leg;
    }

    Eigen::Vector3d RobotModel::calcPinocchioFootPosition(double q_hip, double q_thigh, double q_calf, Leg leg) {
        if (!is_initialized) throw std::logic_error("Call RobotModel::initialize(urdf) first!");
        const LegJointIndices& idx = cached_leg_indices[static_cast<int>(leg)];
        const LegFrameIndices& frames = cached_leg_frames[static_cast<int>(leg)];

        Eigen::VectorXd q = Eigen::VectorXd::Zero(pin_model.nq);
        q(idx.hip_q) = q_hip; q(idx.thigh_q) = q_thigh; q(idx.calf_q) = q_calf;

        pinocchio::forwardKinematics(pin_model, pin_data, q);
        pinocchio::updateFramePlacements(pin_model, pin_data);

        // 3. REPLACE THE OLD RETURN MATH WITH THIS FIX:
        // Extract foot position relative to the non-rotating base chassis!
        Eigen::Vector3d p_in_base = (pin_data.oMf[base_frame_id_].inverse() * pin_data.oMf[frames.foot]).translation();

        // Subtract the fixed (non-rotating) hip mount offset to match our handwriting kinematics origin
        return p_in_base - RobotModel::getHipOffset(leg);
    }

    Eigen::Vector3d RobotModel::calcLegRNEA(double q_hip, double q_thigh, double q_calf,
                                             double dq_hip, double dq_thigh, double dq_calf,
                                             double ddq_hip, double ddq_thigh, double ddq_calf, Leg leg) {
        if (!is_initialized) throw std::logic_error("Call RobotModel::initialize(urdf) first!");
        const LegJointIndices& idx = cached_leg_indices[static_cast<int>(leg)];

        Eigen::VectorXd q   = Eigen::VectorXd::Zero(pin_model.nq);
        Eigen::VectorXd dq  = Eigen::VectorXd::Zero(pin_model.nv);
        Eigen::VectorXd ddq = Eigen::VectorXd::Zero(pin_model.nv);

        q(idx.hip_q) = q_hip;     q(idx.thigh_q) = q_thigh;     q(idx.calf_q) = q_calf;
        dq(idx.hip_v) = dq_hip;   dq(idx.thigh_v) = dq_thigh;   dq(idx.calf_v) = dq_calf;
        ddq(idx.hip_v) = ddq_hip; ddq(idx.thigh_v) = ddq_thigh; ddq(idx.calf_v) = ddq_calf;

        pinocchio::rnea(pin_model, pin_data, q, dq, ddq);

        Eigen::Vector3d tau;
        tau(0) = pin_data.tau(idx.hip_v);
        tau(1) = pin_data.tau(idx.thigh_v);
        tau(2) = pin_data.tau(idx.calf_v);
        return tau;
    }

} // namespace go2_physics