#include <casadi/casadi.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <iostream>
#include <chrono>

using namespace casadi;

int main() {
    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/utec/opt_crtl/src/ur5.urdf", model);
    pinocchio::Data data(model);
    pinocchio::FrameIndex ee_id = model.getFrameId("tool0");
    int nq = model.nv;
    
    Eigen::Vector3d x_des(0.3, 0.15, 0.5);
    Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity();
    
    // ========== VERSION 1: Pinocchio + Eigen (QP directo) ==========
    auto t1_start = std::chrono::high_resolution_clock::now();
    
    auto compute_fk_jac = [&](const std::vector<double>& q_val) 
        -> std::pair<Eigen::Vector3d, Eigen::MatrixXd> {
        Eigen::VectorXd q_eig = Eigen::Map<const Eigen::VectorXd>(q_val.data(), nq);
        pinocchio::forwardKinematics(model, data, q_eig);
        pinocchio::framesForwardKinematics(model, data, q_eig);
        Eigen::Vector3d pos = data.oMf[ee_id].translation();
        Eigen::MatrixXd J_full(6, nq);
        pinocchio::computeFrameJacobian(model, data, q_eig, ee_id, 
                                       pinocchio::LOCAL_WORLD_ALIGNED, J_full);
        return {pos, J_full};
    };
    
    Eigen::VectorXd q_sol1 = Eigen::VectorXd::Zero(nq);
    
    for (int iter = 0; iter < 10; ++iter) {
        auto [x_curr, J] = compute_fk_jac(std::vector<double>(q_sol1.data(), q_sol1.data() + nq));
        Eigen::Vector3d e_pos = x_des - x_curr;
        
        Eigen::Map<Eigen::VectorXd> q_eig(q_sol1.data(), nq);
        pinocchio::forwardKinematics(model, data, q_eig);
        pinocchio::framesForwardKinematics(model, data, q_eig);
        Eigen::Matrix3d R_curr = data.oMf[ee_id].rotation();
        Eigen::Matrix3d R_err = R_des * R_curr.transpose();
        Eigen::Vector3d e_rot;
        e_rot << R_err(2,1) - R_err(1,2), R_err(0,2) - R_err(2,0), R_err(1,0) - R_err(0,1);
        e_rot *= 0.5;
        
        Eigen::VectorXd e(6);
        e << e_pos, e_rot;
        if (e.norm() < 1e-5) break;
        
        Eigen::VectorXd W_diag(6);
        W_diag << 1, 1, 1, 0.1, 0.1, 0.1;
        Eigen::MatrixXd W = W_diag.asDiagonal();
        
        Eigen::MatrixXd H = J.transpose() * W * J + 0.01 * Eigen::MatrixXd::Identity(nq, nq);
        Eigen::VectorXd g = -J.transpose() * W * e;
        Eigen::VectorXd dq = H.ldlt().solve(-g);
        
        q_sol1 += dq;
    }
    
    auto t1_end = std::chrono::high_resolution_clock::now();
    double t1_ms = std::chrono::duration<double, std::milli>(t1_end - t1_start).count();
    
    auto [x_final1, _1] = compute_fk_jac(std::vector<double>(q_sol1.data(), q_sol1.data() + nq));
    Eigen::Map<Eigen::VectorXd> q_final1(q_sol1.data(), nq);
    pinocchio::forwardKinematics(model, data, q_final1);
    pinocchio::framesForwardKinematics(model, data, q_final1);
    Eigen::Matrix3d R_final1 = data.oMf[ee_id].rotation();
    double err1_pos = (x_final1 - x_des).norm();
    double err1_rot = (R_des - R_final1).norm();
    
    // ========== VERSION 2: CasADi + qpoases (QP solver) ==========
    auto t2_start = std::chrono::high_resolution_clock::now();
    
    // Crear solver QP una sola vez (estructura simbolica)
    SX dq_sym = SX::sym("dq", nq);
    SX H_sym = SX::sym("H", nq, nq);
    SX g_sym = SX::sym("g", nq);
    SX qp_obj = 0.5 * dot(dq_sym, mtimes(H_sym, dq_sym)) + dot(g_sym, dq_sym);
    
    SXDict qp_prob = {{"x", dq_sym}, {"f", qp_obj}, {"p", vertcat(vec(H_sym), g_sym)}};
    Dict qp_opts;
    qp_opts["printLevel"] = "none";
    Function qp_solver = qpsol("qp", "qpoases", qp_prob, qp_opts);
    
    Eigen::VectorXd q_sol2 = Eigen::VectorXd::Zero(nq);
    
    for (int iter = 0; iter < 10; ++iter) {
        auto [x_curr, J] = compute_fk_jac(std::vector<double>(q_sol2.data(), q_sol2.data() + nq));
        Eigen::Vector3d e_pos = x_des - x_curr;
        
        Eigen::Map<Eigen::VectorXd> q_eig(q_sol2.data(), nq);
        pinocchio::forwardKinematics(model, data, q_eig);
        pinocchio::framesForwardKinematics(model, data, q_eig);
        Eigen::Matrix3d R_curr = data.oMf[ee_id].rotation();
        Eigen::Matrix3d R_err = R_des * R_curr.transpose();
        Eigen::Vector3d e_rot;
        e_rot << R_err(2,1) - R_err(1,2), R_err(0,2) - R_err(2,0), R_err(1,0) - R_err(0,1);
        e_rot *= 0.5;
        
        Eigen::VectorXd e(6);
        e << e_pos, e_rot;
        if (e.norm() < 1e-5) break;
        
        // Convertir a CasADi DM
        DM J_dm = DM::zeros(6, nq);
        DM e_dm = DM::zeros(6, 1);
        for (int i = 0; i < 6; ++i) {
            e_dm(i) = e[i];
            for (int j = 0; j < nq; ++j)
                J_dm(i, j) = J(i, j);
        }
        
        DM W_dm = DM::diag(DM({1, 1, 1, 0.1, 0.1, 0.1}));
        
        // QP: min 0.5*dq'*H*dq + g'*dq
        DM H_dm = mtimes(mtimes(J_dm.T(), W_dm), J_dm) + 0.01 * DM::eye(nq);
        DM g_dm = -mtimes(mtimes(J_dm.T(), W_dm), e_dm);
        
        // Resolver
        DM p_dm = vertcat(vec(H_dm), g_dm);
        DMDict qp_arg = DMDict{{"p", p_dm}};
        DMDict qp_res = qp_solver(qp_arg);
        DM dq_dm = qp_res.at("x");
        
        for (int i = 0; i < nq; ++i)
            q_sol2[i] += double(dq_dm(i));
    }
    
    auto t2_end = std::chrono::high_resolution_clock::now();
    double t2_ms = std::chrono::duration<double, std::milli>(t2_end - t2_start).count();
    
    auto [x_final2, _2] = compute_fk_jac(std::vector<double>(q_sol2.data(), q_sol2.data() + nq));
    Eigen::Map<Eigen::VectorXd> q_final2(q_sol2.data(), nq);
    pinocchio::forwardKinematics(model, data, q_final2);
    pinocchio::framesForwardKinematics(model, data, q_final2);
    Eigen::Matrix3d R_final2 = data.oMf[ee_id].rotation();
    double err2_pos = (x_final2 - x_des).norm();
    double err2_rot = (R_des - R_final2).norm();
    
    // ========== RESULTADOS ==========
    std::cout << "\n=== Comparacion: Eigen vs CasADi (qpoases) ===\n\n";
    
    std::cout << "[1] Pinocchio + Eigen (LDLT directo):\n";
    std::cout << "    Tiempo:    " << t1_ms << " ms\n";
    std::cout << "    Error pos: " << err1_pos << " m\n";
    std::cout << "    Error rot: " << err1_rot << " rad\n";
    std::cout << "    q: " << q_sol1.transpose() << "\n\n";
    
    std::cout << "[2] Pinocchio + CasADi qpoases:\n";
    std::cout << "    Tiempo:    " << t2_ms << " ms\n";
    std::cout << "    Error pos: " << err2_pos << " m\n";
    std::cout << "    Error rot: " << err2_rot << " rad\n";
    std::cout << "    q: " << q_sol2.transpose() << "\n\n";
    
    std::cout << "vs OSQP (6D): 20 ms, error < 1e-5\n" << std::endl;
    
    return 0;
}
