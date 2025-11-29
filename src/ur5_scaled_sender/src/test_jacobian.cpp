#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <iostream>

Eigen::Matrix4d calib_to_mat(double x, double y, double z, double r, double p, double yaw) {
    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).matrix();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);
    return T;
}

class URKinematics {
    std::vector<Eigen::Matrix4d> T_static_;
public:
    URKinematics(const std::string& yaml_path) {
        YAML::Node cfg = YAML::LoadFile(yaml_path);
        std::vector<std::string> links = {"shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"};
        T_static_.resize(6);
        for(int i=0; i<6; ++i) {
            YAML::Node k = cfg["kinematics"][links[i]];
            T_static_[i] = calib_to_mat(k["x"].as<double>(), k["y"].as<double>(), k["z"].as<double>(),
                                         k["roll"].as<double>(), k["pitch"].as<double>(), k["yaw"].as<double>());
        }
    }

    void compute(const Eigen::VectorXd& q, Eigen::Matrix4d& T_ee, Eigen::MatrixXd& J) {
        std::vector<Eigen::Matrix4d> T_accum(6);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        for(int i=0; i<6; ++i) {
            double c = cos(q(i)), s = sin(q(i));
            Eigen::Matrix4d T_rot = Eigen::Matrix4d::Identity();
            T_rot(0,0) = c; T_rot(0,1) = -s;
            T_rot(1,0) = s; T_rot(1,1) = c;
            T = T * T_static_[i] * T_rot;
            T_accum[i] = T;
        }
        T_ee = T;
        
        J.resize(6, 6);
        Eigen::Vector3d p_ee = T_ee.block<3,1>(0,3);
        
        for(int i=0; i<6; ++i) {
            Eigen::Matrix4d T_before_joint;
            if(i == 0) {
                T_before_joint = Eigen::Matrix4d::Identity();
            } else {
                T_before_joint = T_accum[i-1];
            }
            
            Eigen::Vector3d z_i = (T_before_joint * T_static_[i]).block<3,1>(0,2);
            Eigen::Vector3d p_i = (T_before_joint * T_static_[i]).block<3,1>(0,3);
            
            J.block<3,1>(0,i) = z_i.cross(p_ee - p_i);
            J.block<3,1>(3,i) = z_i;
        }
    }
    
    Eigen::MatrixXd finite_diff_jacobian(const Eigen::VectorXd& q, double eps = 1e-6) {
        Eigen::MatrixXd J_fd(6, 6);
        Eigen::Matrix4d T0, T_plus;
        Eigen::MatrixXd J_unused;
        
        compute(q, T0, J_unused);
        Eigen::Vector3d p0 = T0.block<3,1>(0,3);
        Eigen::Matrix3d R0 = T0.block<3,3>(0,0);
        
        for(int i=0; i<6; ++i) {
            Eigen::VectorXd q_plus = q;
            q_plus(i) += eps;
            compute(q_plus, T_plus, J_unused);
            
            Eigen::Vector3d p_plus = T_plus.block<3,1>(0,3);
            Eigen::Matrix3d R_plus = T_plus.block<3,3>(0,0);
            
            // Linear velocity
            J_fd.block<3,1>(0,i) = (p_plus - p0) / eps;
            
            // Angular velocity (from rotation difference)
            Eigen::Matrix3d R_diff = R_plus * R0.transpose();
            Eigen::Vector3d omega;
            omega << R_diff(2,1) - R_diff(1,2), R_diff(0,2) - R_diff(2,0), R_diff(1,0) - R_diff(0,1);
            J_fd.block<3,1>(3,i) = omega / (2.0 * eps);
        }
        
        return J_fd;
    }
};

int main() {
    URKinematics kin("/home/utec/try_opt/config/my_robot_calibration_ur5e.yaml");
    
    // Test at a random configuration
    Eigen::VectorXd q(6);
    q << -1.6, 1.56, 0.01, 0.03, 0.05, -0.05;
    
    Eigen::Matrix4d T_ee;
    Eigen::MatrixXd J_analytical;
    kin.compute(q, T_ee, J_analytical);
    
    Eigen::MatrixXd J_numerical = kin.finite_diff_jacobian(q);
    
    std::cout << "\n=== Jacobian Test ===\n\n";
    std::cout << "Position: " << T_ee.block<3,1>(0,3).transpose() << "\n\n";
    
    std::cout << "Analytical Jacobian:\n" << J_analytical << "\n\n";
    std::cout << "Numerical Jacobian:\n" << J_numerical << "\n\n";
    
    Eigen::MatrixXd diff = J_analytical - J_numerical;
    std::cout << "Difference (analytical - numerical):\n" << diff << "\n\n";
    std::cout << "Max absolute error: " << diff.cwiseAbs().maxCoeff() << "\n";
    std::cout << "RMS error: " << std::sqrt(diff.squaredNorm() / 36.0) << "\n";
    
    return 0;
}
