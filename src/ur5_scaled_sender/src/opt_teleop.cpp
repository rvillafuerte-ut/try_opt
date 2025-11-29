#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>

using namespace std::chrono_literals;

class OptTeleop : public rclcpp::Node {
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_id_;
    int nq_;
    
    Eigen::VectorXd q_current_, q_cmd_;
    Eigen::Matrix3d R_start_;
    Eigen::Vector3d p_start_;
    double t0_, traj_duration_;
    bool initialized_;
    double omega_, radius_, decay_rate_;
    double omega_rot_, amplitude_rot_;
    std::ofstream csv_file_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<std::string> joint_names_ = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

public:
    OptTeleop() : Node("opt_teleop"), 
                  data_(model_),
                  initialized_(false)
    {
        // Load URDF model
        std::string urdf_path = this->declare_parameter<std::string>("urdf_path", "/home/utec/try_opt/urdf/ur5.urdf");
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        ee_id_ = model_.getFrameId("tool0");
        nq_ = model_.nv;
        
        q_current_ = Eigen::VectorXd::Zero(nq_);
        q_cmd_ = Eigen::VectorXd::Zero(nq_);
        
        // Parámetros para trayectoria circular 3D con amplitud creciente
        omega_ = this->declare_parameter<double>("omega", 0.6);  // rad/s - velocidad angular
        radius_ = this->declare_parameter<double>("radius", 0.09);  // m - radio máximo
        decay_rate_ = this->declare_parameter<double>("decay_rate", 0.1);  // 1/s - tasa de crecimiento
        
        // Parámetros para variación de orientación
        omega_rot_ = this->declare_parameter<double>("omega_rot", 0.4);  // rad/s - velocidad angular de rotación
        amplitude_rot_ = this->declare_parameter<double>("amplitude_rot", 0.15);  // rad - amplitud de oscilación
        
        this->declare_parameter<double>("roll_des", 0.0);
        this->declare_parameter<double>("pitch_des", 0.0);
        this->declare_parameter<double>("yaw_des", 0.0);
        
        traj_duration_ = this->declare_parameter<double>("traj_duration", 5.0);
        double ctrl_hz = this->declare_parameter<double>("ctrl_hz", 100.0);
        
        std::string ctrl_name = this->declare_parameter<std::string>("controller_name", "scaled_joint_trajectory_controller");
        
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                // Mapear por nombre en lugar de por índice
                for(size_t i=0; i<joint_names_.size(); ++i) {
                    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    if(it != msg->name.end()) {
                        size_t idx = std::distance(msg->name.begin(), it);
                        q_current_(i) = msg->position[idx];
                    }
                }
                
                if(!initialized_) {
                    q_cmd_ = q_current_;
                    
                    // Compute initial pose using Pinocchio
                    pinocchio::forwardKinematics(model_, data_, q_current_);
                    pinocchio::framesForwardKinematics(model_, data_, q_current_);
                    p_start_ = data_.oMf[ee_id_].translation();
                    R_start_ = data_.oMf[ee_id_].rotation();
                    
                    t0_ = this->now().seconds();
                    initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Initialized at: [%.3f, %.3f, %.3f]", 
                               p_start_(0), p_start_(1), p_start_(2));
                    RCLCPP_INFO(this->get_logger(), "Trajectory: 3D circle | omega=%.2f rad/s | radius=%.3f m | growth_rate=%.2f", 
                               omega_, radius_, decay_rate_);
                    RCLCPP_INFO(this->get_logger(), "Orientation: variable | omega_rot=%.2f rad/s | amplitude=%.3f rad", 
                               omega_rot_, amplitude_rot_);
                }
            });
        
        pub_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/" + ctrl_name + "/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/ctrl_hz), 
            std::bind(&OptTeleop::control_loop, this));
        
        // Open CSV file for logging
        csv_file_.open("/home/utec/try_opt/teleop_data.csv");
        csv_file_ << "timestamp,pos_err,rot_err,";
        csv_file_ << "q_cmd_0,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,";
        csv_file_ << "q_current_0,q_current_1,q_current_2,q_current_3,q_current_4,q_current_5,";
        csv_file_ << "dq_0,dq_1,dq_2,dq_3,dq_4,dq_5,";
        csv_file_ << "p_des_x,p_des_y,p_des_z,p_actual_x,p_actual_y,p_actual_z\n";
        
        RCLCPP_INFO(this->get_logger(), "OptTeleop ready (Pinocchio): %.1f Hz, traj %.1fs, nq=%d", ctrl_hz, traj_duration_, nq_);
        RCLCPP_INFO(this->get_logger(), "Logging data to: /home/utec/try_opt/teleop_data.csv");
    }
    
    ~OptTeleop() {
        if(csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed");
        }
    }

private:
    void control_loop() {
        if(!initialized_) return;
        
        double t = this->now().seconds() - t0_;
        double s = std::min(1.0, t / traj_duration_);
        
        // Trayectoria circular 3D con amplitud creciente:
        // x = A(t) * sin(ωt)
        // y = A(t) * cos(ωt)  
        // z = A(t) * sin(ωt)
        // donde A(t) = radius * (1 - exp(-decay_rate * t))
        double amplitude = radius_ * (1.0 - std::exp(-decay_rate_ * t));
        double wt = omega_ * t;
        
        Eigen::Vector3d offset;
        offset << amplitude * std::sin(wt),
                  amplitude * std::cos(wt),
                  amplitude * std::sin(wt);
        
        Eigen::Vector3d p_des = p_start_ + offset;
        
        // Orientación deseada con variación temporal:
        // roll = amplitude_rot * sin(omega_rot * t)
        // pitch = amplitude_rot * cos(omega_rot * t)
        // yaw = amplitude_rot * sin(omega_rot * t + π/4)
        double wt_rot = omega_rot_ * t;
        double roll_var = amplitude_rot_ * std::sin(wt_rot);
        double pitch_var = amplitude_rot_ * std::cos(wt_rot);
        double yaw_var = amplitude_rot_ * std::sin(wt_rot + M_PI/4.0);
        
        Eigen::Matrix3d R_offset = (Eigen::AngleAxisd(yaw_var, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(pitch_var, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(roll_var, Eigen::Vector3d::UnitX())).matrix();
        Eigen::Matrix3d R_des = R_offset * R_start_;
        
        // Joint limits
        static const double pi = 3.14159265358979323846;
        Eigen::VectorXd q_min(nq_), q_max(nq_);
        q_min.fill(-pi);
        q_max.fill(pi);

        // Optimization parameters - más agresivo para seguimiento en tiempo real
        const double max_step = 0.05;
        const double damping = 0.01;
        const int max_iters = 10;
        const double pos_threshold = 0.05;
        
        // Re-seed from current position if command drifts too much
        Eigen::VectorXd dq_drift = q_cmd_ - q_current_;
        if(q_cmd_.isZero() || dq_drift.norm() > 0.3) {
            q_cmd_ = q_current_;
        }

        // Optimization loop
        for(int iter=0; iter<max_iters; ++iter) {
            
            // Compute FK and Jacobian using Pinocchio
            pinocchio::forwardKinematics(model_, data_, q_cmd_);
            pinocchio::framesForwardKinematics(model_, data_, q_cmd_);
            
            Eigen::Vector3d p_curr = data_.oMf[ee_id_].translation();
            Eigen::Matrix3d R_curr = data_.oMf[ee_id_].rotation();
            
            Eigen::MatrixXd J(6, nq_);
            pinocchio::computeFrameJacobian(model_, data_, q_cmd_, ee_id_,
                                           pinocchio::LOCAL_WORLD_ALIGNED, J);

            // Compute errors
            Eigen::Vector3d e_pos = p_des - p_curr;
            Eigen::Matrix3d R_err = R_des * R_curr.transpose();
            Eigen::Vector3d e_rot;
            e_rot << R_err(2,1)-R_err(1,2), R_err(0,2)-R_err(2,0), R_err(1,0)-R_err(0,1);
            e_rot *= 0.5;

            Eigen::VectorXd e(6);
            e << e_pos, e_rot;

            // Adaptive weighting
            Eigen::VectorXd W_diag(6);
            double pos_err = e_pos.norm();
            double w_rot = (pos_err > pos_threshold) ? 0.1 : 5.0;
            W_diag << 1000, 1000, 1000, w_rot, w_rot, w_rot;
            Eigen::MatrixXd W = W_diag.asDiagonal();

            // Levenberg-Marquardt step
            Eigen::MatrixXd H = J.transpose() * W * J + damping * Eigen::MatrixXd::Identity(nq_, nq_);
            Eigen::VectorXd g = -J.transpose() * W * e;
            Eigen::VectorXd dq = H.ldlt().solve(-g);

            // Clamp step size
            for(int i=0; i<nq_; ++i) {
                if(dq(i) >  max_step) dq(i) =  max_step;
                if(dq(i) < -max_step) dq(i) = -max_step;
            }

            // Backtracking line search
            double alpha = 1.0;
            double err_norm = e.norm();
            double err_weighted = std::sqrt((e.transpose() * W * e)(0));
            bool accepted = false;
            
            for(int ls=0; ls<5; ++ls) {
                Eigen::VectorXd q_try = q_cmd_ + alpha * dq;
                
                // Apply joint limits
                for(int i=0; i<nq_; ++i) {
                    if(q_try(i) < q_min(i)) q_try(i) = q_min(i);
                    if(q_try(i) > q_max(i)) q_try(i) = q_max(i);
                }

                // Evaluate at trial point
                pinocchio::forwardKinematics(model_, data_, q_try);
                pinocchio::framesForwardKinematics(model_, data_, q_try);
                
                Eigen::Vector3d p_try = data_.oMf[ee_id_].translation();
                Eigen::Matrix3d R_try = data_.oMf[ee_id_].rotation();
                
                Eigen::Vector3d e_pos_try = p_des - p_try;
                Eigen::Matrix3d R_err_try = R_des * R_try.transpose();
                Eigen::Vector3d e_rot_try;
                e_rot_try << R_err_try(2,1)-R_err_try(1,2), R_err_try(0,2)-R_err_try(2,0), R_err_try(1,0)-R_err_try(0,1);
                e_rot_try *= 0.5;
                
                Eigen::VectorXd e_try(6);
                e_try << e_pos_try, e_rot_try;
                double err_try_weighted = std::sqrt((e_try.transpose() * W * e_try)(0));

                if(err_try_weighted <= err_weighted) {
                    q_cmd_ = q_try;
                    err_norm = e_try.norm();
                    err_weighted = err_try_weighted;
                    accepted = true;
                    break;
                }
                alpha *= 0.5;
            }

            if(!accepted) break;
            if(err_norm < 1e-4) break;
        }
        
        // Send command at control frequency (100Hz)
        trajectory_msgs::msg::JointTrajectory cmd;
        cmd.header.stamp = this->now();
        cmd.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(q_cmd_.data(), q_cmd_.data()+nq_);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.02);  // 20ms lookahead
        cmd.points.push_back(pt);
        pub_cmd_->publish(cmd);
        
        // Logging (throttled)
        pinocchio::forwardKinematics(model_, data_, q_cmd_);
        pinocchio::framesForwardKinematics(model_, data_, q_cmd_);
        Eigen::Vector3d p_final = data_.oMf[ee_id_].translation();
        Eigen::Matrix3d R_final = data_.oMf[ee_id_].rotation();
        double pos_err = (p_des - p_final).norm();
        
        Eigen::Matrix3d R_err_final = R_des * R_final.transpose();
        Eigen::Vector3d e_rot_final;
        e_rot_final << R_err_final(2,1)-R_err_final(1,2), R_err_final(0,2)-R_err_final(2,0), R_err_final(1,0)-R_err_final(0,1);
        e_rot_final *= 0.5;
        double rot_err = e_rot_final.norm();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "t=%.1fs | A=%.3fm | Pos:[%.3f,%.3f,%.3f] | Rot:[%.2f,%.2f,%.2f] | Err: pos=%.4fm rot=%.4frad",
            t, amplitude, p_des(0), p_des(1), p_des(2), roll_var, pitch_var, yaw_var, pos_err, rot_err);
        
        // Log data to CSV
        if(csv_file_.is_open()) {
            Eigen::VectorXd dq = q_cmd_ - q_current_;
            csv_file_ << std::fixed << std::setprecision(6);
            csv_file_ << t << "," << pos_err << "," << rot_err << ",";
            for(int i=0; i<6; ++i) csv_file_ << q_cmd_(i) << ",";
            for(int i=0; i<6; ++i) csv_file_ << q_current_(i) << ",";
            for(int i=0; i<6; ++i) csv_file_ << dq(i) << ",";
            csv_file_ << p_des(0) << "," << p_des(1) << "," << p_des(2) << ",";
            csv_file_ << p_final(0) << "," << p_final(1) << "," << p_final(2) << "\n";
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptTeleop>());
    rclcpp::shutdown();
    return 0;
}
