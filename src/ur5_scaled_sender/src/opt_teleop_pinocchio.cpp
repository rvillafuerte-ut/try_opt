#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <chrono>

using namespace std::chrono_literals;

class OptTeleop : public rclcpp::Node {
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_id_;
    int nq_;
    
    Eigen::VectorXd q_current_, q_cmd_;
    Eigen::Matrix3d R_des_, R_start_;
    Eigen::Vector3d p_start_, p_end_;
    double t0_, traj_duration_;
    bool initialized_;
    
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
        
        p_end_ << this->declare_parameter<double>("x_end", 0.0), 
                  this->declare_parameter<double>("y_end", 0.0),
                  this->declare_parameter<double>("z_end", 0.0);
        
        this->declare_parameter<double>("roll_des", 0.0);
        this->declare_parameter<double>("pitch_des", 0.0);
        this->declare_parameter<double>("yaw_des", 0.0);
        
        traj_duration_ = this->declare_parameter<double>("traj_duration", 5.0);
        double ctrl_hz = this->declare_parameter<double>("ctrl_hz", 100.0);
        
        std::string ctrl_name = this->declare_parameter<std::string>("controller_name", "scaled_joint_trajectory_controller");
        
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                for(size_t i=0; i<nq_ && i<msg->position.size(); ++i)
                    q_current_(i) = msg->position[i];
                
                if(!initialized_) {
                    q_cmd_ = q_current_;
                    
                    // Compute initial pose using Pinocchio
                    pinocchio::forwardKinematics(model_, data_, q_current_);
                    pinocchio::framesForwardKinematics(model_, data_, q_current_);
                    p_start_ = data_.oMf[ee_id_].translation();
                    R_start_ = data_.oMf[ee_id_].rotation();
                    
                    // Apply desired orientation offset
                    double roll = this->get_parameter("roll_des").as_double();
                    double pitch = this->get_parameter("pitch_des").as_double();
                    double yaw = this->get_parameter("yaw_des").as_double();
                    Eigen::Matrix3d R_offset = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).matrix();
                    R_des_ = R_offset * R_start_;
                    
                    t0_ = this->now().seconds();
                    initialized_ = true;
                    Eigen::Vector3d p_target = p_start_ + p_end_;
                    RCLCPP_INFO(this->get_logger(), "Start: [%.3f, %.3f, %.3f] -> Offset: [%.3f, %.3f, %.3f] -> Target: [%.3f, %.3f, %.3f]", 
                               p_start_(0), p_start_(1), p_start_(2), 
                               p_end_(0), p_end_(1), p_end_(2),
                               p_target(0), p_target(1), p_target(2));
                }
            });
        
        pub_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/" + ctrl_name + "/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/ctrl_hz), 
            std::bind(&OptTeleop::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "OptTeleop ready (Pinocchio): %.1f Hz, traj %.1fs, nq=%d", ctrl_hz, traj_duration_, nq_);
    }

private:
    void control_loop() {
        if(!initialized_) return;
        
        double t = this->now().seconds() - t0_;
        double s = std::min(1.0, t / traj_duration_);
        Eigen::Vector3d p_des = p_start_ + s * p_end_;
        
        // Joint limits
        static const double pi = 3.14159265358979323846;
        Eigen::VectorXd q_min(nq_), q_max(nq_);
        q_min.fill(-pi);
        q_max.fill(pi);

        // Optimization parameters
        const double max_step = 0.1;
        const double damping = 0.01;
        const int max_iters = 10;
        const double pos_threshold = 0.05;
        
        // Initialize q_cmd_ only once at startup
        if(q_cmd_.isZero()) {
            q_cmd_ = q_current_;
        }

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
            Eigen::Matrix3d R_err = R_des_ * R_curr.transpose();
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
                Eigen::Matrix3d R_err_try = R_des_ * R_try.transpose();
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

            if(iter == 0 || iter == max_iters-1 || err_norm < 1e-3) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "IK iter=%d err=%.4f pos=[%.4f] rot=[%.4f] w_err=%.4f acc=%d", 
                    iter, err_norm, pos_err, e_rot.norm(), err_weighted, accepted);
            }
            if(!accepted) {
                if(iter == 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Line search failed iter 0! dq=%.4f err_w=%.4f", dq.norm(), err_weighted);
                }
                break;
            }
            if(err_norm < 1e-4) break;
        }
        
        // Compute final position error for logging
        pinocchio::forwardKinematics(model_, data_, q_cmd_);
        pinocchio::framesForwardKinematics(model_, data_, q_cmd_);
        double final_pos_err = (p_des - data_.oMf[ee_id_].translation()).norm();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Traj progress: %.1f%% | Target: [%.3f, %.3f, %.3f] | Pos err: %.4fm",
            s*100.0, p_des(0), p_des(1), p_des(2), final_pos_err);
        
        trajectory_msgs::msg::JointTrajectory cmd;
        cmd.header.stamp = this->now();
        cmd.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(q_cmd_.data(), q_cmd_.data()+nq_);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.01);
        cmd.points.push_back(pt);
        pub_cmd_->publish(cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptTeleop>());
    rclcpp::shutdown();
    return 0;
}
