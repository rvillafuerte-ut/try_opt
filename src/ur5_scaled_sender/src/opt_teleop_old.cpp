
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
                  q_current_(Eigen::VectorXd::Zero(6)),
                  q_cmd_(Eigen::VectorXd::Zero(6)),
                  initialized_(false)
    {
        // Load URDF model
        std::string urdf_path = this->declare_parameter<std::string>("urdf_path", "/home/utec/try_opt/urdf/ur5.urdf");
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        ee_id_ = model_.getFrameId("tool0");
        nq_ = model_.nv;
        
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
                for(size_t i=0; i<6 && i<msg->position.size(); ++i)
                    q_current_(i) = msg->position[i];
                
                if(!initialized_) {
                    q_cmd_ = q_current_;
                    
                    // Compute initial pose using Pinocchio
                    pinocchio::forwardKinematics(model_, data_, q_current_);
                    pinocchio::framesForwardKinematics(model_, data_, q_current_);
                    p_start_ = data_.oMf[ee_id_].translation();
                    R_start_ = data_.oMf[ee_id_].rotation();
                    
                    // Apply desired orientation offset (if any) to starting orientation
                    double roll = this->get_parameter("roll_des").as_double();
                    double pitch = this->get_parameter("pitch_des").as_double();
                    double yaw = this->get_parameter("yaw_des").as_double();
                    Eigen::Matrix3d R_offset = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).matrix();
                    R_des_ = R_offset * R_start_;  // Apply offset to current orientation
                    
                    t0_ = this->now().seconds();
                    initialized_ = true;
                    Eigen::Vector3d p_target = p_start_ + p_end_;  // p_end_ is now offset
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
        
        RCLCPP_INFO(this->get_logger(), "OptTeleop ready: %.1f Hz, traj %.1fs", ctrl_hz, traj_duration_);
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

        // Trust-region parameters
        const double max_step = 0.1;     // rad max per-iter change per joint
        const double damping = 0.01;     // Levenberg-Marquardt damping
        const int    max_iters = 20;     // fewer iters per cycle
        const double pos_threshold = 0.05; // 5cm - gate orientation until position close
        
        // Initialize q_cmd_ only once at startup - let it accumulate thereafter
        // Robot tracking will naturally lag command due to controller dynamics
        // However, if optimizer gets stuck (line search failing repeatedly), re-seed from current state
        if(q_cmd_.isZero()) {
            q_cmd_ = q_current_;
            stuck_count_ = 0;
        }

        double prev_err_norm = std::numeric_limits<double>::infinity();

        for(int iter=0; iter<max_iters; ++iter) {
            kin_.compute(q_cmd_, T_ee, J);

            Eigen::Vector3d e_pos = p_des - T_ee.block<3,1>(0,3);
            Eigen::Matrix3d R_err = R_des_ * T_ee.block<3,3>(0,0).transpose();
            Eigen::Vector3d e_rot;
            e_rot << R_err(2,1)-R_err(1,2), R_err(0,2)-R_err(2,0), R_err(1,0)-R_err(0,1);
            e_rot *= 0.5;

            Eigen::VectorXd e(6);
            e << e_pos, e_rot;

            Eigen::VectorXd W_diag(6);
            // Adapt orientation weight: gate it when position error is large
            double pos_err = e_pos.norm();
            double w_rot = (pos_err > pos_threshold) ? 0.1 : 5.0;  // minimal rotation weight until close
            W_diag << 1000, 1000, 1000, w_rot, w_rot, w_rot;
            Eigen::MatrixXd W = W_diag.asDiagonal();

            // Levenberg-Marquardt style step
            Eigen::MatrixXd H = J.transpose() * W * J + damping * Eigen::MatrixXd::Identity(6,6);
            Eigen::VectorXd g = -J.transpose() * W * e;
            Eigen::VectorXd dq = H.ldlt().solve(-g);

            // Clamp step size per joint
            for(int i=0; i<6; ++i) {
                if(dq(i) >  max_step) dq(i) =  max_step;
                if(dq(i) < -max_step) dq(i) = -max_step;
            }

            // Backtracking line search if error increases
            double alpha = 1.0;
            double err_norm = e.norm();
            // Use weighted error for line search to match optimization objective
            double err_weighted = std::sqrt((e.transpose() * W * e)(0));
            bool accepted = false;
            for(int ls=0; ls<5; ++ls) {
                Eigen::VectorXd q_try = q_cmd_ + alpha * dq;
                // Apply joint limits
                for(int i=0; i<6; ++i) {
                    if(q_try(i) < q_min(i)) q_try(i) = q_min(i);
                    if(q_try(i) > q_max(i)) q_try(i) = q_max(i);
                }

                Eigen::Matrix4d T_try; Eigen::MatrixXd J_try;
                kin_.compute(q_try, T_try, J_try);
                Eigen::Vector3d e_pos_try = p_des - T_try.block<3,1>(0,3);
                Eigen::Matrix3d R_err_try = R_des_ * T_try.block<3,3>(0,0).transpose();
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
                alpha *= 0.5; // shrink step
            }

            if(iter == 0 || iter == max_iters-1 || err_norm < 1e-3) {
                Eigen::Vector3d e_rot_log = e.tail<3>();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "IK iter=%d err=%.4f pos=[%.4f] rot=[%.4f] w_err=%.4f acc=%d", 
                    iter, err_norm, pos_err, e_rot_log.norm(), err_weighted, accepted);
            }
            if(!accepted) {
                if(iter == 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Line search failed on iter 0! dq_norm=%.4f err_w=%.4f", dq.norm(), err_weighted);
                }
                break;
            }
            if(err_norm < 1e-4) break;
            if(err_norm > prev_err_norm * 1.001) {
                // Guard against creeping increase
                break;
            }
            prev_err_norm = err_norm;
        }
        
        // Compute final position error for logging
        kin_.compute(q_cmd_, T_ee, J);
        double final_pos_err = (p_des - T_ee.block<3,1>(0,3)).norm();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Traj progress: %.1f%% | Target: [%.3f, %.3f, %.3f] | Pos err: %.4fm",
            s*100.0, p_des(0), p_des(1), p_des(2), final_pos_err);
        
        trajectory_msgs::msg::JointTrajectory cmd;
        cmd.header.stamp = this->now();
        cmd.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(q_cmd_.data(), q_cmd_.data()+6);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.01);  // 10ms lookahead
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