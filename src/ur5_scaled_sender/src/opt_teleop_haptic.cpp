#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    double t0_;
    bool initialized_;
    std::ofstream csv_file_;
    
    // Haptic variables
    bool haptic_initialized_;
    Eigen::Vector3d phantom_pos_initial_, phantom_pos_current_;
    Eigen::Quaterniond phantom_quat_initial_, phantom_quat_current_;
    bool tracking_active_;
    
    // Scaling
    double haptic_scale_pos_;
    double haptic_scale_rot_;
    
    // Axis inversion flags
    double sign_x_, sign_y_, sign_z_;
    double sign_roll_, sign_pitch_, sign_yaw_;
    
    // Axis remapping (0=X/Roll, 1=Y/Pitch, 2=Z/Yaw)
    int map_x_, map_y_, map_z_;
    int map_roll_, map_pitch_, map_yaw_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_phantom_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Joint names in correct order for Pinocchio model
    std::vector<std::string> joint_names_ = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    
    Eigen::VectorXd q_min_, q_max_;

public:
    OptTeleop() : Node("opt_teleop_haptic"), 
                  data_(model_),
                  initialized_(false),
                  haptic_initialized_(false),
                  tracking_active_(false)
    {
        // Parameters
        std::string urdf_path = this->declare_parameter<std::string>("urdf_path", "/home/utec/try_opt/urdf/ur5.urdf");
        double ctrl_hz = this->declare_parameter<double>("ctrl_hz", 100.0);
        std::string ctrl_name = this->declare_parameter<std::string>("controller_name", "scaled_joint_trajectory_controller");
        
        haptic_scale_pos_ = this->declare_parameter<double>("haptic_scale_pos", 1.0);
        haptic_scale_rot_ = this->declare_parameter<double>("haptic_scale_rot", 1.0);
        
        
       // POSICIÓN: Alineación Phantom RAW -> UR5 Base
        // Robot X (Adelante) <-> Phantom Z (Profundidad, Index 2). Invertido.
        map_x_ = this->declare_parameter<int>("map_x", 2);
        sign_x_ = this->declare_parameter<double>("sign_x", -1.0);

        // Robot Y (Lateral) <-> Phantom X (Lateral, Index 0). Invertido.
        map_y_ = this->declare_parameter<int>("map_y", 0);
        sign_y_ = this->declare_parameter<double>("sign_y", -1.0);

        // Robot Z (Arriba) <-> Phantom Y (Vertical, Index 1). Normal.
        map_z_ = this->declare_parameter<int>("map_z", 1);
        sign_z_ = this->declare_parameter<double>("sign_z", 1.0);

        // ORIENTACIÓN: Indices descubiertos en tus LOGS
        // Robot Roll (X) <-> Phantom Index 2 (Roll/Muñeca)
        map_roll_ = this->declare_parameter<int>("map_roll", 2);
        sign_roll_ = this->declare_parameter<double>("sign_roll", 1.0);

        // Robot Pitch (Y) <-> Phantom Index 0 (Stylus Arriba/Abajo - Confirmado en Log)
        map_pitch_ = this->declare_parameter<int>("map_pitch", 0);
        sign_pitch_ = this->declare_parameter<double>("sign_pitch", 1.0);

        // Robot Yaw (Z) <-> Phantom Index 1 (Base - Confirmado en Log)
        map_yaw_ = this->declare_parameter<int>("map_yaw", 1);
        sign_yaw_ = this->declare_parameter<double>("sign_yaw", 1.0);
        
        // Build Pinocchio model
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        ee_id_ = model_.getFrameId("tool0");
        nq_ = model_.nv;
        
        q_current_ = Eigen::VectorXd::Zero(nq_);
        q_cmd_ = Eigen::VectorXd::Zero(nq_);
        
        // Configure UR5 joint limits
        q_min_ = Eigen::VectorXd(nq_);
        q_max_ = Eigen::VectorXd(nq_);
        double pi2 = 2.0 * M_PI;
        q_min_ << -pi2, -pi2, -M_PI, -pi2, -pi2, -pi2;
        q_max_ <<  pi2,  pi2,  M_PI,  pi2,  pi2,  pi2;
        
        // Joint states subscription with name-based mapping
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                // Map joints by name to handle any ordering
                for(size_t i=0; i<joint_names_.size(); ++i) {
                    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    if(it != msg->name.end()) {
                        size_t idx = std::distance(msg->name.begin(), it);
                        q_current_(i) = msg->position[idx];
                    }
                }
                
                if(!initialized_) {
                    if (q_current_.norm() < 0.001) {
                        return; // Wait for valid joint states
                    }
                    
                    q_cmd_ = q_current_;
                    pinocchio::forwardKinematics(model_, data_, q_current_);
                    pinocchio::framesForwardKinematics(model_, data_, q_current_);
                    p_start_ = data_.oMf[ee_id_].translation();
                    R_start_ = data_.oMf[ee_id_].rotation();
                    t0_ = this->now().seconds();
                    initialized_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "Robot initialized at: [%.3f, %.3f, %.3f]",
                               p_start_(0), p_start_(1), p_start_(2));
                }
            });
        
        // Phantom pose subscription
        sub_phantom_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/phantom/pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                phantom_pos_current_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
                phantom_quat_current_ = Eigen::Quaterniond(
                    msg->pose.orientation.w,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z);
                
                if(initialized_ && !haptic_initialized_) {
                    phantom_pos_initial_ = phantom_pos_current_;
                    phantom_quat_initial_ = phantom_quat_current_;
                    haptic_initialized_ = true;
                    tracking_active_ = true;
                    
                    const char* axis_names[] = {"X", "Y", "Z"};
                    const char* rot_names[] = {"Roll", "Pitch", "Yaw"};
                    
                    RCLCPP_INFO(this->get_logger(), "Haptic initialized at: [%.3f, %.3f, %.3f]",
                               phantom_pos_initial_(0), phantom_pos_initial_(1), phantom_pos_initial_(2));
                    RCLCPP_INFO(this->get_logger(), "Position mapping: Robot[X,Y,Z] <- Phantom[%s, %s, %s]",
                               axis_names[map_x_], axis_names[map_y_], axis_names[map_z_]);
                    RCLCPP_INFO(this->get_logger(), "Position signs: x=%.0f, y=%.0f, z=%.0f (scale=%.2f)",
                               sign_x_, sign_y_, sign_z_, haptic_scale_pos_);
                    RCLCPP_INFO(this->get_logger(), "Orientation mapping: Robot[R,P,Y] <- Phantom[%s, %s, %s]",
                               rot_names[map_roll_], rot_names[map_pitch_], rot_names[map_yaw_]);
                    RCLCPP_INFO(this->get_logger(), "Orientation signs: roll=%.0f, pitch=%.0f, yaw=%.0f (scale=%.2f)",
                               sign_roll_, sign_pitch_, sign_yaw_, haptic_scale_rot_);
                }
            });
        
        pub_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/" + ctrl_name + "/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/ctrl_hz),
            std::bind(&OptTeleop::control_loop, this));
        
        csv_file_.open("/home/utec/try_opt/teleop_haptic_data.csv");
        csv_file_ << "time,phantom_offset_x,phantom_offset_y,phantom_offset_z,"
                  << "robot_x,robot_y,robot_z,error_pos,error_rot\n";
        
        RCLCPP_INFO(this->get_logger(), "OptTeleop started. Control frequency: %.1f Hz", ctrl_hz);
    }
    
    ~OptTeleop() { 
        if(csv_file_.is_open()) csv_file_.close(); 
    }

private:
    void control_loop() {
        if(!initialized_) return;
        
        double t = this->now().seconds() - t0_;
        Eigen::Vector3d p_des;
        Eigen::Matrix3d R_des;
        
        if(tracking_active_ && haptic_initialized_) {
            // Position offset with remapping and signs
            Eigen::Vector3d phantom_offset_raw = phantom_pos_current_ - phantom_pos_initial_;
            Eigen::Vector3d phantom_offset_remapped;
            phantom_offset_remapped(0) = phantom_offset_raw(map_x_) * sign_x_ * haptic_scale_pos_;
            phantom_offset_remapped(1) = phantom_offset_raw(map_y_) * sign_y_ * haptic_scale_pos_;
            phantom_offset_remapped(2) = phantom_offset_raw(map_z_) * sign_z_ * haptic_scale_pos_;
            
            // Transform to robot frame
            Eigen::Vector3d phantom_offset_robot = R_start_ * phantom_offset_remapped;
            p_des = p_start_ + phantom_offset_robot;
            
            // Orientation offset with remapping and signs
            Eigen::Quaterniond quat_offset = phantom_quat_current_ * phantom_quat_initial_.inverse();
            Eigen::AngleAxisd aa(quat_offset);
            Eigen::Vector3d axis_angle_raw = aa.axis() * aa.angle();
            
            Eigen::Vector3d axis_angle_remapped;
            axis_angle_remapped(0) = axis_angle_raw(map_roll_) * sign_roll_ * haptic_scale_rot_;
            axis_angle_remapped(1) = axis_angle_raw(map_pitch_) * sign_pitch_ * haptic_scale_rot_;
            axis_angle_remapped(2) = axis_angle_raw(map_yaw_) * sign_yaw_ * haptic_scale_rot_;
            
            double angle = axis_angle_remapped.norm();
            Eigen::Matrix3d R_offset;
            if (angle > 1e-6) {
                R_offset = Eigen::AngleAxisd(angle, axis_angle_remapped.normalized()).toRotationMatrix();
            } else {
                R_offset = Eigen::Matrix3d::Identity();
            }
            R_des = R_start_ * R_offset;
            
        } else {
            p_des = p_start_;
            R_des = R_start_;
        }
        
        // Anti-drift protection
        Eigen::VectorXd dq = q_cmd_ - q_current_;
        if(dq.norm() > 0.5) {
            RCLCPP_WARN(this->get_logger(), "Drift detected! Resetting to current state.");
            q_cmd_ = q_current_;
        }
        
        // Inverse kinematics loop
        const int max_iters = 10;
        for(int iter=0; iter<max_iters; ++iter) {
            pinocchio::forwardKinematics(model_, data_, q_cmd_);
            pinocchio::framesForwardKinematics(model_, data_, q_cmd_);
            
            Eigen::MatrixXd J(6, nq_);
            pinocchio::computeFrameJacobian(model_, data_, q_cmd_, ee_id_, 
                                           pinocchio::LOCAL_WORLD_ALIGNED, J);
            
            // Position error
            Eigen::Vector3d e_pos = p_des - data_.oMf[ee_id_].translation();
            
            // Orientation error
            Eigen::Matrix3d R_err = R_des * data_.oMf[ee_id_].rotation().transpose();
            Eigen::Vector3d e_rot;
            e_rot << R_err(2,1) - R_err(1,2),
                     R_err(0,2) - R_err(2,0),
                     R_err(1,0) - R_err(0,1);
            e_rot *= 0.5;
            
            Eigen::VectorXd e(6);
            e << e_pos, e_rot;
            
            if(e.norm() < 1e-4) break;
            
            // Damped least squares
            double damping = 0.01;
            Eigen::MatrixXd H = J.transpose() * J + damping * Eigen::MatrixXd::Identity(nq_, nq_);
            Eigen::VectorXd dq_step = H.ldlt().solve(J.transpose() * e);
            
            // Limit step size
            double max_step = 0.05;
            if(dq_step.norm() > max_step) {
                dq_step = dq_step.normalized() * max_step;
            }
            
            q_cmd_ += dq_step;
        }
        
        // Apply joint limits
        for(int i=0; i<nq_; ++i) {
            q_cmd_(i) = std::clamp(q_cmd_(i), q_min_(i), q_max_(i));
        }
        
        // Publish trajectory
        trajectory_msgs::msg::JointTrajectory cmd;
        cmd.header.stamp = this->now();
        cmd.joint_names = joint_names_;
        
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(q_cmd_.data(), q_cmd_.data() + nq_);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.02);
        cmd.points.push_back(pt);
        
        pub_cmd_->publish(cmd);
        
        // CSV logging
        if(tracking_active_ && haptic_initialized_) {
            Eigen::Vector3d phantom_offset_raw = phantom_pos_current_ - phantom_pos_initial_;
            Eigen::Vector3d current_pos = data_.oMf[ee_id_].translation();
            double error_pos = (p_des - current_pos).norm();
            
            Eigen::Matrix3d R_current = data_.oMf[ee_id_].rotation();
            Eigen::Matrix3d R_error = R_des * R_current.transpose();
            Eigen::AngleAxisd aa_error(R_error);
            double error_rot = aa_error.angle();
            
            csv_file_ << std::fixed << std::setprecision(6)
                     << t << ","
                     << phantom_offset_raw(0) << "," << phantom_offset_raw(1) << "," << phantom_offset_raw(2) << ","
                     << current_pos(0) << "," << current_pos(1) << "," << current_pos(2) << ","
                     << error_pos << "," << error_rot << "\n";
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptTeleop>());
    rclcpp::shutdown();
    return 0;
}
