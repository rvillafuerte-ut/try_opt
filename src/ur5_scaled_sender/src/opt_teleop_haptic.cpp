#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

using namespace std::chrono_literals;

class OptTeleop : public rclcpp::Node {
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_id_;
    int nq_;
    
    Eigen::VectorXd q_current_, q_cmd_;
    Eigen::Matrix3d R_start_;
    Eigen::Vector3d p_start_;
    bool robot_initialized_ = false;
    bool haptic_initialized_ = false;
    
    Eigen::Vector3d phantom_pos_initial_, phantom_pos_current_;
    Eigen::Quaterniond phantom_quat_initial_, phantom_quat_current_;
    
    // Parameters
    double scale_pos_, scale_rot_;
    Eigen::Vector3d sign_pos_, sign_rot_;
    Eigen::Vector3i map_pos_, map_rot_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_phantom_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    const std::vector<std::string> joint_names_ = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    
    Eigen::VectorXd q_min_, q_max_;

public:
    OptTeleop() : Node("opt_teleop_haptic"), data_(model_) {
        // 1. Parameters
        auto urdf_path = this->declare_parameter<std::string>("urdf_path", "/home/utec/try_opt/urdf/ur5.urdf");
        double ctrl_hz = this->declare_parameter<double>("ctrl_hz", 100.0);
        
        scale_pos_ = this->declare_parameter<double>("haptic_scale_pos", 1.0);
        scale_rot_ = this->declare_parameter<double>("haptic_scale_rot", 1.0);
        
        // Mapping: Robot Index <- Phantom Index
        map_pos_ = {this->declare_parameter<int>("map_x", 2),
                    this->declare_parameter<int>("map_y", 0),
                    this->declare_parameter<int>("map_z", 1)};
        sign_pos_ = {this->declare_parameter<double>("sign_x", -1.0),
                     this->declare_parameter<double>("sign_y", -1.0),
                     this->declare_parameter<double>("sign_z", 1.0)};

        map_rot_ = {this->declare_parameter<int>("map_roll", 2),
                    this->declare_parameter<int>("map_pitch", 0),
                    this->declare_parameter<int>("map_yaw", 1)};
        sign_rot_ = {this->declare_parameter<double>("sign_roll", 1.0),
                     this->declare_parameter<double>("sign_pitch", 1.0),
                     this->declare_parameter<double>("sign_yaw", 1.0)};
        
        // 2. Setup Pinocchio
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        ee_id_ = model_.getFrameId("tool0");
        nq_ = model_.nv;
        
        q_current_ = Eigen::VectorXd::Zero(nq_);
        q_cmd_ = Eigen::VectorXd::Zero(nq_);
        
        // Limits
        q_min_ = Eigen::VectorXd::Constant(nq_, -2*M_PI); q_min_[2] = -M_PI;
        q_max_ = Eigen::VectorXd::Constant(nq_,  2*M_PI); q_max_[2] =  M_PI;
        
        // 3. Subscribers
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                // Map disordered joint names
                for(size_t i=0; i<joint_names_.size(); ++i) {
                    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    if(it != msg->name.end()) {
                        q_current_(i) = msg->position[std::distance(msg->name.begin(), it)];
                    }
                }
                
                if(!robot_initialized_ && q_current_.norm() > 0.001) {
                    q_cmd_ = q_current_;
                    pinocchio::forwardKinematics(model_, data_, q_current_);
                    pinocchio::framesForwardKinematics(model_, data_, q_current_);
                    p_start_ = data_.oMf[ee_id_].translation();
                    R_start_ = data_.oMf[ee_id_].rotation();
                    robot_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Robot Initialized.");
                }
            });
        
        sub_phantom_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/phantom/pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                phantom_pos_current_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
                phantom_quat_current_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
                
                if(robot_initialized_ && !haptic_initialized_) {
                    phantom_pos_initial_ = phantom_pos_current_;
                    phantom_quat_initial_ = phantom_quat_current_;
                    haptic_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Haptic Initialized.");
                }
            });
        
        pub_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/ctrl_hz), 
                                       std::bind(&OptTeleop::control_loop, this));
    }

private:
    void control_loop() {
        if(!haptic_initialized_) return;
        
        // --- 1. Calculate Desired Pose ---
        Eigen::Vector3d p_des = p_start_;
        Eigen::Matrix3d R_des = R_start_;
        
        // Position Mapping
        Eigen::Vector3d d_pos_raw = phantom_pos_current_ - phantom_pos_initial_;
        Eigen::Vector3d d_pos_map;
        for(int i=0; i<3; i++) d_pos_map(i) = d_pos_raw(map_pos_(i)) * sign_pos_(i) * scale_pos_;
        p_des += R_start_ * d_pos_map; // Apply relative to initial tool orientation
        
        // Orientation Mapping (Local Difference)
        Eigen::Quaterniond d_quat = phantom_quat_initial_.inverse() * phantom_quat_current_;
        Eigen::AngleAxisd aa(d_quat);
        Eigen::Vector3d axis_raw = aa.axis() * aa.angle();
        Eigen::Vector3d axis_map;
        for(int i=0; i<3; i++) axis_map(i) = axis_raw(map_rot_(i)) * sign_rot_(i) * scale_rot_;
        
        if (axis_map.norm() > 1e-6) {
            R_des = R_start_ * Eigen::AngleAxisd(axis_map.norm(), axis_map.normalized()).toRotationMatrix();
        }

        // --- 2. Inverse Kinematics (Damped Least Squares) ---
        for(int iter=0; iter<5; ++iter) {
            pinocchio::forwardKinematics(model_, data_, q_cmd_);
            pinocchio::framesForwardKinematics(model_, data_, q_cmd_);
            
            Eigen::MatrixXd J(6, nq_);
            pinocchio::computeFrameJacobian(model_, data_, q_cmd_, ee_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
            
            Eigen::Vector3d ep = p_des - data_.oMf[ee_id_].translation();
            Eigen::Matrix3d Re = R_des * data_.oMf[ee_id_].rotation().transpose();
            Eigen::Vector3d er;
            er << Re(2,1)-Re(1,2), Re(0,2)-Re(2,0), Re(1,0)-Re(0,1);
            er *= 0.5;
            
            Eigen::VectorXd e(6); e << ep, er;
            if(e.norm() < 1e-4) break;
            
            Eigen::MatrixXd H = J.transpose() * J + 1e-3 * Eigen::MatrixXd::Identity(nq_, nq_);
            q_cmd_ += H.ldlt().solve(J.transpose() * e);
        }
        
        // --- 3. Publish ---
        for(int i=0; i<nq_; ++i) q_cmd_(i) = std::clamp(q_cmd_(i), q_min_(i), q_max_(i));
        
        trajectory_msgs::msg::JointTrajectory cmd;
        cmd.header.stamp = this->now();
        cmd.joint_names = joint_names_;
        cmd.points.resize(1);
        cmd.points[0].positions.assign(q_cmd_.data(), q_cmd_.data() + nq_);
        cmd.points[0].time_from_start = rclcpp::Duration::from_seconds(0.02);
        pub_cmd_->publish(cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptTeleop>());
    rclcpp::shutdown();
    return 0;
}
