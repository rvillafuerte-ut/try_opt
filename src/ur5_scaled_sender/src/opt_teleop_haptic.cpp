#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <fstream>
#include <iomanip>

using namespace std::chrono_literals;

class OptTeleop : public rclcpp::Node {
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_id_;
    int nq_;
    
    Eigen::VectorXd q_current_, q_cmd_, dq_current_;
    Eigen::VectorXd q_cmd_prev_; // To calculate command velocity
    Eigen::Matrix3d R_start_;
    Eigen::Vector3d p_start_;
    bool robot_initialized_ = false;
    bool haptic_initialized_ = false;
    
    Eigen::Vector3d phantom_pos_initial_, phantom_pos_current_;
    Eigen::Quaterniond phantom_quat_initial_, phantom_quat_current_;
    
    // Filter variables
    Eigen::Vector3d phantom_pos_filtered_;
    Eigen::Quaterniond phantom_quat_filtered_;
    bool filter_initialized_ = false;

    std::ofstream csv_file_;
    double t0_;

    // Parameters
    double scale_pos_, scale_rot_;
    double filter_gain_, max_joint_vel_;
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
        csv_file_.open("/home/utec/try_opt/teleop_data.csv");
        // Header: time, qc0-5, qa0-5, dqa0-5, p_des_xyz, p_act_xyz
        csv_file_ << "time,qc0,qc1,qc2,qc3,qc4,qc5,qa0,qa1,qa2,qa3,qa4,qa5,dqa0,dqa1,dqa2,dqa3,dqa4,dqa5,pd0,pd1,pd2,pa0,pa1,pa2\n";
        t0_ = this->now().seconds();

        // 1. Parameters
        auto urdf_path = this->declare_parameter<std::string>("urdf_path", "/home/utec/try_opt/urdf/ur5.urdf");
        double ctrl_hz = this->declare_parameter<double>("ctrl_hz", 500.0); // Increased to 500Hz for smoother control
        
        scale_pos_ = this->declare_parameter<double>("haptic_scale_pos", 1.0);
        scale_rot_ = this->declare_parameter<double>("haptic_scale_rot", 1.0);
        filter_gain_ = this->declare_parameter<double>("filter_gain", 0.8); // Increased for faster response (was 0.1)
        max_joint_vel_ = this->declare_parameter<double>("max_joint_vel", 2.5); // Increased limit (was 0.7)
        
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
        q_cmd_prev_ = Eigen::VectorXd::Zero(nq_);
        dq_current_ = Eigen::VectorXd::Zero(nq_);
        
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
                        size_t idx = std::distance(msg->name.begin(), it);
                        q_current_(i) = msg->position[idx];
                        if(msg->velocity.size() > idx) dq_current_(i) = msg->velocity[idx];
                    }
                }
                
                if(!robot_initialized_ && q_current_.norm() > 0.001) {
                    q_cmd_ = q_current_;
                    q_cmd_prev_ = q_current_;
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
                
                // Apply Low-Pass Filter
                if (!filter_initialized_) {
                    phantom_pos_filtered_ = phantom_pos_current_;
                    phantom_quat_filtered_ = phantom_quat_current_;
                    filter_initialized_ = true;
                } else {
                    phantom_pos_filtered_ = phantom_pos_filtered_ * (1.0 - filter_gain_) + phantom_pos_current_ * filter_gain_;
                    phantom_quat_filtered_ = phantom_quat_filtered_.slerp(filter_gain_, phantom_quat_current_);
                }

                if(robot_initialized_ && !haptic_initialized_) {
                    phantom_pos_initial_ = phantom_pos_filtered_;
                    phantom_quat_initial_ = phantom_quat_filtered_;
                    haptic_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Haptic Initialized.");
                }
            });
        
        pub_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);
        
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/ctrl_hz), 
                                       std::bind(&OptTeleop::control_loop, this));
    }

    ~OptTeleop() { if(csv_file_.is_open()) csv_file_.close(); }

private:
    void control_loop() {
        if(!haptic_initialized_) return;
        
        // --- 1. Calculate Desired Pose ---
        Eigen::Vector3d p_des = p_start_;
        Eigen::Matrix3d R_des = R_start_;
        
        // Position Mapping (Using Filtered Data)
        Eigen::Vector3d d_pos_raw = phantom_pos_filtered_ - phantom_pos_initial_;
        Eigen::Vector3d d_pos_map;
        for(int i=0; i<3; i++) d_pos_map(i) = d_pos_raw(map_pos_(i)) * sign_pos_(i) * scale_pos_;
        p_des += R_start_ * d_pos_map; // Apply relative to initial tool orientation
        
        // Orientation Mapping (Local Difference using Filtered Data)
        Eigen::Quaterniond d_quat = phantom_quat_initial_.inverse() * phantom_quat_filtered_;
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
            Eigen::VectorXd dq = H.ldlt().solve(J.transpose() * e);
            
            // Limit Max Angular Velocity (Per-Joint Scaling to preserve direction)
            double dt = 1.0 / 500.0;
            double max_step = max_joint_vel_ * dt;
            
            double max_dq = dq.cwiseAbs().maxCoeff();
            if (max_dq > max_step) {
                dq *= (max_step / max_dq);
            }
            
            q_cmd_ += dq;
        }
        
        // --- 3. Publish ---
        for(int i=0; i<nq_; ++i) q_cmd_(i) = std::clamp(q_cmd_(i), q_min_(i), q_max_(i));
        
        // Calculate velocity command (Optional: Commented out for safety)
        /*
        double dt = 1.0 / 500.0;
        Eigen::VectorXd v_cmd = (q_cmd_ - q_cmd_prev_) / dt;
        q_cmd_prev_ = q_cmd_;
        */

        trajectory_msgs::msg::JointTrajectory cmd;
        // Use Time(0) to execute immediately (ignore synchronization issues)
        cmd.header.stamp = rclcpp::Time(0); 
        cmd.joint_names = joint_names_;
        cmd.points.resize(1);
        cmd.points[0].positions.assign(q_cmd_.data(), q_cmd_.data() + nq_);
        // cmd.points[0].velocities.assign(v_cmd.data(), v_cmd.data() + nq_); // Disabled for safety
        // time_from_start is relative to "now" since header.stamp is 0
        cmd.points[0].time_from_start = rclcpp::Duration::from_seconds(0.01); // Increased slightly for stability
        pub_cmd_->publish(cmd);

        if(csv_file_.is_open()) {
            Eigen::Vector3d p_act = data_.oMf[ee_id_].translation();
            csv_file_ << std::fixed << std::setprecision(4) << (this->now().seconds() - t0_);
            for(int i=0; i<nq_; ++i) csv_file_ << "," << q_cmd_(i);
            for(int i=0; i<nq_; ++i) csv_file_ << "," << q_current_(i);
            for(int i=0; i<nq_; ++i) csv_file_ << "," << dq_current_(i);
            csv_file_ << "," << p_des(0) << "," << p_des(1) << "," << p_des(2);
            csv_file_ << "," << p_act(0) << "," << p_act(1) << "," << p_act(2);
            csv_file_ << "\n";
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptTeleop>());
    rclcpp::shutdown();
    return 0;
}
