#include <ur5_kinematics/kinematics.hpp>
#include <ur5_impedance/impedance.hpp>
#include <ur5_sliding/sliding.hpp>
#include <ur5_controller/structs.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// Mensajes
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <omni_msgs/msg/omni_button_event.hpp>
#include <omni_msgs/msg/omni_state.hpp>
#include <omni_msgs/msg/omni_feedback.hpp>

// Eigen para matemáticas
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Utilidades de ROS 2
#include <ament_index_cpp/get_package_share_directory.hpp>

// Standard C++
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <unordered_map>
#include <filesystem>
#include <iomanip>
#include <ctime>

// Usar el namespace para las structs
using namespace ur5_controller;


class TrajectoryGenerator {
public:
    // Estructura para contener los resultados
    struct State {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };

    // Calcula el estado completo de la trayectoria en un tiempo dado
    static State calculate(
        const Eigen::Vector3d& x_init,
        const Eigen::Vector3d& A,
        double wn,
        double c0,
        double time_elapsed,
        int grafica)
    {
        State state;
        double t = time_elapsed;

        // --- Términos comunes (se calculan una sola vez) ---
        double exp_neg_c0_t = exp(-c0 * t);
        double sin_wn_t = sin(wn * t);
        double cos_wn_t = cos(wn * t);
        
        double amp_factor = 1.0 - exp_neg_c0_t;
        double d_amp_factor_dt = c0 * exp_neg_c0_t;
        double d2_amp_factor_dt2 = -c0 * c0 * exp_neg_c0_t;
        // ----------------------------------------------------
        if (grafica ==1) {
            // --- Cálculo de Posición ---
            state.position.x() = x_init.x() + A.x() * amp_factor * sin_wn_t;
            state.position.y() = x_init.y() + A.y() * amp_factor * cos_wn_t;
            state.position.z() = x_init.z() + A.z() * amp_factor * sin_wn_t;

            // --- Cálculo de Velocidad ---
            state.velocity.x() = A.x() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);
            state.velocity.y() = A.y() * (d_amp_factor_dt * cos_wn_t - amp_factor * wn * sin_wn_t);
            state.velocity.z() = A.z() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);

            // --- Cálculo de Aceleración ---
            double term1_sin = d2_amp_factor_dt2 - amp_factor * wn * wn;
            double term2_cos = 2 * d_amp_factor_dt * wn;
            state.acceleration.x() = A.x() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
            state.acceleration.y() = A.y() * (term1_sin * cos_wn_t - term2_cos * sin_wn_t);
            state.acceleration.z() = A.z() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
        }
        else if (grafica == 2){
            state.position.x() = x_init.x() -0.3 + 0.3*exp_neg_c0_t;
            state.position.y() = x_init.y() + 0.1 - 0.1*exp_neg_c0_t;
            state.position.z() = x_init.z() + 0.1 - 0.1*exp_neg_c0_t;

            state.velocity.x() = -0.3 * (-c0) * exp_neg_c0_t;
            state.velocity.y() = 0.1 * (-c0) * exp_neg_c0_t;
            state.velocity.z() = 0.1 * (-c0) * exp_neg_c0_t;

            state.acceleration.x() = -0.3 * (c0 * c0) * exp_neg_c0_t;
            state.acceleration.y() = 0.1 * (c0 * c0) * exp_neg_c0_t;
            state.acceleration.z() = 0.1 * (c0 * c0) * exp_neg_c0_t;
        }
        // grafica == 3 eliminado

        return state;
    }
};

Eigen::Vector3d trayectoria_geomagic(Eigen::Vector3d x_init, Eigen::Vector3d r_init,Eigen::Vector3d r_actual, double escala){
    Eigen::Vector3d x_des = Eigen::Vector3d::Zero();
    x_des[0] = (r_actual[1]-r_init[1])*escala + x_init[0];
    x_des[1] = (x_init[1]- (r_actual[0]-r_init[0])*escala);
    x_des[2] = (r_actual[2]-r_init[2])*escala + x_init[2];
    return x_des;
};


void initializeUR5(PinocchioResources& pinocchio, const std::string& urdf_path) {
    pinocchio.model = std::make_unique<pinocchio::Model>();

    auto logger = rclcpp::get_logger("UR5Kinematics");
    RCLCPP_INFO(logger, "Intentando cargar URDF desde: %s", urdf_path.c_str());

    try {
        pinocchio::urdf::buildModel(urdf_path, *pinocchio.model);
        RCLCPP_INFO(logger, "URDF cargado exitosamente!");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error cargando URDF: %s", e.what());
        throw;
    }

    pinocchio.data = std::make_unique<pinocchio::Data>(*pinocchio.model);
    pinocchio.tool_frame_id = pinocchio.model->getFrameId("tool0");

    if (pinocchio.tool_frame_id == static_cast<pinocchio::FrameIndex>(pinocchio.model->nframes)) {
        RCLCPP_ERROR(logger, "Error: Marco 'tool0' no encontrado en el URDF!");
        throw std::runtime_error("Frame tool0 no encontrado");
    }
}


std::string get_file_path(const std::string& package_name, const std::string& relative_path) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        return package_share_directory + "/" + relative_path;
    } catch (const std::exception& e) {
        throw std::runtime_error("No se pudo encontrar el paquete: " + package_name);
    }
}


class UR5IKNode : public rclcpp::Node
{
public:
  UR5IKNode() : Node("ur5_ik_node")
  {
    // 1. Declarar parámetros
    this->declare_parameter<std::string>("control_topic", config_.control_topic);
    this->declare_parameter<bool>("geomagic", config_.use_geomagic);
    this->declare_parameter<std::string>("ur", config_.ur_model);
    this->declare_parameter<std::string>("nmspace", config_.nmspace);
    this->declare_parameter<std::string>("urdf_path", "");
    this->declare_parameter<std::string>("geomagic_topic", "/phantom/pose");
    this->declare_parameter<std::string>("geomagic_button_topic", "/phantom/button");
    // Movimiento inicial
    this->declare_parameter<bool>("use_ur5_pos_init", config_.use_ur5_pos_init);
    this->declare_parameter<std::vector<double>>("q_target", config_.q_target);
    this->declare_parameter<double>("q_target_time", config_.q_target_time);
    // CSV
    this->declare_parameter<bool>("csv_log_enable", config_.csv_enabled);
    this->declare_parameter<std::string>("csv_log_dir", config_.csv_path);
    this->declare_parameter<std::string>("csv_log_prefix", config_.csv_prefix);
    // Trayectoria automática
    this->declare_parameter<std::vector<double>>("traj_A", {config_.traj_A.x(), config_.traj_A.y(), config_.traj_A.z()});
    this->declare_parameter<double>("traj_wn", config_.traj_wn);
    this->declare_parameter<double>("traj_c0", config_.traj_c0);
    this->declare_parameter<int>("traj_mode", config_.traj_mode);

    // 2. Obtener parámetros y guardarlos en la struct de configuración
    this->get_parameter("control_topic", config_.control_topic);
    this->get_parameter("geomagic", config_.use_geomagic);
    this->get_parameter("ur", config_.ur_model);
    this->get_parameter("nmspace", config_.nmspace);
    this->get_parameter("use_ur5_pos_init", config_.use_ur5_pos_init);
    this->get_parameter("q_target", config_.q_target);
    this->get_parameter("q_target_time", config_.q_target_time);
    this->get_parameter("csv_log_enable", config_.csv_enabled);
    this->get_parameter("csv_log_dir", config_.csv_path);
    this->get_parameter("csv_log_prefix", config_.csv_prefix);
    // Obtener parámetros de trayectoria
    std::vector<double> A_param;
    this->get_parameter("traj_A", A_param);
    if (A_param.size() == 3) {
        config_.traj_A = Eigen::Vector3d(A_param[0], A_param[1], A_param[2]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Parametro traj_A debe tener tamaño 3. Usando valores por defecto.");
    }
    this->get_parameter("traj_wn", config_.traj_wn);
    this->get_parameter("traj_c0", config_.traj_c0);
    this->get_parameter("traj_mode", config_.traj_mode);
    if (config_.traj_mode < 1 || config_.traj_mode > 3) {
        RCLCPP_WARN(this->get_logger(), "traj_mode fuera de rango (%d). Se usará 1.", config_.traj_mode);
        config_.traj_mode = 1;
    }
    
    std::string urdf_param;
    std::string geomagic_topic;
    std::string geomagic_button_topic;

    this->get_parameter("geomagic_button_topic", geomagic_button_topic);
    this->get_parameter("geomagic_topic", geomagic_topic);
    this->get_parameter("urdf_path", urdf_param);
    if (urdf_param.empty()) {
        config_.urdf_path = get_file_path("ur5_description", "urdf/" + config_.ur_model + ".urdf");
    } else {
        config_.urdf_path = urdf_param;
    }

    // Inicializar Pinocchio
    initializeUR5(pinocchio_, config_.urdf_path);
    kinematics_solver_ = std::make_unique<UR5Kinematics>(config_.urdf_path);
    impedance_controller_ = std::make_unique<UR5Impedance>(config_.urdf_path);
    std::string c_topic = "/" + config_.nmspace + config_.control_topic;
    std::string joint_states_topic = config_.nmspace.empty() ? std::string("/joint_states")
                                                            : std::string("/") + config_.nmspace + std::string("/joint_states");

    RCLCPP_INFO(this->get_logger(), "Publicando en el tópico de control: '%s'", c_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Usando modelo UR: '%s'", config_.ur_model.c_str());
    RCLCPP_INFO(this->get_logger(), "Usando URDF en: '%s'", config_.urdf_path.c_str());
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(c_topic, 10);
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&UR5IKNode::update_joint_positions, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Suscrito a joint_states: '%s'", joint_states_topic.c_str());
    // Suscripción de respaldo al tópico global en caso de que el driver no use namespace
    if (!config_.nmspace.empty() && joint_states_topic != "/joint_states") {
        joint_states_sub_global_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR5IKNode::update_joint_positions, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Suscripción de respaldo a joint_states: '/joint_states'");
    }

    if (config_.use_geomagic) {
        geomagic_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(geomagic_topic, 10, std::bind(&UR5IKNode::pose_callback, this, std::placeholders::_1));
        subscription_phantom_button_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(geomagic_button_topic, 10, std::bind(&UR5IKNode::button_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Usando tópico de Geomagic: '%s'", geomagic_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Usando tópico de botón Geomagic: '%s'", geomagic_button_topic.c_str());
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Modo Geomagic deshabilitado. Seguimiento de la trayectoria predefinida.");
        // Programar movimiento inicial tipo ur5_pos si está habilitado
        if (config_.use_ur5_pos_init) {
            init_move_active_ = true;            // activar modo publicación hasta detectar movimiento
            init_baseline_set_ = false;          // baseline de joints se tomará al recibir JointState
            // Publicar periódicamente hasta detectar movimiento
            init_move_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),  // 10 Hz
                std::bind(&UR5IKNode::init_move_tick, this)
            );
            RCLCPP_INFO(this->get_logger(), "Modo ur5_pos activo: publicando objetivo hasta detectar movimiento (umbral %.3f rad)", move_detect_threshold_);
        }
        // Trayectoria automática se activará una vez termine movimiento inicial o inmediatamente si no se usa init
        if (!config_.use_ur5_pos_init) {
            trajectory_active_ = false; // se activará tras captura de pose
        }
    }

    // Inicializar CSV si está habilitado
    if (config_.csv_enabled) {
        init_csv_logger();
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&UR5IKNode::control_loop, this));
  }

private:
    // Suscriptores y publicadores
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_global_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr geomagic_pose_sub_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr subscription_phantom_button_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;  

    // --- Variables agrupadas en structs ---
    RobotState robot_state_;
    CartesianState cartesian_state_;
    HapticState haptic_state_;
    NodeConfig config_;
    PinocchioResources pinocchio_;
    // ------------------------------------

    // Banderas de estado
    bool pose_inicial_capturada_ = false;
    bool posicion_inicial_alcanzada_ = false;
    bool capturar_pose_inicial_haptico_ = false;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<UR5Kinematics> kinematics_solver_;
    std::unique_ptr<UR5Impedance> impedance_controller_;
   

    // ---- Mapeo robusto de joints por nombre ----
    std::vector<int> joint_index_map_{};          // índice en msg->name para cada joint esperado
    bool joint_map_initialized_ = false;
    std::vector<std::string> last_js_names_{};    // para detectar cambios y reconstruir el mapa

    // ---- CSV logging ----
    //bool csv_enabled = false;
    //std::string csv_path;
    //std::string config_.csv_prefix = "ur5_log";
    std::string csv_file_path_;
    std::ofstream csv_file_;
    rclcpp::Time start_time_{};
    rclcpp::Time last_log_time_{};
    // Métricas de tiempo por ciclo
    double last_ik_ms_ {0.0};
    double last_loop_ms_ {0.0};

    // ---- Movimiento inicial tipo ur5_pos ----
    // (Variables trasladadas a config_)
    bool init_pose_published_ = false; // legado (no usado en modo repetitivo)
    bool init_move_active_ = false;    // publicar hasta detectar movimiento
    bool init_baseline_set_ = false;   // baseline ya capturada
    std::vector<double> q_baseline_{0.0,0.0,0.0,0.0,0.0,0.0};
    double move_detect_threshold_ = 0.010; // rad, ~0.57 deg
    rclcpp::TimerBase::SharedPtr init_move_timer_;
    // Detección de llegada a objetivo articular inicial
    bool init_movement_started_ = false;     // ya comenzó a moverse hacia el objetivo
    double reach_threshold_rad_ = 0.020;     // rad, condición de cercanía a q_target
    int reach_count_ = 0;                    // conteo de comprobaciones consecutivas en umbral
    int reach_count_required_ = 5;           // número de comprobaciones consecutivas para confirmar llegada
    // ---- Trayectoria automática sin geomagic ----
    // (Parámetros de trayectoria ahora en config_)
    bool trajectory_active_ {false};
    rclcpp::Time trajectory_start_time_ {};
    double t_traj_ = 0.0; // Tiempo para la trayectoria, independiente de dt

    Eigen::Matrix<double, 7, 1> Kp;
    Eigen::Matrix<double, 7, 1> Kd;

    static std::string get_home_dir() {
        const char* home = std::getenv("HOME");
        return home ? std::string(home) : std::string(".");
    }

    std::string now_timestamp_string() {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&tt, &tm);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    void init_csv_logger() {
        // Directorio por defecto: ~/.ros/ur5_logs
        if (config_.csv_path.empty()) {
            config_.csv_path = get_home_dir() + std::string("/.ros/ur5_logs");
        }
        std::error_code ec;
        std::filesystem::create_directories(config_.csv_path, ec);
        if (ec) {
            RCLCPP_WARN(this->get_logger(), "No se pudo crear el directorio de CSV '%s': %s", config_.csv_path.c_str(), ec.message().c_str());
        }

        // Nombre de archivo único: <prefix>_<ns>_<YYYYMMDD_HHMMSS>.csv
        std::string ns_part = config_.nmspace.empty() ? std::string("nonamespace") : config_.nmspace;
        std::string fname = config_.csv_prefix + std::string("_") + ns_part + std::string("_") + now_timestamp_string() + std::string(".csv");
        csv_file_path_ = (std::filesystem::path(config_.csv_path) / fname).string();

        csv_file_.open(csv_file_path_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir archivo CSV: %s", csv_file_path_.c_str());
            config_.csv_enabled = false;
            return;
        }

        // Tiempos base
        start_time_ = this->now();
        last_log_time_ = start_time_;

    // Encabezado
    csv_file_ << "t,dt"
        << ",q_meas_0,q_meas_1,q_meas_2,q_meas_3,q_meas_4,q_meas_5"
        << ",q_cmd_0,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5"
        << ",e_q_0,e_q_1,e_q_2,e_q_3,e_q_4,e_q_5"
        << ",x_des_x,x_des_y,x_des_z"
        << ",x_meas_x,x_meas_y,x_meas_z"
        << ",q_des_w,q_des_x,q_des_y,q_des_z"
        << ",q_meas_w,q_meas_x,q_meas_y,q_meas_z"
        << ",euler_des_r,euler_des_p,euler_des_y"
        << ",euler_meas_r,euler_meas_p,euler_meas_y"
        << ",e_R_angle"
        << ",pos_err_x,pos_err_y,pos_err_z"
        << ",ori_err_axis_x,ori_err_axis_y,ori_err_axis_z,ori_err_angle"
        << ",control_loop_time"
        << ",ik_ms,loop_ms"
        << std::endl;

        RCLCPP_INFO(this->get_logger(), "CSV logging habilitado: %s", csv_file_path_.c_str());
    }

    void write_csv_row(double t, double dt,
                       const Eigen::Vector3d& x_des,
                       const Eigen::Vector3d& x_meas,
                       const Eigen::Quaterniond& q_des,
                       const Eigen::Quaterniond& q_meas,
                       const Eigen::Vector3d& euler_des,
                       const Eigen::Vector3d& euler_meas,
                       double e_R_angle,
                       const Eigen::Vector3d& pos_err,
                       const Eigen::Vector3d& ori_err_axis,
                       double ori_err_angle,
                       double ik_ms,
                       double loop_ms) {
        if (!config_.csv_enabled || !csv_file_.is_open()) return;

        csv_file_ << std::fixed << std::setprecision(6)
                  << t << "," << dt;

        // q_meas
        for (int i = 0; i < 6; ++i) csv_file_ << "," << robot_state_.q[i];
        // q_cmd
        for (int i = 0; i < 6; ++i) csv_file_ << "," << robot_state_.q_solution[i];
        // e_q = q_cmd - q_meas
        for (int i = 0; i < 6; ++i) csv_file_ << "," << (robot_state_.q_solution[i] - robot_state_.q[i]);

        // x_des, x_meas
        csv_file_ << "," << x_des.x() << "," << x_des.y() << "," << x_des.z();
        csv_file_ << "," << x_meas.x() << "," << x_meas.y() << "," << x_meas.z();
        // quaternions deseado/medido
        csv_file_ << "," << q_des.w() << "," << q_des.x() << "," << q_des.y() << "," << q_des.z();
        csv_file_ << "," << q_meas.w() << "," << q_meas.x() << "," << q_meas.y() << "," << q_meas.z();
        // euler deseado/medido (roll, pitch, yaw)
        csv_file_ << "," << euler_des.x() << "," << euler_des.y() << "," << euler_des.z();
        csv_file_ << "," << euler_meas.x() << "," << euler_meas.y() << "," << euler_meas.z();
    // e_R_angle
        csv_file_ << "," << e_R_angle;
    // Error cartesiano de posición (desired - measured)
    csv_file_ << "," << pos_err.x() << "," << pos_err.y() << "," << pos_err.z();
    // Error de orientación (eje normalizado y ángulo)
    csv_file_ << "," << ori_err_axis.x() << "," << ori_err_axis.y() << "," << ori_err_axis.z() << "," << ori_err_angle;
    // control_loop_time
        csv_file_ << "," << config_.control_loop_time;

        // métricas de tiempos
        csv_file_ << "," << ik_ms << "," << loop_ms;

        csv_file_ << std::endl;
    }

    // Publica una sola vez una trayectoria a q_target_ similar al nodo ur5_pos
    void publish_initial_joint_position() {
        if (config_.q_target.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "q_target debe tener 6 elementos, tiene %zu", config_.q_target.size());
            return;
        }
        std::string prefix = config_.nmspace.empty() ? std::string("") : (config_.nmspace + std::string("_"));
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {prefix + "shoulder_pan_joint", prefix + "shoulder_lift_joint", prefix + "elbow_joint",
                            prefix + "wrist_1_joint", prefix + "wrist_2_joint", prefix + "wrist_3_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = config_.q_target;
        pt.time_from_start = rclcpp::Duration::from_seconds(config_.q_target_time);
        RCLCPP_INFO(this->get_logger(), "Target: %f, %f, %f, %f, %f, %f", config_.q_target[0], config_.q_target[1], config_.q_target[2], config_.q_target[3], config_.q_target[4], config_.q_target[5]);
        traj.points.push_back(pt);
        joint_trajectory_pub_->publish(traj);
        RCLCPP_INFO(this->get_logger(), "Movimiento inicial publicado (ur5_pos-like) en %0.2fs", config_.q_target_time);
    }

    // Tick periódico: publica mientras no detecte movimiento respecto a baseline
    void init_move_tick() {
        if (!init_move_active_) return;
        // Asegurar baseline al recibir primeras articulaciones
        if (!init_baseline_set_) {
            // Necesitamos haber recibido joint_states y tener un mapeo válido
            if (last_joint_state_ && joint_map_initialized_) {
                for (int i = 0; i < 6; ++i) q_baseline_[i] = robot_state_.q[i];
                init_baseline_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Baseline articular capturada para detección de movimiento.");
            }
        }

        // Detectar inicio de movimiento respecto a baseline
        if (!init_movement_started_ && init_baseline_set_) {
            for (int i = 0; i < 6; ++i) {
                if (std::abs(robot_state_.q[i] - q_baseline_[i]) > move_detect_threshold_) {
                    init_movement_started_ = true;
                    RCLCPP_INFO(this->get_logger(), "Movimiento inicial detectado (> %.3f rad). Continuando hasta alcanzar q_target.", move_detect_threshold_);
                    break;
                }
            }
        }

        // Verificar llegada a q_target con histéresis (consecutivo)
        double max_err = 0.0;
        for (int i = 0; i < 6; ++i) {
            max_err = std::max(max_err, std::abs(robot_state_.q[i] - config_.q_target[i]));
        }
        if (max_err < reach_threshold_rad_) {
            reach_count_++;
        } else {
            reach_count_ = 0;
        }

        if (reach_count_ >= reach_count_required_) {
            init_move_active_ = false;
            if (init_move_timer_) init_move_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Objetivo articular alcanzado (|e|_max < %.3f rad por %d ciclos).", reach_threshold_rad_, reach_count_required_);
            // Activar trayectoria automática desde t = 0
            if (!config_.use_geomagic) {
                // Reasignar x_init y R_init a la pose actual alcanzada para arrancar la trayectoria exactamente desde q_target
                pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
                pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
                const auto& frame_now = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
                cartesian_state_.position_initial = frame_now.translation();
                cartesian_state_.orientation_initial = Eigen::Quaterniond(frame_now.rotation());
                RCLCPP_INFO(this->get_logger(), "x_init y R_init reconfigurados a la pose actual antes de iniciar trayectoria.");
                trajectory_start_time_ = this->now();
                trajectory_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Trayectoria automática ACTIVADA desde t=0 tras alcanzar objetivo articular.");
            }
            return;
        }

        // Publicar el objetivo mientras no se alcance
        publish_initial_joint_position();
    }

    static bool ends_with(const std::string& str, const std::string& suffix) {
        if (suffix.size() > str.size()) return false;
        return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
    }

    std::vector<std::string> get_expected_joint_names() const {
        std::string prefix = config_.nmspace.empty() ? std::string("") : (config_.nmspace + std::string("_"));
        return {
            prefix + "shoulder_pan_joint",
            prefix + "shoulder_lift_joint",
            prefix + "elbow_joint",
            prefix + "wrist_1_joint",
            prefix + "wrist_2_joint",
            prefix + "wrist_3_joint"
        };
    }

    std::vector<std::string> get_expected_base_joint_names() const {
        return {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };
    }

    bool same_name_list(const std::vector<std::string>& a, const std::vector<std::string>& b) {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); ++i) if (a[i] != b[i]) return false;
        return true;
    }

    void rebuild_joint_index_map(const sensor_msgs::msg::JointState::SharedPtr& msg) {
        joint_map_initialized_ = false;
        joint_index_map_.assign(6, -1);
        last_js_names_ = msg->name;

        const auto expected = get_expected_joint_names();
        const auto expected_base = get_expected_base_joint_names();

        // Mapa rápido de nombre -> índice del mensaje
        std::unordered_map<std::string, int> name_to_idx;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            name_to_idx[msg->name[i]] = static_cast<int>(i);
        }

        // 1) Intento por coincidencia exacta
        int found_exact = 0;
        for (int i = 0; i < 6; ++i) {
            auto it = name_to_idx.find(expected[i]);
            if (it != name_to_idx.end()) {
                joint_index_map_[i] = it->second;
                ++found_exact;
            }
        }

        // 2) Fallback: buscar por nombre base exacto si faltan
        if (found_exact < 6) {
            for (int i = 0; i < 6; ++i) {
                if (joint_index_map_[i] != -1) continue;
                auto itb = name_to_idx.find(expected_base[i]);
                if (itb != name_to_idx.end()) {
                    joint_index_map_[i] = itb->second;
                    ++found_exact;
                }
            }
        }

        // 3) Último recurso: buscar entradas que terminen con el nombre base
        if (found_exact < 6) {
            for (int i = 0; i < 6; ++i) {
                if (joint_index_map_[i] != -1) continue;
                for (size_t j = 0; j < msg->name.size(); ++j) {
                    if (ends_with(msg->name[j], expected_base[i])) {
                        joint_index_map_[i] = static_cast<int>(j);
                        ++found_exact;
                        break;
                    }
                }
            }
        }

        if (found_exact == 6) {
            joint_map_initialized_ = true;
            std::ostringstream map_info;
            map_info << "Mapa de joints establecido (idx en /joint_states): ";
            for (int i = 0; i < 6; ++i) map_info << joint_index_map_[i] << (i < 5 ? "," : "");
            RCLCPP_INFO(this->get_logger(), "%s", map_info.str().c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No se pudo establecer el mapeo de joints por nombre. Recibidos %zu nombres.", msg->name.size());
        }
    }

    // Callback de JOINT STATES del UR5(e)
    void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = msg;
        // (Re)construir mapeo si es la primera vez o si la lista de nombres cambió
        if (!joint_map_initialized_ || !same_name_list(msg->name, last_js_names_)) {
            rebuild_joint_index_map(msg);
        }

        if (!joint_map_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Esperando mapeo válido de joints para reordenar JointState");
            return;
        }

        // Reordenar posiciones y velocidades según el mapeo
        for (int i = 0; i < 6; ++i) {
            int j = joint_index_map_[i];
            if (j < 0 || static_cast<size_t>(j) >= msg->position.size()) {
                RCLCPP_WARN(this->get_logger(), "Índice de joint fuera de rango para posiciones: %d", j);
                return;
            }
            robot_state_.q[i] = msg->position[j];
            if (static_cast<size_t>(j) < msg->velocity.size()) {
                robot_state_.qd[i] = msg->velocity[j];
            } else {
                robot_state_.qd[i] = 0.0;
            }
        }

        // Actualizar medición cartesiana actual (posición y orientación) del efector
        pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
        pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
        const auto& frame_placement_now = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
        cartesian_state_.position = frame_placement_now.translation();
        cartesian_state_.orientation = Eigen::Quaterniond(frame_placement_now.rotation());

        // Si estamos en modo ur5_pos y aún no hay baseline, capturarla cuando llegan los primeros joints
        if (config_.use_ur5_pos_init && !config_.use_geomagic && init_move_active_ && !init_baseline_set_) {
            for (int i = 0; i < 6; ++i) q_baseline_[i] = robot_state_.q[i];
            init_baseline_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Baseline articular capturada (update_joint_positions).");
        }

        // Capturar la pose inicial automáticamente en la primera recepción
        if (!pose_inicial_capturada_) {
            RCLCPP_INFO(this->get_logger(), "Capturando la pose inicial actual del robot...");
            
            robot_state_.q_init = robot_state_.q;
            
            pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q_init);
            pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);

            const auto& frame_placement = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
            cartesian_state_.position_initial = frame_placement.translation();
            cartesian_state_.orientation_initial = Eigen::Quaterniond(frame_placement.rotation());
            // Inicializar medición actual también
            cartesian_state_.position = cartesian_state_.position_initial;
            cartesian_state_.orientation = cartesian_state_.orientation_initial;

            pose_inicial_capturada_ = true;
            posicion_inicial_alcanzada_ = true;
            
            if (config_.use_geomagic) {
                capturar_pose_inicial_haptico_ = true;
            }
            
            RCLCPP_INFO(this->get_logger(), "Pose inicial capturada. El control está activo.");
            RCLCPP_INFO(this->get_logger(), "q_init: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                        robot_state_.q_init[0], robot_state_.q_init[1], robot_state_.q_init[2], robot_state_.q_init[3], robot_state_.q_init[4], robot_state_.q_init[5]);
            // Si geomagic es false y no hay movimiento inicial activo, arrancar trayectoria ahora
            if (!config_.use_geomagic && !init_move_active_) {
                trajectory_start_time_ = this->now();
                trajectory_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Trayectoria automática ACTIVADA desde t=0 tras captura de pose inicial (sin movimiento inicial)." );
            }
        }
    }
    
    // Callback del Geomagic Touch
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Mensaje nulo recibido en /phantom/pose.");
            return;
        }
        
        haptic_state_.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        haptic_state_.orientation.w() = msg->pose.orientation.w;
        haptic_state_.orientation.x() = msg->pose.orientation.x;
        haptic_state_.orientation.y() = msg->pose.orientation.y;
        haptic_state_.orientation.z() = msg->pose.orientation.z;



        
        //guardamos velocidad cartesiana del haptico por derivada de euler
        haptic_state_.velocity = (haptic_state_.position - haptic_state_.position_last) / (1e-3); //asumiendo 1ms entre callbacks
        haptic_state_.position_last = haptic_state_.position;
        Eigen::Quaterniond delta_orientation = haptic_state_.orientation * haptic_state_.orientation_last.inverse();
        // Aproximación de velocidad angular a partir de delta quaternion usando eje-ángulo
        haptic_state_.angular_velocity << delta_orientation.x()/(1e-3), delta_orientation.y()/(1e-3), delta_orientation.z()/(1e-3), delta_orientation.w()/(1e-3); //asumiendo 1ms entre callbacks
        haptic_state_.orientation_last = haptic_state_.orientation;
        haptic_state_.acceleration = (haptic_state_.velocity - haptic_state_.velocity_last) / (1e-3); //asumiendo 1ms entre callbacks
        haptic_state_.velocity_last = haptic_state_.velocity;
        



    }

    void button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg){
        if (msg->grey_button == 1 && capturar_pose_inicial_haptico_){ //
            RCLCPP_INFO(this->get_logger(), "Botón gris presionado: Capturando pose inicial del Geomagic.");
            capturar_pose_inicial_haptico_ = false; // el boton solo sirve para capturar la pose inicial una vez
            RCLCPP_INFO(this->get_logger(), "Pose inicial háptica capturada.");

            haptic_state_.position_initial = haptic_state_.position;
            haptic_state_.orientation_initial = haptic_state_.orientation;
            
        }
        if (msg->grey_button == 1) {
                RCLCPP_INFO(this->get_logger(), "Botón gris presionado");
            }
        if (msg->white_button == 1) {
            RCLCPP_INFO(this->get_logger(), "Botón blanco presionado");
        }
    } 

    
    void control_loop() {
        auto loop_t0 = std::chrono::steady_clock::now();
        if (!pose_inicial_capturada_ || !posicion_inicial_alcanzada_) {
            return;
        }
        if (config_.use_geomagic && capturar_pose_inicial_haptico_) {
            RCLCPP_WARN(this->get_logger(), "Esperando a que se capture la pose inicial del Geomagic. Presione el botón gris.");
            return;
        }
        // Rama geomagic: seguir referencia del háptico
        if (config_.use_geomagic) {
            Eigen::Quaterniond dif_orientacion_haptic = haptic_state_.orientation * haptic_state_.orientation_initial.inverse();
            Eigen::AngleAxisd angle_axis(dif_orientacion_haptic);
            double escala = 0.5; // factor de orientación
            dif_orientacion_haptic = Eigen::Quaterniond(Eigen::AngleAxisd(escala * angle_axis.angle(), angle_axis.axis()));
            cartesian_state_.orientation_desired = cartesian_state_.orientation_initial * dif_orientacion_haptic;
            RCLCPP_INFO(this->get_logger(), "Dif orientación háptico (eje): [%.3f, %.3f, %.3f]",
                        dif_orientacion_haptic.vec().x(), dif_orientacion_haptic.vec().y(), dif_orientacion_haptic.vec().z());
            cartesian_state_.position_desired = trayectoria_geomagic(
                cartesian_state_.position_initial, haptic_state_.position_initial, haptic_state_.position, 2.5);
            cartesian_state_.velocity = haptic_state_.velocity * 2.5; // escala de velocidad
            cartesian_state_.angular_velocity = haptic_state_.angular_velocity; // escala de velocidad
            cartesian_state_.acceleration = haptic_state_.acceleration * 2.5; // escala de aceleración
                
            
        } else {

            

            // Rama automática: generar trayectoria paramétrica
            if (!trajectory_active_) {
                // Aún no activada (esperando fin de movimiento inicial)
                return;
            }
            double t_elapsed = (this->now() - trajectory_start_time_).seconds();
            auto st = TrajectoryGenerator::calculate(
                cartesian_state_.position_initial,
                config_.traj_A,
                config_.traj_wn,
                config_.traj_c0,
                t_traj_,
                config_.traj_mode
            );
            
            cartesian_state_.position_desired = st.position;
            cartesian_state_.velocity = st.velocity;
            cartesian_state_.angular_velocity = Eigen::Vector4d::Zero(); // sin rot
            cartesian_state_.orientation_desired = cartesian_state_.orientation_initial;
            cartesian_state_.acceleration = st.acceleration;
            cartesian_state_.angular_acceleration = Eigen::Vector3d::Zero();

            //x_des << -0.03, 0.7, 0.1;
            // Mantener orientación constante
            cartesian_state_.orientation_desired = cartesian_state_.orientation_initial;
            std::cout<<"x_des: "<<cartesian_state_.position_desired.transpose()<<std::endl;
            std::cout<<"vel_des: "<<cartesian_state_.velocity.transpose()<<std::endl;
        }
        t_traj_ += 0.01;
        // Medición cartesiana actual (para logging y control si fuera necesario)
        pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, robot_state_.q);
        pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
        const auto& frame_meas = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
        cartesian_state_.position = frame_meas.translation();
        std::cout << cartesian_state_.position.transpose() << std::endl;
        Eigen::Matrix3d R_meas = frame_meas.rotation();
        cartesian_state_.orientation = Eigen::Quaterniond(R_meas);
        Eigen::Matrix3d R_des = cartesian_state_.orientation_desired.toRotationMatrix();
        Eigen::Matrix3d R_err = R_meas.transpose() * R_des;
        double e_R_angle = Eigen::AngleAxisd(R_err).angle();
        // Quaterniones y Euler (roll-pitch-yaw) deseados y medidos
        Eigen::Quaterniond q_des = cartesian_state_.orientation_desired;
        Eigen::Quaterniond q_meas(R_meas);
        Eigen::Vector3d euler_des = R_des.eulerAngles(0, 1, 2); // RPY
        Eigen::Vector3d euler_meas = R_meas.eulerAngles(0, 1, 2); // RPY

        auto ik_t0 = std::chrono::steady_clock::now();
        //std::cout<<"x_des: "<<cartesian_state_.position_desired.transpose()<<std::endl;
        //std::cout<<"vel_des: "<<haptic_state_.velocity.transpose()<<std::endl;

        if (config_.controller == "QP") {

            robot_state_.q_solution = kinematics_solver_->inverseKinematicsQP(
                robot_state_.q,
                cartesian_state_.position_desired,
                cartesian_state_.orientation_desired,
                600,
                0.1
            );}
        else if (config_.controller == "IMP") {
            Kp << 1850,1850,1850,500,500,500,500;
            Kd << 10,10,10,10,10,10,10;
            robot_state_.q_solution = impedance_controller_->calculateControlCommand(
                robot_state_.q,
                robot_state_.qd,
                cartesian_state_.position_desired,
                cartesian_state_.orientation_desired,
                cartesian_state_.velocity,
                cartesian_state_.acceleration,
                Kp,
                Kd,
                0.01);
            //std::cout<<"x_des: "<<cartesian_state_.position_desired.transpose()<<std::endl;

        } else if (config_.controller == "SLD") {
            return;
        } else {
            config_.controller = "QP";
            robot_state_.q_solution = kinematics_solver_->inverseKinematicsQP(
                robot_state_.q,
                cartesian_state_.position_desired,
                cartesian_state_.orientation_desired,
                600,
                0.1
            );}
        last_ik_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - ik_t0)
                           .count() / 1000.0;

        if ((robot_state_.q - robot_state_.q_solution).norm() < 0.001) {       // si la diferencia entre la posición inicial y la solución es muy pequeña, se usa la posición actual
            robot_state_.q_solution = robot_state_.q;
            config_.control_loop_time = std::max(0.01, config_.control_loop_time - 0.005); // Decrecer tiempo cuando está cerca del objetivo
        } else {
            // Calcular si nos acercamos o alejamos del objetivo
            static Eigen::VectorXd previous_error = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd current_error = robot_state_.q - robot_state_.q_solution;
            double current_error_norm = current_error.norm();
            double previous_error_norm = previous_error.norm();
            
            // Si el error actual es menor que el anterior, nos acercamos (signo negativo)
            // Si el error actual es mayor que el anterior, nos alejamos (signo positivo)
            double error_change = current_error_norm - previous_error_norm;
            
            // Ajustar el tiempo de control: incrementar si nos alejamos, decrecer si nos acercamos
            config_.control_loop_time += 0.001 * error_change;
            config_.control_loop_time = std::max(0.01, std::min(0.1, config_.control_loop_time)); // Limitar entre 0.01 y 1.0 segundos
            
            previous_error = current_error;
            std::cout << "Error norm: " << current_error_norm << ", Error change: " << error_change << ", Tiempo de control: " << config_.control_loop_time << std::endl;
        }
        RCLCPP_INFO(this->get_logger(), "q_actual: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
            robot_state_.q[0], robot_state_.q[1], robot_state_.q[2], robot_state_.q[3], robot_state_.q[4], robot_state_.q[5]);
        RCLCPP_INFO(this->get_logger(), "q_solution: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
            robot_state_.q_solution[0], robot_state_.q_solution[1], robot_state_.q_solution[2], robot_state_.q_solution[3], robot_state_.q_solution[4], robot_state_.q_solution[5]);    
        // --------------------------------------------------------------------------------
        
        std::string prefix = config_.nmspace.empty() ? "" : (config_.nmspace + "_");

        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.joint_names = {prefix + "shoulder_pan_joint", prefix + "shoulder_lift_joint", prefix + "elbow_joint",
                                        prefix + "wrist_1_joint", prefix + "wrist_2_joint", prefix + "wrist_3_joint"};
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        
        point.positions.assign(robot_state_.q_solution.data(), robot_state_.q_solution.data() + robot_state_.q_solution.size());

        point.time_from_start = rclcpp::Duration::from_seconds(config_.control_loop_time);
        trajectory_msg.points.push_back(point);
        joint_trajectory_pub_->publish(trajectory_msg);

        // Tiempo total del ciclo (hasta después de publicar)
        last_loop_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - loop_t0)
                    .count() / 1000.0;

        // Mostrar en terminal con throttling para no saturar
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        //             "ik_ms: %.3f | loop_ms: %.3f", last_ik_ms_, last_loop_ms_);
        std::cout << "ik_ms: " << last_ik_ms_ << " | loop_ms: " << last_loop_ms_ << std::endl;

        // Logging CSV
        if (config_.csv_enabled) {
            rclcpp::Time now_ros = this->now();
            double t = (now_ros - start_time_).seconds();
            double dt = (now_ros - last_log_time_).seconds();
            last_log_time_ = now_ros;
            // Calcular errores cartesianos
            Eigen::Vector3d pos_err = cartesian_state_.position_desired - cartesian_state_.position; // error de posición
            std::cout<<"pos_err: "<<pos_err.transpose()<<std::endl;
            // Para orientación: quaternion de error (desired * meas^{-1})
            Eigen::Quaterniond q_err = q_des * cartesian_state_.orientation.inverse();
            Eigen::AngleAxisd aa(q_err);
            Eigen::Vector3d ori_axis = aa.axis();
            double ori_angle = aa.angle();
            write_csv_row(t, dt, cartesian_state_.position_desired, cartesian_state_.position, q_des, cartesian_state_.orientation, euler_des, euler_meas, e_R_angle,
                          pos_err, ori_axis, ori_angle,
                          last_ik_ms_, last_loop_ms_);
        }

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR5IKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// ros2 run ur5_controller controller_backup --ros-args -p control_topic:="/scaled_joint_trajectory_controller/joint_trayectory" -p ur:="ur5e" -p nmspace:="ur5e" -p geomagic_topic:="/phantom2/pose" -p geomagic_button_topic:="/phantom2/button" -p csv_log_enable:="true" -p geomagic:="true"



