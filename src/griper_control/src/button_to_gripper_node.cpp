#include <rclcpp/rclcpp.hpp>
#include "omni_msgs/msg/omni_button_event.hpp"
#include "griper_control/msg/gripper_command.hpp"

class ButtonToGripperNode : public rclcpp::Node
{
public:
    ButtonToGripperNode() : Node("button_to_gripper_node")
    {
        // Declarar parámetros para personalizar el comportamiento
        this->declare_parameter<int>("closed_position", 0);
        this->declare_parameter<int>("open_position", 250);
        this->declare_parameter<int>("close_force", 255);
        this->declare_parameter<int>("open_force", 150);
        this->declare_parameter<std::string>("button_topic", "/phantom/button");
        
        // Obtener parámetros
        closed_position_ = this->get_parameter("closed_position").as_int();
        open_position_ = this->get_parameter("open_position").as_int();
        close_force_ = this->get_parameter("close_force").as_int();
        open_force_ = this->get_parameter("open_force").as_int();
        std::string button_topic = this->get_parameter("button_topic").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Configuración del gripper:");
        RCLCPP_INFO(this->get_logger(), "  - Posición cerrado: %d", closed_position_);
        RCLCPP_INFO(this->get_logger(), "  - Posición abierto: %d", open_position_);
        RCLCPP_INFO(this->get_logger(), "  - Fuerza cerrado: %d", close_force_);
        RCLCPP_INFO(this->get_logger(), "  - Fuerza abierto: %d", open_force_);
        RCLCPP_INFO(this->get_logger(), "  - Escuchando en: %s", button_topic.c_str());
        
        // Crear suscriptor a los botones del Phantom
        button_subscription_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
            button_topic,
            10,
            std::bind(&ButtonToGripperNode::buttonCallback, this, std::placeholders::_1)
        );
        
        // Crear publicador para comandos del gripper
        gripper_command_publisher_ = this->create_publisher<griper_control::msg::GripperCommand>(
            "/gripper/command",
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "Nodo button_to_gripper listo");
        RCLCPP_INFO(this->get_logger(), "Presiona el botón gris para alternar entre abrir/cerrar el gripper");
    }

private:
    void buttonCallback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg)
    {
        // Solo reaccionar al botón gris cuando se presiona (flanco ascendente)
        if (msg->grey_button == 1) {
            gripper_closed_ = !gripper_closed_;
            
            auto command = griper_control::msg::GripperCommand();
            
            if (gripper_closed_) {
                command.position = closed_position_;
                command.force = close_force_;
                RCLCPP_INFO(this->get_logger(), "Cerrando gripper (pos: %d, fuerza: %d)", 
                            command.position, command.force);
            } else {
                command.position = open_position_;
                command.force = open_force_;
                RCLCPP_INFO(this->get_logger(), "Abriendo gripper (pos: %d, fuerza: %d)", 
                            command.position, command.force);
            }
            
            gripper_command_publisher_->publish(command);
        }
        
        // Opcional: El botón blanco podría tener otra función
        if (msg->white_button == 1 && !previous_white_button_state_) {
            RCLCPP_INFO(this->get_logger(), "Botón blanco presionado - sin acción asignada");
            // Aquí podrías agregar otra funcionalidad, como cambiar la fuerza o hacer un stop de emergencia
        }
        
        // Actualizar estado previo de los botones
        previous_grey_button_state_ = msg->grey_button;
        previous_white_button_state_ = msg->white_button;
    }

    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_subscription_;
    rclcpp::Publisher<griper_control::msg::GripperCommand>::SharedPtr gripper_command_publisher_;
    
    // Estado del gripper
    bool gripper_closed_ = false;
    
    // Estado previo de los botones para detectar flancos
    bool previous_grey_button_state_ = false;
    bool previous_white_button_state_ = false;
    
    // Parámetros configurables
    int closed_position_;
    int open_position_;
    int close_force_;
    int open_force_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonToGripperNode>());
    rclcpp::shutdown();
    return 0;
}