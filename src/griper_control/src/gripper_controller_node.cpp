#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <memory>
#include "griper_control/msg/gripper_command.hpp"
#include <modbus/modbus.h>
#include <unistd.h>
#include <iomanip>
#include <cstdlib>  

#define PI 3.14159265358979323846
#define MAX_JOINT_DELTA 0.1
// Parámetros del gripper
#define CMD_REG_START 0x03E8
#define STATUS_REG    0x07D0
#define SLAVE_ID      0x09

class GripperController : public rclcpp::Node
{
public:
    GripperController() : Node("gripper_controller_node")
    {
        // Declarar parámetros
        this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("slave_id", SLAVE_ID);
        
        // Obtener parámetros
        std::string device = this->get_parameter("device").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        int slave_id = this->get_parameter("slave_id").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Iniciando gripper controller en dispositivo: %s", device.c_str());
        
        // Configurar permisos del dispositivo serie
        std::string chmod_cmd = "sudo chmod 666 " + device;
        std::system(chmod_cmd.c_str());
        
        // Inicializar comunicación Modbus
        ctx_ = modbus_new_rtu(device.c_str(), baudrate, 'N', 8, 1);
        if (!ctx_ || modbus_set_slave(ctx_, slave_id) == -1 || modbus_connect(ctx_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error al configurar comunicación Modbus");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Comunicación Modbus establecida correctamente");
        
        // Activar el gripper al inicio
        if (activateGripper()) {
            RCLCPP_INFO(this->get_logger(), "Gripper activado correctamente");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error al activar el gripper");
        }
        
        // Crear suscriptor para comandos del gripper
        command_subscription_ = this->create_subscription<griper_control::msg::GripperCommand>(
            "/gripper/command",
            10,
            std::bind(&GripperController::commandCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Gripper controller listo. Escuchando en /gripper/command");
    }
    
    ~GripperController()
    {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
    }

private:
    // Muestra bytes en hexadecimal
    void printHex(const uint8_t* data, int len, const std::string& label) {
        std::cout << label;
        for (int i = 0; i < len; ++i)
            std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
        std::cout << std::dec << std::endl;
    }

    // Calcula CRC16-Modbus
    uint16_t crc16_modbus(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t pos = 0; pos < length; pos++) {
            crc ^= data[pos];
            for (int i = 0; i < 8; i++) {
                if ((crc & 0x0001)) {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else crc >>= 1;
            }
        }
        return crc;
    }

    // Enviar comando raw con CRC automático
    bool sendRawWithCRC(uint8_t* request, int len_wo_crc) {
        uint16_t crc = crc16_modbus(request, len_wo_crc);
        request[len_wo_crc] = crc & 0xFF;
        request[len_wo_crc + 1] = (crc >> 8) & 0xFF;

        printHex(request, len_wo_crc + 2, "Request: ");
        uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];

        if (modbus_send_raw_request(ctx_, request, len_wo_crc + 2) == -1 ||
            modbus_receive_confirmation(ctx_, rsp) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error en comunicación Modbus");
            return false;
        }
        printHex(rsp, rsp[2] + 5, "Respuesta: ");
        return true;
    }

    // Activa el gripper si no está activado aún
    bool activateGripper() {
        uint16_t status;
        if (modbus_read_registers(ctx_, STATUS_REG, 1, &status) != 1) {
            RCLCPP_ERROR(this->get_logger(), "Error al leer estado del gripper");
            return false;
        }

        if ((status & 0x3100) == 0x3100) {
            RCLCPP_INFO(this->get_logger(), "Gripper ya está activado");
            return true;  // Ya activado
        }

        RCLCPP_INFO(this->get_logger(), "Activando gripper...");

        // RESET
        uint8_t reset[] = { SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendRawWithCRC(reset, sizeof(reset) - 2);
        usleep(500000);

        // ACTIVAR
        uint8_t activate[] = { SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                               0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendRawWithCRC(activate, sizeof(activate) - 2);

        // Esperar hasta activación
        for (int i = 0; i < 20; ++i) {
            usleep(200000);
            if (modbus_read_registers(ctx_, STATUS_REG, 1, &status) == 1 &&
                (status & 0x3100) == 0x3100) {
                RCLCPP_INFO(this->get_logger(), "Gripper activado exitosamente");
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Timeout activando gripper");
        return false;
    }

    // Controla el gripper (posición: 0-255, fuerza: 0-255)
    void moveGripper(uint8_t position, uint8_t force) {
        if (!activateGripper()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo activar el gripper");
            return;
        }

        uint8_t cmd[] = {
            SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
            0x09, 0x00, 0x00,       // rACT=1, rGTO=0, rATR=0
            position,               // rPR
            0xFF,                   // rSP (velocidad máxima)
            force,                  // rFR
            0x00, 0x00              // CRC
        };
        
        if (sendRawWithCRC(cmd, sizeof(cmd) - 2)) {
            RCLCPP_INFO(this->get_logger(), "Comando enviado: posición=%d, fuerza=%d", position, force);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error enviando comando al gripper");
        }
    }
    
    void commandCallback(const griper_control::msg::GripperCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Comando recibido: posición=%d, fuerza=%d", 
                    msg->position, msg->force);
        
        // Validar rangos
        if (msg->position > 255 || msg->force > 255) {
            RCLCPP_WARN(this->get_logger(), "Valores fuera de rango (0-255). Posición: %d, Fuerza: %d", 
                        msg->position, msg->force);
            return;
        }
        
        moveGripper(msg->position, msg->force);
    }

    rclcpp::Subscription<griper_control::msg::GripperCommand>::SharedPtr command_subscription_;
    modbus_t* ctx_ = nullptr;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperController>());
    rclcpp::shutdown();
    return 0;
}