#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {
        // Inizializza il subscriber per lo stato delle giunzioni
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1)
        );

        // Inizializza il publisher per i comandi di posizione dei giunti
        joint_position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/command", 10
        );

        // Timer per inviare comandi di posizione periodici (opzionale)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&ArmControllerNode::publishPositionCommand, this)
        );
    }

private:
    // Callback per il subscriber del joint state
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Posizioni correnti delle giunzioni:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Giunzione: %s, Posizione: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }

    // Funzione per pubblicare comandi di posizione
    void publishPositionCommand() {
        auto command_msg = std_msgs::msg::Float64MultiArray();
        
        // Esempio di comando per impostare ogni giunto a una posizione specifica
        command_msg.data = {0.5, -0.5, 0.3, -0.3};  // Imposta le posizioni desiderate per i giunti

        RCLCPP_INFO(this->get_logger(), "Inviando comando di posizione ai giunti...");
        joint_position_publisher_->publish(command_msg);
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}

