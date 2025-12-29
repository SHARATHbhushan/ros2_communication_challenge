#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_system/msg/sensor_signal.hpp"

class SensorNode1 : public rclcpp::Node {
public:
    SensorNode1() : Node("sensor_node_1") {
        publisher_ = this->create_publisher<sensor_system::msg::SensorSignal>("sensor_data_1", 10);
        
        // 500Hz = 2ms = 2000 microseconds
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(2000), 
            std::bind(&SensorNode1::timer_callback, this));
            
        start_time_ = this->now().seconds();
        RCLCPP_INFO(this->get_logger(), "Sensor 1 (500Hz) initialized.");
    }

private:
    void timer_callback() {
        auto msg = sensor_system::msg::SensorSignal();
        msg.sensor_id = 1;
        double current_time = this->now().seconds();
        double elapsed = current_time - start_time_;
        msg.value = std::sin(2.0 * M_PI * 5.0 * elapsed);
        msg.sent_timestamp = this->now();
        publisher_->publish(msg);
    }
    rclcpp::Publisher<sensor_system::msg::SensorSignal>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double start_time_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode1>());
    rclcpp::shutdown();
    return 0;
}