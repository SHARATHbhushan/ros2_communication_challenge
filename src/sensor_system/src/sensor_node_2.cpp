#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_system/msg/sensor_signal.hpp"

class SensorNode2 : public rclcpp::Node {
public:
    SensorNode2() : Node("sensor_node_2") {
        publisher_ = this->create_publisher<sensor_system::msg::SensorSignal>("sensor_data_2", 10);
        
        // 500Hz = 2ms = 2000 microseconds
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(2000), 
            std::bind(&SensorNode2::timer_callback, this));
            
        start_time_ = this->now().seconds();
        RCLCPP_INFO(this->get_logger(), "Sensor 2 (500Hz) initialized.");
    }

private:
    void timer_callback() {
        auto msg = sensor_system::msg::SensorSignal();
        msg.sensor_id = 2;
        
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
    rclcpp::spin(std::make_shared<SensorNode2>());
    rclcpp::shutdown();
    return 0;
}