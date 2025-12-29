#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class SineWavePublisher : public rclcpp::Node {
public:
  SineWavePublisher() : Node("sine_wave_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sine_topic", 10);

    // 500 Hz = 1/500 seconds = 0.002 seconds = 2ms
    timer_ = this->create_wall_timer(2ms, std::bind(&SineWavePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::Float64();
    
    double time = this->get_clock()->now().seconds();
    
    // Frequency of the wave (not the publishing rate)
    // Let's set a 5Hz wave so you can see cycles clearly at 500Hz sampling
    double frequency = 5.0; 
    message.data = std::sin(2.0 * M_PI * frequency * time); 

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineWavePublisher>());
  rclcpp::shutdown();
  return 0;
}