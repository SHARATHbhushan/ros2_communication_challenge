#include <vector>
#include <fstream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "sensor_system/msg/sensor_signal.hpp"

struct Recorder {
    int id;
    bool recording = false;
    bool done = false;
    std::vector<std::vector<double>> log; // Time | Value | Latency
    rclcpp::TimerBase::SharedPtr timer;
};

class CentralNode : public rclcpp::Node {
public:
    CentralNode() : Node("central_node") {
        recorders_.resize(2);
        for(int i=0; i<2; ++i) recorders_[i].id = i+1;

        auto sub_callback = [this](const sensor_system::msg::SensorSignal::SharedPtr msg, int idx) {
            auto& r = recorders_[idx];
            if (r.done) return;

            if (!r.recording) {
                r.recording = true;
                RCLCPP_INFO(this->get_logger(), "Starting 10s capture for Sensor %d", r.id);
                r.timer = this->create_wall_timer(std::chrono::seconds(10), [this, idx]() {
                    save_data(idx);
                });
            }

            double receive_time = this->now().seconds();
            double send_time = rclcpp::Time(msg->sent_timestamp).seconds();
            double latency_ms = (receive_time - send_time) * 1000.0;

            r.log.push_back({send_time, msg->value, latency_ms});
        };

        sub1_ = this->create_subscription<sensor_system::msg::SensorSignal>(
            "sensor_data_1", 10, [sub_callback](const sensor_system::msg::SensorSignal::SharedPtr msg) {
                sub_callback(msg, 0);
            });
            
        sub2_ = this->create_subscription<sensor_system::msg::SensorSignal>(
            "sensor_data_2", 10, [sub_callback](const sensor_system::msg::SensorSignal::SharedPtr msg) {
                sub_callback(msg, 1);
            });
    }

private:
    void save_data(int idx) {
        auto& r = recorders_[idx];
        r.done = true;
        r.timer->cancel();

        std::ofstream f("sensor_" + std::to_string(r.id) + ".dat");
        for(auto& row : r.log) {
            f << std::fixed << std::setprecision(6) << row[0] << " " << row[1] << " " << row[2] << "\n";
        }
        f.close();
        RCLCPP_INFO(this->get_logger(), "sensor_%d.dat saved successfully.", r.id);

        if (recorders_[0].done && recorders_[1].done) {
            RCLCPP_INFO(this->get_logger(), "Benchmark complete. Shutting down.");
            rclcpp::shutdown();
        }
    }

    std::vector<Recorder> recorders_;
    rclcpp::Subscription<sensor_system::msg::SensorSignal>::SharedPtr sub1_, sub2_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    return 0;
}