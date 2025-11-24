#include "rj_strategy/coordinator/waller.hpp"

namespace strategy {

Waller::Waller() : Coordinator("waller_srv", "waller_data", "waller_node") {
    walling_robots_.fill(std::numeric_limits<uint8_t>::max());
}

void Waller::service_callback(RequestPtr request, ResponsePtr response) {
    if (request->joining) {
        if (num_wallers_ == kMaxWallers) {
            response->success = false;
        } else {
            if (std::find(walling_robots_.begin(), walling_robots_.end(), request->robot_id) ==
                walling_robots_.end()) {
                walling_robots_[num_wallers_] = request->robot_id;
                num_wallers_++;
                update_wallers();
            }
            response->success = true;
        }
    } else {
        auto it = std::find(walling_robots_.begin(), walling_robots_.end(), request->robot_id);
        if (it != walling_robots_.end()) {
            num_wallers_--;
            *it = std::numeric_limits<uint8_t>::max();
            update_wallers();
        }
        response->success = true;
    }
}

void Waller::update_wallers() {
    auto prev_wallers = walling_robots_;
    std::sort(walling_robots_.begin(), walling_robots_.end());
    publisher_->publish(
        rj_msgs::msg::Waller().set__wall_list(walling_robots_).set__wall_size(num_wallers_));
}

}  // namespace strategy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<strategy::Waller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}