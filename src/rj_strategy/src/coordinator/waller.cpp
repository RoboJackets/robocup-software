#include "rj_strategy/coordinator/waller.hpp"

namespace strategy {

Waller::Waller()
    : Coordinator("waller_srv", "waller_data", "waller_node") {
    walling_robots_.fill(-1);
    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
            update_wallers();
        });
}

void Waller::service_callback(RequestPtr request, ResponsePtr response) {
    if (request->joining) {
        if (num_wallers_ == kMaxWallers) {       
            response->success = false;
        } else {
            if (std::find(walling_robots_.begin(), walling_robots_.end(), request->robot_id) == walling_robots_.end()) {
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
            *it = -1;
            response->success = true;
            update_wallers();
        } else {
            response->success = false;
        }
    }
}

void Waller::update_wallers() {
    auto prev_wallers = walling_robots_;
    std::sort(walling_robots_.begin(), walling_robots_.end());
    auto must_republish = false;
    for (int i = 0; i < num_wallers_ && !must_republish; i++) {
        if (prev_wallers[i] != walling_robots_[i])  must_republish = true;
    }

    if (must_republish) {
        publisher_->publish(rj_msgs::msg::Waller().set__wall_list(walling_robots_).set__wall_size(num_wallers_));
    }
}

}  // namespace strategy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<strategy::Waller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}