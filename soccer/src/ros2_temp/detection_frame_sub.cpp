#include <ros2_temp/detection_frame_sub.h>

#include <rj_constants/topic_names.hpp>

namespace ros2_temp {
DetectionFrameSub::DetectionFrameSub() {
    queue_ = std::make_shared<MessageQueueNode<DetectionFrameMsg>>(
        "DetectionFrameSub_queue", vision_receiver::topics::kDetectionFramePub);
    config_client_ = std::make_shared<config_client::ConfigClientNode>(
        "DetectionFrameSub_config_client");

    executor_.add_node(queue_);
    executor_.add_node(config_client_);
}

std::vector<DetectionFrameMsg::UniquePtr> DetectionFrameSub::GetFrames() {
    std::vector<DetectionFrameMsg::UniquePtr> msgs;
    if (!queue_->GetAll(msgs)) {
        std::cout << "Got no messages!" << std::endl;
    }
    return msgs;
}

void DetectionFrameSub::run() { executor_.spin_some(); }
}  // namespace ros2_temp
