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

    worker_ = std::thread(&DetectionFrameSub::spin, this);
}

std::vector<DetectionFrameMsg::UniquePtr> DetectionFrameSub::GetFrames() {
    std::vector<DetectionFrameMsg::UniquePtr> msgs;
    if (!queue_->GetAllThreaded(msgs)) {
        std::cout << "Got no messages!" << std::endl;
    }
    return msgs;
}

void DetectionFrameSub::spin() {
    executor_.spin();
};
}  // namespace ros2_temp
