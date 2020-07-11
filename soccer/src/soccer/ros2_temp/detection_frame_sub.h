#include <config_client/config_client_node.h>
#include <ros2_temp/message_queue.h>

#include <Geometry2d/TransformMatrix.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <thread>

namespace ros2_temp {
using DetectionFrameMsg = rj_msgs::msg::DetectionFrame;

/**
 * @brief A temporary class (until VisionFilter becomes a Node) that obtains
 * messages from vision_receiver by spinning off a thread to handle callbacks.
 */
class DetectionFrameSub {
public:
    DetectionFrameSub();

    /**
     * @brief Returns a vector of the received DetectionFrameMsgs, emptying
     * queue_.
     * @return
     */
    std::vector<DetectionFrameMsg::UniquePtr> GetFrames();

    /**
     * @brief Returns whether this node is ok, ie. config_client_ is connected.
     * @return
     */
    [[nodiscard]] bool ok() const { return config_client_->connected(); }

    /**
     * @brief Returns the team angle (It's horrible, but it's only temporary
     * until VisionFilter gets refactored).
     * @return
     */
    [[nodiscard]] double TeamAngle() const {
        const bool defend_plus_x = config_client_->gameSettings().defend_plus_x;
        return defend_plus_x ? -M_PI_2 : M_PI_2;
    }

    [[nodiscard]] Geometry2d::TransformMatrix WorldToTeam() const {
        Geometry2d::TransformMatrix world_to_team =
            Geometry2d::TransformMatrix::translate(
                0, config_client_->fieldDimensions().length / 2.0f);
        world_to_team *= Geometry2d::TransformMatrix::rotate(
            static_cast<float>(TeamAngle()));
        return world_to_team;
    }

private:
    std::shared_ptr<MessageQueueNode<DetectionFrameMsg>> queue_;
    std::shared_ptr<config_client::ConfigClientNode> config_client_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;

    void spin();
};
}  // namespace ros2_temp
