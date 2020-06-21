#include <config_client/config_client_node.h>
#include <ros2_temp/message_queue.h>

#include <Geometry2d/TransformMatrix.hpp>
#include <rj_msgs/msg/detection_frame.hpp>

namespace ros2_temp {
using DetectionFrameMsg = rj_msgs::msg::DetectionFrame;

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
     * @brief Calls spin_some on queue_.
     */
    void run();

    /**
     * @brief Returns whether this node is ok, ie. config_client_ is connected.
     * @return
     */
    bool ok() const { return config_client_->connected(); }

    /**
     * @brief Returns the team angle (It's horrible, but it's only temporary
     * until VisionFilter gets refactored).
     * @return
     */
    double TeamAngle() const {
        const bool defend_plus_x = config_client_->gameSettings().defend_plus_x;
        return defend_plus_x ? -M_PI_2 : M_PI_2;
    }

    Geometry2d::TransformMatrix WorldToTeam() const {
        Geometry2d::TransformMatrix world_to_team =
            Geometry2d::TransformMatrix::translate(
                0, config_client_->fieldDimensions().length / 2.0f);
        world_to_team *= Geometry2d::TransformMatrix::rotate(TeamAngle());
        return world_to_team;
    }

private:
    std::shared_ptr<MessageQueueNode<DetectionFrameMsg>> queue_;
    std::shared_ptr<config_client::ConfigClientNode> config_client_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};
}  // namespace ros2_temp
