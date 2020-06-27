#include <rj_utils/conversions.hpp>

RawProtobufMsg::UniquePtr ToROSMsg(const SSL_WrapperPacket& packet) {
    RawProtobufMsg::UniquePtr msg = std::make_unique<RawProtobufMsg>();
    const auto packet_size = packet.ByteSizeLong();
    msg->data.resize(packet_size);

    packet.SerializeWithCachedSizesToArray(msg->data.data());

    return msg;
}

DetectionBallMsg ToROSMsg(const SSL_DetectionBall& ball) {
    return rj_msgs::build<DetectionBallMsg>()
        .confidence(ball.confidence())
        .area(ball.area())
        .x(ball.x())
        .y(ball.y())
        .z(ball.z())
        .pixel_x(ball.pixel_x())
        .pixel_y(ball.pixel_y());
}

DetectionRobotMsg ToROSMsg(const SSL_DetectionRobot& robot) {
    return rj_msgs::build<DetectionRobotMsg>()
        .confidence(robot.confidence())
        .robot_id(robot.robot_id())
        .x(robot.x())
        .y(robot.y())
        .orientation(robot.orientation())
        .pixel_x(robot.pixel_x())
        .pixel_y(robot.pixel_y())
        .height(robot.height());
}
