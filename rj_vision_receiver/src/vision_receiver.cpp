#include <rj_vision_receiver/vision_receiver.h>

#include <boost/exception/diagnostic_information.hpp>
#include <rj_common/Field_Dimensions.hpp>
#include <rj_common/multicast.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_utils/logging.hpp>
#include <stdexcept>

#include "math.h"

namespace vision_receiver {
using boost::asio::ip::udp;

VisionReceiver::VisionReceiver()
    : Node{"vision_receiver"}, config_{this}, port_{-1}, _socket{_io_context} {
    _recv_buffer.resize(65536);

    // There should be at most four packets: one for each camera on each of two
    // frames, assuming some clock skew between this
    // computer and the vision computer.
    _packets.reserve(4);

    // Parameters
    declare_parameter<int>("port", SharedVisionPortSinglePrimary);
    get_parameter<int>("port", port_);

    double hz{};
    declare_parameter<double>("hz", 60.0);
    get_parameter<double>("hz", hz);

    setPort(port_);

    auto period = std::chrono::duration<double>(1 / hz);

    raw_packet_pub_ =
        create_publisher<RawProtobufMsg>(topics::kRawProtobufPub, 10);
    detection_frame_pub_ =
        create_publisher<DetectionFrameMsg>(topics::kDetectionFramePub, 10);

    // Create timer for callback
    timer_ = this->create_wall_timer(period, [this]() { run(); });
}

void VisionReceiver::setPort(int port) {
    // If the socket is already open, close it to cancel any pending
    // operations before we reopen it on a new port.
    if (_socket.is_open()) {
        _socket.close();
    }

    // Open the socket and allow address reuse. We need this to allow
    // multiple instances of soccer to listen on the same multicast port.
    _socket.open(udp::v4());
    _socket.set_option(udp::socket::reuse_address(true));

    // Set up multicast.
    if (!multicast_add_native(_socket.native_handle(),
                              SharedVisionAddress.c_str())) {
        EZ_ERROR("Multicast add failed");
        return;
    }

    // Bind the socket.
    boost::system::error_code bind_error;
    _socket.bind(udp::endpoint(udp::v4(), port), bind_error);
    if (static_cast<bool>(bind_error)) {
        EZ_ERROR_STREAM(
            "Vision port bind failed with error: " << bind_error.message());
        return;
    }

    // Start receiving. Note that this only listens once; any later receive
    // will call `startReceive` again to continue listening.
    startReceive();
}

void VisionReceiver::run() {
    // If we haven't received config from the config server yet, return
    if (!config_.connected()) {
        return;
    }
    static bool connected = false;
    if (!connected) {
        connected = true;
        EZ_INFO("Connected!");
    }

    // Let boost::asio check for new packets
    _io_context.poll();

    // Process and clear new packets.
    processNewPackets();
}

void VisionReceiver::processNewPackets() {
    for (StampedSSLWrapperPacket::UniquePtr& packet : _packets) {
        // Publish raw packet
        RawProtobufMsg::UniquePtr raw_protobuf_msg = ToROSMsg(packet->wrapper);
        raw_packet_pub_->publish(std::move(raw_protobuf_msg));

        // If packet has geometry data, attempt to read information and
        // update if changed.
        if (packet->wrapper.has_geometry()) {
            UpdateGeometryPacket(packet->wrapper.geometry().field());
        }

        // If the packet has detection data, publish it.
        if (packet->wrapper.has_detection()) {
            SSL_DetectionFrame* det = packet->wrapper.mutable_detection();

            DetectionFrameMsg::UniquePtr detection_frame_msg =
                std::make_unique<DetectionFrameMsg>(
                    ToROSMsg(*det, packet->receive_time));
            SyncDetectionTimestamp(detection_frame_msg.get(),
                                   packet->receive_time);

            detection_frame_pub_->publish(std::move(detection_frame_msg));
        }
    }
    _packets.clear();
}

[[nodiscard]] RawProtobufMsg::UniquePtr VisionReceiver::ToROSMsg(
    const SSL_WrapperPacket& packet) {
    RawProtobufMsg::UniquePtr msg = std::make_unique<RawProtobufMsg>();
    const auto packet_size = packet.ByteSizeLong();
    msg->data.resize(packet_size);

    packet.SerializeWithCachedSizesToArray(msg->data.data());

    return msg;
}

DetectionFrameMsg VisionReceiver::ToROSMsg(
    const SSL_DetectionFrame& frame, const rclcpp::Time& received_time) const {
    DetectionFrameMsg msg{};

    msg.t_sent = ToROSTime(frame.t_sent());
    msg.t_capture = ToROSTime(frame.t_capture());
    msg.t_received = received_time;

    const bool defend_plus_x = config_.gameSettings().defend_plus_x;

    // Only add balls that are in a used half
    const google::protobuf::RepeatedPtrField<SSL_DetectionBall>& balls =
        frame.balls();
    msg.balls.reserve(balls.size());
    for (int i = 0; i < balls.size(); ++i) {
        const SSL_DetectionBall& ball = balls.Get(i);
        if (InUsedHalf(defend_plus_x, ball.x())) {
            msg.balls.emplace_back(ToROSMsg(ball));
        }
    }

    // Add blue robots that are in a used half
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots_blue =
        frame.robots_blue();
    msg.robots_blue.reserve(robots_blue.size());
    for (int i = 0; i < robots_blue.size(); ++i) {
        const SSL_DetectionRobot& robot = robots_blue.Get(i);
        if (InUsedHalf(defend_plus_x, robot.x())) {
            msg.robots_blue.emplace_back(ToROSMsg(robot));
        }
    }

    // Add yellow robots that are in a used half
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>&
        robots_yellow = frame.robots_yellow();
    msg.robots_yellow.reserve(robots_yellow.size());
    for (int i = 0; i < robots_yellow.size(); ++i) {
        const SSL_DetectionRobot& robot = robots_yellow.Get(i);
        if (InUsedHalf(defend_plus_x, robot.x())) {
            msg.robots_yellow.emplace_back(ToROSMsg(robot));
        }
    }

    return msg;
}

rclcpp::Time VisionReceiver::ToROSTime(double time_since_epoch_s) {
    const auto seconds = static_cast<int32_t>(time_since_epoch_s);
    const auto nanoseconds =
        static_cast<uint32_t>((time_since_epoch_s - seconds) * 10e9);
    return rclcpp::Time{seconds, nanoseconds, RCL_ROS_TIME};
}

DetectionBallMsg VisionReceiver::ToROSMsg(const SSL_DetectionBall& ball) {
    DetectionBallMsg msg{};
    msg.confidence = ball.confidence();
    msg.area = ball.area();
    msg.x = ball.x();
    msg.y = ball.y();
    msg.z = ball.z();
    msg.pixel_x = ball.pixel_x();
    msg.pixel_y = ball.pixel_y();
    return msg;
}

DetectionRobotMsg VisionReceiver::ToROSMsg(const SSL_DetectionRobot& robot) {
    DetectionRobotMsg msg{};
    msg.confidence = robot.confidence();
    msg.robot_id = robot.robot_id();
    msg.x = robot.x();
    msg.y = robot.y();
    msg.orientation = robot.orientation();
    msg.pixel_x = robot.pixel_x();
    msg.pixel_y = robot.pixel_y();
    msg.height = robot.height();
    return msg;
}

void VisionReceiver::SyncDetectionTimestamp(DetectionFrameMsg* frame,
                                            const rclcpp::Time& receive_time) {
    // Assume that the frame was sent when we received it, so offset the capture
    // time by the difference (ie. estimated drift in clock between this
    // computer and the vision computer).
    frame->t_capture = receive_time - frame->t_sent + frame->t_capture;
    frame->t_sent = receive_time;
}

void VisionReceiver::startReceive() {
    // Set a receive callback
    _socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _sender_endpoint,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            // Handle the packet and then restart reception to allow new
            // incoming data
            receivePacket(error, num_bytes);
            startReceive();
        });
}

void VisionReceiver::receivePacket(const boost::system::error_code& error,
                                   std::size_t num_bytes) {
    // Check for error
    if (static_cast<bool>(error)) {
        EZ_ERROR_STREAM(
            "Vision receive failed with error: " << error.message());
        return;
    }

    // Parse the protobuf message and tack on the receive time.
    StampedSSLWrapperPacket::UniquePtr stamped_packet =
        std::make_unique<StampedSSLWrapperPacket>();
    stamped_packet->receive_time = get_clock()->now();
    if (!stamped_packet->wrapper.ParseFromArray(&_recv_buffer[0], num_bytes)) {
        EZ_ERROR_STREAM("Got bad packet of " << num_bytes << " bytes from "
                                             << _sender_endpoint.address()
                                             << ":" << _sender_endpoint.port());
        return;
    }

    // Put the message into the list
    _packets.emplace_back(std::move(stamped_packet));
}

bool VisionReceiver::InUsedHalf(bool defend_plus_x, double x) const {
    const bool use_their_half = config_.gameSettings().use_their_half;
    const bool use_our_half = config_.gameSettings().use_our_half;

    const bool in_our_half =
        (defend_plus_x && x > 0) || (!defend_plus_x && x < 0);
    const bool in_their_half = !in_our_half;

    return (use_their_half && in_their_half) || (use_our_half && in_our_half);
}

/*
 * Updates the geometry packet in `Context` based on data from the vision packet
 */
void VisionReceiver::UpdateGeometryPacket(
    const SSL_GeometryFieldSize& fieldSize) {
    if (fieldSize.field_lines_size() == 0) {
        return;
    }

    const SSL_FieldCicularArc* center = nullptr;
    float penaltyShortDist = 0;  // default value
    float penaltyLongDist = 0;   // default value
    float displacement =
        Field_Dimensions::Default_Dimensions.GoalFlat();  // default displacment

    // Loop through field arcs looking for needed fields
    for (const SSL_FieldCicularArc& arc : fieldSize.field_arcs()) {
        if (arc.name() == "CenterCircle") {
            // Assume center circle
            center = &arc;
        }
    }

    for (const SSL_FieldLineSegment& line : fieldSize.field_lines()) {
        if (line.name() == "RightPenaltyStretch") {
            displacement = abs(line.p2().y() - line.p1().y());
            penaltyLongDist = displacement;
        } else if (line.name() == "RightFieldRightPenaltyStretch") {
            penaltyShortDist = abs(line.p2().x() - line.p1().x());
        }
    }

    float thickness = fieldSize.field_lines().Get(0).thickness() / 1000.0f;

    // The values we get are the center of the lines, we want to use the
    // outside, so we can add this as an offset.
    float adj = fieldSize.field_lines().Get(0).thickness() / 1000.0f / 2.0f;

    float fieldBorder = config_.fieldDimensions().border;

    if (penaltyLongDist != 0 && penaltyShortDist != 0 && center != nullptr &&
        thickness != 0) {
        // Force a resize
        const Field_Dimensions new_field_dim{
            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f,
            fieldBorder,
            thickness,
            fieldSize.goal_width() / 1000.0f,
            fieldSize.goal_depth() / 1000.0f,
            Field_Dimensions::Default_Dimensions.GoalHeight(),
            penaltyShortDist / 1000.0f,              // PenaltyShortDist
            penaltyLongDist / 1000.0f,               // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2)};
        config_.updateFieldDimensions(new_field_dim);
    } else if (center != nullptr && thickness != 0) {
        const Field_Dimensions defaultDim =
            Field_Dimensions::Default_Dimensions;

        const Field_Dimensions new_field_dim{
            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f,
            fieldBorder,
            thickness,
            fieldSize.goal_width() / 1000.0f,
            fieldSize.goal_depth() / 1000.0f,
            Field_Dimensions::Default_Dimensions.GoalHeight(),
            defaultDim.PenaltyShortDist(),           // PenaltyShortDist
            defaultDim.PenaltyLongDist(),            // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2)};

        config_.updateFieldDimensions(new_field_dim);
    } else {
        EZ_ERROR_STREAM(
            "Error: failed to decode SSL geometry packet. Not resizing "
            "field.");
    }
}
}  // namespace vision_receiver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vision_receiver::VisionReceiver>());
    rclcpp::shutdown();
    return 0;
}
