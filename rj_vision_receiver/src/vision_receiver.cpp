#include <cmath>
#include <stdexcept>

#include <boost/exception/diagnostic_information.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/multicast.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_utils/conversions.hpp>
#include <rj_utils/logging_macros.hpp>
#include <rj_vision_receiver/vision_receiver.hpp>

constexpr auto kVisionReceiverParamModule = "vision_receiver";

DEFINE_INT64(kVisionReceiverParamModule, port, kSimVisionPort,
             "The port used for the vision receiver.")
DEFINE_STRING(kVisionReceiverParamModule, vision_interface, "", "The hardware interface to use.")

namespace vision_receiver {
using boost::asio::ip::udp;

VisionReceiver::VisionReceiver()
    : Node{"vision_receiver", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)},
      config_{this},
      port_{-1},
      socket_{io_context_},
      param_provider_(this, kVisionReceiverParamModule) {
    recv_buffer_.resize(65536);

    set_port(PARAM_vision_interface, PARAM_port);

    raw_packet_pub_ = create_publisher<RawProtobufMsg>(topics::kRawProtobufPub, 10);
    detection_frame_pub_ = create_publisher<DetectionFrameMsg>(topics::kDetectionFramePub, 10);

    // Spin off threads for the networking part and the publishing part.
    network_thread_ = std::thread{&VisionReceiver::receive_thread, this};
    publish_thread_ = std::thread{&VisionReceiver::publish_thread, this};

    rclcpp::on_shutdown([this]() {
        network_thread_.join();
        publish_thread_.join();
    });
}

void VisionReceiver::receive_thread() {
    while (rclcpp::ok()) {
        io_context_.run_for(kTimeout);
    }
}

void VisionReceiver::publish_thread() {
    // Block wait until either ConfigClient is connected or rclcpp::ok returns
    // false.
    if (!config_.wait_until_connected()) {
        return;
    }

    while (rclcpp::ok()) {
        // Blocking call to get one packet and process it.
        process_one_packet();
    }
}

void VisionReceiver::set_port(const std::string& interface, int port) {
    // If the socket is already open, close it to cancel any pending
    // operations before we reopen it on a new port.
    if (socket_.is_open()) {
        socket_.close();
    }

    port_ = port;

    // Open the socket and allow address reuse. We need this to allow
    // multiple instances of soccer to listen on the same multicast port.
    socket_.open(udp::v4());
    socket_.set_option(udp::socket::reuse_address(true));

    socket_.set_option(udp::socket::reuse_address(true));
    if (!interface.empty()) {
        socket_.set_option(boost::asio::ip::multicast::join_group(
            boost::asio::ip::address::from_string(kSharedVisionAddress).to_v4(),
            boost::asio::ip::address::from_string(interface).to_v4()));
    } else {
        socket_.set_option(boost::asio::ip::multicast::join_group(
            boost::asio::ip::address::from_string(kSharedVisionAddress).to_v4()));
    }

    // Bind the socket.
    boost::system::error_code bind_error;
    socket_.bind(udp::endpoint(udp::v4(), port), bind_error);
    if (static_cast<bool>(bind_error)) {
        EZ_ERROR_STREAM("Vision port bind failed with error: " << bind_error.message());
        return;
    }

    // Start receiving. Note that this only listens once; any later receive
    // will call `start_receive` again to continue listening.
    start_receive();
}

void VisionReceiver::process_one_packet() {
    // Get a packet, with blocking.
    StampedSSLWrapperPacket::UniquePtr packet;
    if (!packets_.try_get(packet, kTimeout)) {
        return;
    }

    // Publish raw packet
    RawProtobufMsg::UniquePtr raw_protobuf_msg = to_ros_msg(packet->wrapper);
    raw_packet_pub_->publish(std::move(raw_protobuf_msg));

    // If packet has geometry data, attempt to read information and
    // update if changed.
    if (packet->wrapper.has_geometry()) {
        update_geometry_packet(packet->wrapper.geometry().field());
    }

    // If the packet has detection data, publish it.
    if (packet->wrapper.has_detection()) {
        SSL_DetectionFrame* det = packet->wrapper.mutable_detection();

        DetectionFrameMsg::UniquePtr detection_frame_msg =
            std::make_unique<DetectionFrameMsg>(construct_ros_msg(*det, packet->receive_time));
        sync_detection_timestamp(detection_frame_msg.get(), packet->receive_time);

        detection_frame_pub_->publish(std::move(detection_frame_msg));
    }
}

DetectionFrameMsg VisionReceiver::construct_ros_msg(const SSL_DetectionFrame& frame,
                                                    const rclcpp::Time& received_time) const {
    DetectionFrameMsg msg{};

    msg.frame_number = frame.frame_number();
    msg.t_capture = to_ros_time(frame.t_capture());
    msg.t_sent = to_ros_time(frame.t_sent());
    msg.t_received = received_time;
    msg.camera_id = frame.camera_id();

    const bool defend_plus_x = config_.game_settings().defend_plus_x;

    // Only add balls that are in a used half
    const google::protobuf::RepeatedPtrField<SSL_DetectionBall>& balls = frame.balls();
    msg.balls.reserve(balls.size());
    for (int i = 0; i < balls.size(); ++i) {
        const SSL_DetectionBall& ball = balls.Get(i);
        if (in_used_half(defend_plus_x, ball.x())) {
            msg.balls.emplace_back(to_ros_msg(ball));
        }
    }

    // Add blue robots that are in a used half
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots_blue = frame.robots_blue();
    msg.robots_blue.reserve(robots_blue.size());
    for (int i = 0; i < robots_blue.size(); ++i) {
        const SSL_DetectionRobot& robot = robots_blue.Get(i);
        if (in_used_half(defend_plus_x, robot.x())) {
            msg.robots_blue.emplace_back(to_ros_msg(robot));
        }
    }

    // Add yellow robots that are in a used half
    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>& robots_yellow =
        frame.robots_yellow();
    msg.robots_yellow.reserve(robots_yellow.size());
    for (int i = 0; i < robots_yellow.size(); ++i) {
        const SSL_DetectionRobot& robot = robots_yellow.Get(i);
        if (in_used_half(defend_plus_x, robot.x())) {
            msg.robots_yellow.emplace_back(to_ros_msg(robot));
        }
    }

    return msg;
}

rclcpp::Time VisionReceiver::to_ros_time(double time_since_epoch_s) {
    const auto seconds = static_cast<int32_t>(time_since_epoch_s);
    const auto nanoseconds = static_cast<uint32_t>((time_since_epoch_s - seconds) * 1e9);
    if (time_since_epoch_s < 0) {
        return rclcpp::Time{0, 0, RCL_ROS_TIME};
    }
    return rclcpp::Time{seconds, nanoseconds, RCL_ROS_TIME};
}

void VisionReceiver::sync_detection_timestamp(DetectionFrameMsg* frame,
                                              const rclcpp::Time& receive_time) {
    // Assume that the frame was sent when we received it, so offset the capture
    // time by the difference (ie. estimated drift in clock between this
    // computer and the vision computer).
    frame->t_capture = receive_time - frame->t_sent + frame->t_capture;
    frame->t_sent = receive_time;
}

void VisionReceiver::start_receive() {
    // Set a receive callback
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), sender_endpoint_,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            // Handle the packet and then restart reception to allow new
            // incoming data
            receive_packet(error, num_bytes);
            start_receive();
        });
}

void VisionReceiver::receive_packet(const boost::system::error_code& error, std::size_t num_bytes) {
    // Check for error
    if (static_cast<bool>(error)) {
        EZ_ERROR_STREAM("Vision receive failed with error: " << error.message());
        return;
    }

    // Parse the protobuf message and tack on the receive time.
    StampedSSLWrapperPacket::UniquePtr stamped_packet = std::make_unique<StampedSSLWrapperPacket>();
    stamped_packet->receive_time = get_clock()->now();
    if (!stamped_packet->wrapper.ParseFromArray(&recv_buffer_[0], num_bytes)) {
        EZ_ERROR_STREAM("Got bad packet of " << num_bytes << " bytes from "
                                             << sender_endpoint_.address() << ":"
                                             << sender_endpoint_.port());
        return;
    }

    // Put the message into the list
    packets_.push(std::move(stamped_packet));
}

bool VisionReceiver::in_used_half(bool defend_plus_x, double x) const {
    const bool use_their_half = config_.game_settings().use_their_half;
    const bool use_our_half = config_.game_settings().use_our_half;

    const bool in_our_half = (defend_plus_x && x > 0) || (!defend_plus_x && x < 0);
    const bool in_their_half = !in_our_half;

    return (use_their_half && in_their_half) || (use_our_half && in_our_half);
}

/*
 * Updates the geometry packet in `Context` based on data from the vision packet
 */
void VisionReceiver::update_geometry_packet(const SSL_GeometryFieldSize& field_size) {
    if (field_size.field_lines_size() == 0) {
        return;
    }

    const SSL_FieldCircularArc* center = nullptr;
    float penalty_short_dist = 0;                                          // default value
    float penalty_long_dist = 0;                                           // default value
    float displacement = FieldDimensions::kDefaultDimensions.goal_flat();  // default displacment

    // Loop through field arcs looking for needed fields
    for (const SSL_FieldCircularArc& arc : field_size.field_arcs()) {
        if (arc.name() == "CenterCircle") {
            // Assume center circle
            center = &arc;
        }
    }

    for (const SSL_FieldLineSegment& line : field_size.field_lines()) {
        if (line.name() == "RightPenaltyStretch") {
            displacement = abs(line.p2().y() - line.p1().y());
            penalty_long_dist = displacement;
        } else if (line.name() == "RightFieldRightPenaltyStretch") {
            penalty_short_dist = abs(line.p2().x() - line.p1().x());
        }
    }

    float thickness = field_size.field_lines().Get(0).thickness() / 1000.0f;

    // The values we get are the center of the lines, we want to use the
    // outside, so we can add this as an offset.
    float adj = field_size.field_lines().Get(0).thickness() / 1000.0f / 2.0f;

    float field_border = config_.field_dimensions().border;

    if (penalty_long_dist != 0 && penalty_short_dist != 0 && center != nullptr && thickness != 0) {
        // Force a resize
        const FieldDimensions new_field_dim{
            field_size.field_length() / 1000.0f,
            field_size.field_width() / 1000.0f,
            field_border,
            thickness,
            field_size.goal_width() / 1000.0f,
            field_size.goal_depth() / 1000.0f,
            FieldDimensions::kDefaultDimensions.goal_height(),
            penalty_short_dist / 1000.0f,
            penalty_long_dist / 1000.0f,
            center->radius() / 1000.0f + adj,
            (center->radius()) * 2 / 1000.0f + adj,
            displacement / 1000.0f,
            (field_size.field_length() / 1000.0f + (field_border)*2),
            (field_size.field_width() / 1000.0f + (field_border)*2)};
        config_.update_field_dimensions(rj_convert::convert_to_ros<FieldDimensions>(new_field_dim));
    } else if (center != nullptr && thickness != 0) {
        const FieldDimensions default_dim = FieldDimensions::kDefaultDimensions;

        const FieldDimensions new_field_dim{
            field_size.field_length() / 1000.0f,
            field_size.field_width() / 1000.0f,
            field_border,
            thickness,
            field_size.goal_width() / 1000.0f,
            field_size.goal_depth() / 1000.0f,
            FieldDimensions::kDefaultDimensions.goal_height(),
            default_dim.penalty_short_dist(),
            default_dim.penalty_long_dist(),
            center->radius() / 1000.0f + adj,
            (center->radius()) * 2 / 1000.0f + adj,
            displacement / 1000.0f,
            (field_size.field_length() / 1000.0f + (field_border)*2),
            (field_size.field_width() / 1000.0f + (field_border)*2)};

        config_.update_field_dimensions(rj_convert::convert_to_ros(new_field_dim));
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
