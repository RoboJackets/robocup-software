#include <field_dimensions.h>
#include <network/multicast.h>

#include <boost/exception/diagnostic_information.hpp>
#include <stdexcept>

#include "vision_receiver_sub.hpp"

namespace vision_receiver {
using boost::asio::ip::udp;

VisionReceiver::VisionReceiver()
    : Node{"vision_receiver"}, config_{this}, port_{-1}, _socket{_io_context} {
    _recv_buffer.resize(65536);

    _last_receive_time = RJ::now();

    // There should be at most four packets: one for each camera on each of two
    // frames, assuming some clock skew between this
    // computer and the vision computer.
    _packets.reserve(4);

    // Parameters
    declare_parameter<int>("port", SharedVisionPortSinglePrimary);
    get_parameter<int>("port", port_);

    double hz;
    declare_parameter<double>("hz", 60.0);
    get_parameter<double>("hz", hz);

    setPort(port_);

    auto period = std::chrono::duration<double>(1 / hz);

    raw_packet_pub_ =
        create_publisher<RawProtobufMsg>("vision/raw_protobuf", 10);
    vision_packet_pub_ =
        create_publisher<VisionPacketMsg>("vision/vision_packet", 10);

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
        RCLCPP_ERROR(get_logger(), "Multicast add failed");
        return;
    }

    // Bind the socket.
    boost::system::error_code bind_error;
    _socket.bind(udp::endpoint(udp::v4(), port), bind_error);
    if (static_cast<bool>(bind_error)) {
        RCLCPP_ERROR_STREAM(get_logger(), "Vision port bind failed with error: "
                                              << bind_error.message());
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
        RCLCPP_INFO_STREAM(get_logger(), "Connected!");
    }

    // Let boost::asio check for new packets
    _io_context.poll();

    processNewPackets();

    for (auto&& packet : _packets) {
        publishVisionPacket(std::move(packet));
    }
    _packets.clear();
}

void VisionReceiver::processNewPackets() {
    for (auto& packet : _packets) {
        publishRawPacket(packet->wrapper);

        _last_receive_time = packet->receivedTime;

        // If packet has geometry data, attempt to read information and
        // update if changed.
        if (packet->wrapper.has_geometry()) {
            updateGeometryPacket(packet->wrapper.geometry().field());
        }

        const bool defend_plus_x = config_.gameSettings().defend_plus_x;

        if (packet->wrapper.has_detection()) {
            SSL_DetectionFrame* det = packet->wrapper.mutable_detection();

            double rt = RJ::numSeconds(packet->receivedTime.time_since_epoch());
            det->set_t_capture(rt - det->t_sent() + det->t_capture());
            det->set_t_sent(rt);

            // Remove balls on the excluded half of the field
            google::protobuf::RepeatedPtrField<SSL_DetectionBall>* balls =
                det->mutable_balls();
            for (int i = 0; i < balls->size(); ++i) {
                float x = balls->Get(i).x();
                // FIXME - OMG too many terms
                if (shouldRemove(defend_plus_x, x)) {
                    balls->SwapElements(i, balls->size() - 1);
                    balls->RemoveLast();
                    --i;
                }
            }

            // Remove robots on the excluded half of the field
            google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots[2] =
                {det->mutable_robots_yellow(), det->mutable_robots_blue()};

            for (auto& robot : robots) {
                for (int i = 0; i < robot->size(); ++i) {
                    float x = robot->Get(i).x();
                    if (shouldRemove(defend_plus_x, x)) {
                        robot->SwapElements(i, robot->size() - 1);
                        robot->RemoveLast();
                        --i;
                    }
                }
            }
        }
    }
}

void VisionReceiver::publishVisionPacket(std::unique_ptr<VisionPacket> packet) {
    vision_packet_pub_->publish(packet->toMsg());
}

void VisionReceiver::publishRawPacket(const SSL_WrapperPacket& packet) {
    RawProtobufMsg::UniquePtr msg = std::make_unique<RawProtobufMsg>();
    const auto packet_size = packet.ByteSizeLong();
    msg->data.resize(packet_size);

    packet.SerializeWithCachedSizesToArray(msg->data.data());

    raw_packet_pub_->publish(std::move(msg));
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
        std::cerr << "Vision receive failed with error: " << error.message()
                  << std::endl;
        return;
    }

    // Parse the protobuf message
    std::unique_ptr<VisionPacket> packet = std::make_unique<VisionPacket>();
    packet->receivedTime = RJ::now();
    if (!packet->wrapper.ParseFromArray(&_recv_buffer[0], num_bytes)) {
        std::cerr << "VisionReceiver: got bad packet of " << num_bytes
                  << " bytes from " << _sender_endpoint.address() << ":"
                  << _sender_endpoint.port() << std::endl;
        return;
    }

    // Put the message into the list
    _packets.push_back(std::move(packet));
}

bool VisionReceiver::shouldRemove(bool defendPlusX, double x) {
    const auto use_their_half = config_.gameSettings().use_their_half;
    const auto use_our_half = config_.gameSettings().use_our_half;

    return (!use_their_half &&
            ((defendPlusX && x < 0) || (!defendPlusX && x > 0))) ||
           (!use_our_half &&
            ((defendPlusX && x > 0) || (!defendPlusX && x < 0)));
}

/*
 * Updates the geometry packet in `Context` based on data from the vision packet
 */
void VisionReceiver::updateGeometryPacket(
    const SSL_GeometryFieldSize& fieldSize) {
    if (fieldSize.field_lines_size() == 0) {
        return;
    }

    const SSL_FieldCicularArc* center = nullptr;
    float penaltyShortDist = 0;  // default value
    float penaltyLongDist = 0;   // default value
    float displacement =
        FieldDimensions::Default_Dimensions.GoalFlat();  // default displacment

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
        const FieldDimensions new_field_dim{
            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f,
            fieldBorder,
            thickness,
            fieldSize.goal_width() / 1000.0f,
            fieldSize.goal_depth() / 1000.0f,
            FieldDimensions::Default_Dimensions.GoalHeight(),
            penaltyShortDist / 1000.0f,              // PenaltyShortDist
            penaltyLongDist / 1000.0f,               // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2)};

        config_.updateFieldDimensions(new_field_dim.toMsg());
    } else if (center != nullptr && thickness != 0) {
        const FieldDimensions defaultDim = FieldDimensions::Default_Dimensions;

        const FieldDimensions new_field_dim{
            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f,
            fieldBorder,
            thickness,
            fieldSize.goal_width() / 1000.0f,
            fieldSize.goal_depth() / 1000.0f,
            FieldDimensions::Default_Dimensions.GoalHeight(),
            defaultDim.PenaltyShortDist(),           // PenaltyShortDist
            defaultDim.PenaltyLongDist(),            // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2)};

        config_.updateFieldDimensions(new_field_dim.toMsg());

    } else {
        std::cerr
            << "Error: failed to decode SSL geometry packet. Not resizing "
               "field."
            << std::endl;
    }
}
}  // namespace vision_receiver
