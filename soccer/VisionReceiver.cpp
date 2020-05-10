#include "VisionReceiver.hpp"

#include <unistd.h>
#include <Utils.hpp>
#include <multicast.hpp>
#include <stdexcept>

using namespace std;
using boost::asio::ip::udp;

VisionReceiver::VisionReceiver(Context* context, bool /*sim*/, int port)
    : port(port), _context(context), _socket(_io_context) {
    _recv_buffer.resize(65536);

    _last_receive_time = RJ::now();

    // There should be at most four packets: one for each camera on each of two
    // frames, assuming some clock skew between this
    // computer and the vision computer.
    _packets.reserve(4);

    setPort(port);
}

void VisionReceiver::setPort(int port) {
    // If the socket is already open, close it to cancel any pending operations
    // before we reopen it on a new port.
    if (_socket.is_open()) {
        _socket.close();
    }

    // Open the socket and allow address reuse. We need this to allow multiple
    // instances of soccer to listen on the same multicast port.
    _socket.open(udp::v4());
    _socket.set_option(udp::socket::reuse_address(true));

    // Set up multicast.
    if (!multicast_add_native(_socket.native_handle(),
                              SharedVisionAddress.c_str())) {
        std::cerr << "Multicast add failed" << std::endl;
        return;
    }

    // Bind the socket.
    boost::system::error_code bind_error;
    _socket.bind(udp::endpoint(udp::v4(), port), bind_error);
    if (bind_error != nullptr) {
        std::cerr << "Vision port bind failed with error: "
                  << bind_error.message() << std::endl;
        return;
    }

    // Start receiving. Note that this only listens once; any later receive
    // will call `startReceive` again to continue listening.
    startReceive();
}

void VisionReceiver::run() {
    // Let boost::asio check for new packets
    _io_context.poll();

    for (auto& packet : _packets) {
        SSL_WrapperPacket* log = _context->state.logFrame->add_raw_vision();
        log->CopyFrom(packet->wrapper);

        _last_receive_time = packet->receivedTime;

        // If packet has geometry data, attempt to read information and
        // update if changed.
        if (packet->wrapper.has_geometry()) {
            updateGeometryPacket(packet->wrapper.geometry().field());
        }

        bool defendPlusX = _context->game_state.defendPlusX;

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
                if (shouldRemove(defendPlusX, x)) {
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
                    if (shouldRemove(defendPlusX, x)) {
                        robot->SwapElements(i, robot->size() - 1);
                        robot->RemoveLast();
                        --i;
                    }
                }
            }
        }
    }

    // Move new packets into context using a move_iterator.
    _context->vision_packets.insert(_context->vision_packets.begin(),
                                    std::make_move_iterator(_packets.begin()),
                                    std::make_move_iterator(_packets.end()));

    // Remove the nullptr packets that we just moved to context.
    _packets.clear();
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
    if (error != nullptr) {
        std::cerr << "Vision receive failed with error: " << error.message()
                  << std::endl;
        return;
    }

    static int num = 0;

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
    return (!_context->state.logFrame->use_our_half() &&
            ((defendPlusX && x < 0) || (!defendPlusX && x > 0))) ||
           (!_context->state.logFrame->use_our_half() &&
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

    const SSL_FieldCicularArc* penalty = nullptr;
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

    float fieldBorder = _context->field_dimensions.Border();

    if (penaltyLongDist != 0 && penaltyShortDist != 0 && center != nullptr &&
        thickness != 0) {
        // Force a resize
        _context->field_dimensions = Field_Dimensions(

            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f, fieldBorder, thickness,
            fieldSize.goal_width() / 1000.0f, fieldSize.goal_depth() / 1000.0f,
            Field_Dimensions::Default_Dimensions.GoalHeight(),
            penaltyShortDist / 1000.0f,              // PenaltyShortDist
            penaltyLongDist / 1000.0f,               // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2));
    } else if (center != nullptr && thickness != 0) {
        Field_Dimensions defaultDim = Field_Dimensions::Default_Dimensions;

        _context->field_dimensions = Field_Dimensions(
            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f, fieldBorder, thickness,
            fieldSize.goal_width() / 1000.0f, fieldSize.goal_depth() / 1000.0f,
            Field_Dimensions::Default_Dimensions.GoalHeight(),
            defaultDim.PenaltyShortDist(),           // PenaltyShortDist
            defaultDim.PenaltyLongDist(),            // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2));
    } else {
        cerr << "Error: failed to decode SSL geometry packet. Not resizing "
                "field."
             << endl;
    }
}
