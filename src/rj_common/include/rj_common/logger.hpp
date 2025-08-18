#pragma once

#include <deque>
#include <fstream>
#include <optional>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <spdlog/spdlog.h>

#include <rj_protos/LogFrame.pb.h>
#include <rj_utils/logging.hpp>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/radio/robot_status.hpp"
#include "rj_common/radio/packet_convert.hpp"
#include "rj_common/time.hpp"
#include "rj_common/node.hpp"
#include "rj_common/robot_intent.hpp"
#include "rj_common/world_state.hpp"
#include "rj_common/context.hpp"

// For FRIEND_TEST
// #include <gtest/gtest_prod.h>

/**
 * \brief Populates the log frame in Context, and writes it to a file.
 *
 * When viewing logs, reads logs from a file and populates the log frame.
 */
class Logger : public Node {
public:
    Logger(Context* context) : context_(context) {}

    /**
     * \brief Open the given file for reading
     */
    void read(const std::string& filename);

    /**
     * \brief Open the given file for writing.
     */
    void write(const std::string& filename);

    /**
     * \brief Close the current log file.
     */
    void close();

    /**
     * \brief Flush the file to disk.
     */

    void start() override;
    void run() override;
    void stop() override;

private:
    static std::shared_ptr<Packet::LogFrame> create_log_frame(Context* context);
    static bool write_to_file(Packet::LogFrame* frame,
                            google::protobuf::io::ZeroCopyOutputStream* out);
    static bool read_from_file(Packet::LogFrame* frame,
                             google::protobuf::io::ZeroCopyInputStream* in);
    static void fill_robot(Packet::LogFrame::Robot* out, int shell_id,
                          RobotState const* state, RobotStatus const* status,
                          MotionSetpoint const* setpoint);

    FRIEND_TEST(Logger, SaveContext);
    FRIEND_TEST(Logger, SerializeDeserialize);

    std::optional<std::fstream> log_file_;

    Context* context_;
};
