#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "Context.hpp"
#include "Logger.hpp"

// NOLINTNEXTLINE
TEST(Logger, SaveContext) {
    RJ::Time start_time = RJ::now();

    // Make sure the logger saves stuff from Context. This is not anywhere near
    // exhaustive, it just gets a few items.
    Context context;

    // It should save robot states, and their radio packets
    context.world_state.our_robots.at(0) =
        RobotState{Geometry2d::Pose(1, 2, 3), Geometry2d::Twist(4, 5, 6), start_time, true};
    context.robot_status.at(0).timestamp = start_time;
    context.robot_status.at(0).kicker = RobotStatus::KickerState::kCharged;

    // It should not save invisible robots though
    context.world_state.our_robots.at(1) =
        RobotState{Geometry2d::Pose(1, 2, 3), Geometry2d::Twist(4, 5, 6), start_time, false};

    // Or their radio packets
    context.robot_status.at(1).kicker = RobotStatus::KickerState::kFailed;

    // We should use blue_team from GameState, not GameSettings
    context.game_settings.request_blue_team = true;
    context.blue_team = false;

    std::shared_ptr<Packet::LogFrame> frame = Logger::create_log_frame(&context);

    EXPECT_EQ(frame->self_size(), 1);
    EXPECT_EQ(frame->self(0).shell(), 0);
    EXPECT_EQ(frame->self(0).pos().x(), 1);
    EXPECT_EQ(frame->self(0).pos().y(), 2);
    EXPECT_EQ(frame->self(0).angle(), 3);
    EXPECT_EQ(frame->radio_rx_size(), 1);

    EXPECT_EQ(frame->blue_team(), false);
}

// NOLINTNEXTLINE
TEST(Logger, SerializeDeserialize) {
    // Make a simple log frame
    Context context;

    RJ::Time start_time = RJ::now();
    context.world_state.our_robots.at(0) =
        RobotState{Geometry2d::Pose(1, 2, 3), Geometry2d::Twist(4, 5, 6), start_time, true};
    context.robot_status.at(0).timestamp = start_time;
    context.robot_status.at(0).kicker = RobotStatus::KickerState::kCharged;

    std::shared_ptr<Packet::LogFrame> frame = Logger::create_log_frame(&context);

    // Allocate 1MiB
    std::vector<char> data(1 << 20);
    google::protobuf::io::ArrayOutputStream output(data.data(), data.size());

    constexpr int kExpected = 3;
    for (int i = 0; i < 3; i++) {
        EXPECT_TRUE(Logger::write_to_file(frame.get(), &output));
    }

    Packet::LogFrame other_frame;
    int count = 0;

    google::protobuf::util::MessageDifferencer differencer;
    std::string diff;
    differencer.ReportDifferencesToString(&diff);

    google::protobuf::io::ArrayInputStream input(data.data(), output.ByteCount());
    while (Logger::read_from_file(&other_frame, &input)) {
        count++;
        EXPECT_TRUE(differencer.Compare(*frame, other_frame));
        other_frame.Clear();
    }

    std::cout << diff << std::endl;

    EXPECT_EQ(kExpected, count);
}