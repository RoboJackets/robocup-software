#include <QDir>
#include <QDateTime>
#include <QUdpSocket>

#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/LogFrame.pb.h>
#include <protobuf/referee.pb.h>
#include <git_version.hpp>

#include <multicast.hpp>
#include <Network.hpp>
#include <Utils.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>

using namespace std;
using namespace Packet;

void usage(const char* prog) {
    fprintf(stderr, "Usage: %s[-sim] <filename.log>\n", prog);
    fprintf(stderr, "-sim:\t Connect to the simulator for vision packets.\n");
    exit(1);
}

int main(int argc, char* argv[]) {
    auto framePeriod = RJ::Seconds(1) / 60;  // 60 frames per second

    bool simulation = false;
    QString logFile;

    // Process args
    if (argc == 3 && strcmp(argv[1], "-sim") == 0) {
        logFile = argv[2];
        simulation = true;
    } else if (argc == 2) {
        logFile = argv[1];
    } else if (argc != 1) {
        usage(argv[0]);
    }

    if (logFile.isNull()) {
        if (!QDir("logs").exists()) {
            printf("No logs directory and no log file specified\n");
            return 1;
        }

        logFile = QString("logs/") +
                  QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log");
    }

    // Create vision socket
    QUdpSocket visionSocket;
    if (simulation) {
        // The simulator doesn't multicast its vision.  Instead, it sends to two
        // different ports.
        // Try to bind to the first one and, if that fails, use the second one.
        if (!visionSocket.bind(SimVisionPort)) {
            if (!visionSocket.bind(SimVisionPort + 1)) {
                throw runtime_error(
                    "Can't bind to either simulated vision port");
            }
        }
    } else {
        if (!visionSocket.bind(SharedVisionPortDoubleNew,
                               QUdpSocket::ShareAddress)) {
            printf("Can't bind to shared vision port");
            return 1;
        }
    }
    multicast_add(&visionSocket, SharedVisionAddress);

    // Create referee socket
    QUdpSocket refereeSocket;
    if (!refereeSocket.bind(ProtobufRefereePort, QUdpSocket::ShareAddress)) {
        throw runtime_error("Can't bind to shared referee port");
    }

    multicast_add(&refereeSocket, RefereeAddress);

    // Create log file
    int fd = creat(logFile.toLatin1(), 0666);
    if (fd < 0) {
        printf("Can't create %s: %m\n", (const char*)logFile.toLatin1());
        return 1;
    }

    fprintf(stderr, "Writing to %s\n", (const char*)logFile.toLatin1());
    fprintf(stderr, "Press any key and press ENTER to exit\n");

    // Main loop
    LogFrame logFrame;
    bool first = true;
    while (true) {
        auto startTime = RJ::now();

        logFrame.Clear();
        logFrame.set_command_time(RJ::timestamp(startTime));
        logFrame.set_timestamp(RJ::timestamp(startTime));
        logFrame.set_blue_team(false);  // Always assume self is Yellow for logs

        // Check for user input (to exit)
        struct pollfd pfd;
        pfd.fd = 0;
        pfd.events = POLLIN;
        if (poll(&pfd, 1, 0) > 0) {
            // Enter pressed
            break;
        }

        // Read vision data
        while (visionSocket.hasPendingDatagrams()) {
            string buf;
            unsigned int n = visionSocket.pendingDatagramSize();
            buf.resize(n);
            visionSocket.readDatagram(&buf[0], n);

            SSL_WrapperPacket* packet = logFrame.add_raw_vision();
            if (!packet->ParseFromString(buf)) {
                printf("Bad vision packet of %d bytes\n", n);
                continue;
            }
        }

        // Read referee data
        bool firstRefpacket = true;
        while (refereeSocket.hasPendingDatagrams()) {
            string buf;
            unsigned int n = refereeSocket.pendingDatagramSize();
            buf.resize(n);
            refereeSocket.readDatagram(&buf[0], n);

            SSL_Referee* packet = logFrame.add_raw_refbox();
            if (!packet->ParseFromString(buf)) {
                printf("Bad referee packet of %d bytes\n", n);
                continue;
            }

            // Copy team names into LogFrame
            if (firstRefpacket) {
                firstRefpacket = false;

                logFrame.set_team_name_yellow(packet->yellow().name());
                logFrame.set_team_name_blue(packet->blue().name());
            }
        }

        if (first) {
            first = false;

            LogConfig* logConfig = logFrame.mutable_log_config();
            logConfig->set_generator("simple_logger");
            logConfig->set_git_version_hash(git_version_hash);
            logConfig->set_git_version_dirty(git_version_dirty);
            logConfig->set_simulation(simulation);
        }

        uint32_t size = logFrame.ByteSize();
        if (write(fd, &size, sizeof(size)) != sizeof(size)) {
            printf("Failed to write size: %m\n");
            break;
        } else if (!logFrame.SerializeToFileDescriptor(fd)) {
            printf("Failed to write frame: %m\n");
            break;
        }

        auto endTime = RJ::now();
        auto computationTime = endTime - startTime;
        if (computationTime < framePeriod) {
            auto sleepPeriod = framePeriod - computationTime;
            usleep(RJ::numMicroseconds(sleepPeriod));
        } else {
            printf("Processor took too long: %ld us\n",
                   RJ::numMicroseconds(computationTime));
        }
    }

    // Discard input on stdin
    tcflush(0, TCIFLUSH);

    printf("Done.\n");

    return 0;
}
