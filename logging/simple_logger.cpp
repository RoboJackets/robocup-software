#include <QDir>
#include <QDateTime>
#include <QUdpSocket>

#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/LogFrame.pb.h>
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

int main(int argc, char *argv[])
{
	int framePeriod = 1000000 / 60;
	
	QString logFile;
	
	// Determine log file name
	if (argc == 2)
	{
		logFile = argv[1];
	}
	
	if (logFile.isNull())
	{
		if (!QDir("logs").exists())
		{
			printf("No logs directory and no log file specified\n");
			return 1;
		}
		
		logFile = QString("logs/") + QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss.log");
	}
	
	// Create vision socket
	QUdpSocket visionSocket;
	if (!visionSocket.bind(SharedVisionPort, QUdpSocket::ShareAddress))
	{
		printf("Can't bind to shared vision port");
		return 1;
	}
	multicast_add(&visionSocket, SharedVisionAddress);
	
	// Create referee socket
	QUdpSocket refereeSocket;
	if (!refereeSocket.bind(LegacyRefereePort, QUdpSocket::ShareAddress))
	{
		printf("Can't bind to referee port");
		return 1;
	}
	multicast_add(&refereeSocket, RefereeAddress);
	
	// Create log file
	int fd = creat(logFile.toLatin1(), 0666);
	if (fd < 0)
	{
		printf("Can't create %s: %m\n", (const char *)logFile.toLatin1());
		return 1;
	}
	
	printf("Writing to %s\n", (const char *)logFile.toLatin1());
	
	// Main loop
	LogFrame logFrame;
	bool first = true;
	while (true)
	{
		Time startTime = timestamp();
		
		logFrame.Clear();
		logFrame.set_command_time(startTime);
		
		// Check for user input (to exit)
		struct pollfd pfd;
		pfd.fd = 0;
		pfd.events = POLLIN;
		if (poll(&pfd, 1, 0) > 0)
		{
			// Enter pressed
			break;
		}
		
		// Read vision data
		while (visionSocket.hasPendingDatagrams())
		{
			string buf;
			unsigned int n = visionSocket.pendingDatagramSize();
			buf.resize(n);
			visionSocket.readDatagram(&buf[0], n);
			
			SSL_WrapperPacket *packet = logFrame.add_raw_vision();
			if (!packet->ParseFromString(buf))
			{
				printf("Bad vision packet of %d bytes\n", n);
				continue;
			}
		}
		
		// Read referee data
		while (refereeSocket.hasPendingDatagrams())
		{
			unsigned int n = refereeSocket.pendingDatagramSize();
			string str(6, 0);
			refereeSocket.readDatagram(&str[0], str.size());
			
			// Check the size after receiving to discard bad packets
			if (n != str.size())
			{
				printf("Bad referee packet of %d bytes\n", n);
				continue;
			}
			
			logFrame.add_raw_referee(str);
		}
		
		if (first)
		{
			first = false;
			
			LogConfig *logConfig = logFrame.mutable_log_config();
			logConfig->set_generator("simple_logger");
			logConfig->set_git_version_hash(git_version_hash);
			logConfig->set_git_version_dirty(git_version_dirty);
		}
		
		uint32_t size = logFrame.ByteSize();
		if (write(fd, &size, sizeof(size)) != sizeof(size))
		{
			printf("Failed to write size: %m\n");
			break;
		} else if (!logFrame.SerializeToFileDescriptor(fd))
		{
			printf("Failed to write frame: %m\n");
			break;
		}
		
		Time endTime = timestamp();
		int lastFrameTime = endTime - startTime;
		if (lastFrameTime < framePeriod)
		{
			usleep(framePeriod - lastFrameTime);
		} else {
			printf("Processor took too long: %d us\n", lastFrameTime);
		}
	}
	
	// Discard input on stdin
	tcflush(0, TCIFLUSH);
	
	printf("Done.\n");
	
	return 0;
}
