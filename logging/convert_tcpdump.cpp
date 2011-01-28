#include <stdio.h>
#include <pcap.h>
#include <stdint.h>
#include <fcntl.h>

#include <protobuf/LogFrame.pb.h>
#include <git_version.h>

using namespace std;
using namespace Packet;

const uint8_t referee_addr[] = {224, 5, 23, 1};
const uint8_t referee_port[] = {0x27, 0x11};		// 10001
const uint8_t vision_addr[] = {224, 5, 23, 2};
const uint8_t vision_port[] = {0x27, 0x12};			// 10002

const unsigned int FramePeriod = 1000000 / 60;

bool writeFrame(int fd, const LogFrame &frame)
{
	uint32_t size = frame.ByteSize();
	if (write(fd, &size, sizeof(size)) != sizeof(size))
	{
		printf("Failed to write size: %m\n");
		return false;
	}
	
	if (!frame.SerializePartialToFileDescriptor(fd))
	{
		printf("Failed to write packet: %m\n");
		return false;
	}
	
	return true;
}

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		printf("Usage: %s <tcpdump.cap> <output.log>\n", argv[0]);
		return 1;
	}
	
	char errbuf[PCAP_ERRBUF_SIZE];
	
	pcap_t *pcap = pcap_open_offline(argv[1], errbuf);
	if (!pcap)
	{
		printf("pcap_open_offline: %s\n", errbuf);
		return 1;
	}
	
	int out_fd = creat(argv[2], 0666);
	if (out_fd < 0)
	{
		printf("Can't create %s: %m\n", argv[2]);
		pcap_close(pcap);
		return 1;
	}
	
	LogFrame frame;
	bool needWrite = false;
	
	// Pretend we were reading packets in a fixed-frequency loop (like Processor).
	// endTime is the time that a simulated processing loop iteration ends.
	uint64_t endTime = 0;
	
	bool first = true;
	while (true)
	{
		struct pcap_pkthdr header;
		const u_char *data = pcap_next(pcap, &header);
		if (!data)
		{
			break;
		}
		
		// Verify ethertype for IPv4
		if (data[12] != 8 || data[13] != 0)
		{
			continue;
		}
		
		// Require IPv4 with no options
		if (data[14] != 0x45)
		{
			continue;
		}
		
		// FIXME - Checksum
		
		uint64_t timestamp = header.ts.tv_usec + header.ts.tv_sec * 1000000;
		
		if (!endTime)
		{
			// Find the time to write the first frame
			endTime = timestamp + FramePeriod;
		}
		
		if (timestamp >= endTime)
		{
			// We're about to handle a packet received after this iteration,
			// so write everything we have.
			
			// Calculate the time this iteration would have started
			frame.set_start_time(endTime - FramePeriod);
			
			// Advance endTime to the next iteration.
			endTime += FramePeriod;
			
			// Add software information
			if (first)
			{
				first = false;
				
				LogConfig *logConfig = frame.mutable_log_config();
				logConfig->set_generator("convert_tcpdump");
				logConfig->set_git_version_hash(git_version_hash);
				logConfig->set_git_version_dirty(git_version_dirty);
			}
			
			// Write this frame
			needWrite = false;
			if (!writeFrame(out_fd, frame))
			{
				break;
			}
			
			// Clear for the next iteration
			frame.Clear();
		}
		
		// Check for data we're interested in
		if (!memcmp(data + 30, referee_addr, 4) && !memcmp(data + 36, referee_port, 2))
		{
			// Referee packet
			frame.add_raw_referee(data + 42, header.caplen - 42);
			needWrite = true;
		} else if (!memcmp(data + 30, vision_addr, 4) && !memcmp(data + 36, vision_port, 2))
		{
			// Vision packet
			SSL_WrapperPacket *packet = frame.add_raw_vision();
			if (!packet->ParseFromArray(data + 42, header.caplen - 42))
			{
				printf("Failed to parse: %s\n", packet->InitializationErrorString().c_str());
			}
			needWrite = true;
		}
	}
	
	if (needWrite)
	{
		writeFrame(out_fd, frame);
	}
	
	close(out_fd);
	pcap_close(pcap);
	
	return 0;
}
