#include "Status_Thread.hpp"
#include "Command_Thread.hpp"
#include "Radio.hpp"
#include "crc.hpp"
#include "main.hpp"

#include <sys/time.h>

Status_Thread::Status_Thread(Command_Thread *ct, Team team)
{
    _command_thread = ct;
    _radio = ct->radio();
    _team = team;
    _running = true;
    _sender = 0;
}

void Status_Thread::stop()
{
    _running = false;
    wait();
}

void Status_Thread::run()
{
    struct timeval last;
    
    _sender = new Packet::Sender<Packet::RobotStatus>(_team);

    int pos = 0;
    uint8_t packet[7];
    
    gettimeofday(&last, 0);
    while (_running)
    {
        uint8_t byte = _radio->serial()->read() ^ 0x55;
        if (debug)
        {
            //printf("read %02x\n", byte);
        }
        
        // Get the time since the last byte
        struct timeval t;
        gettimeofday(&t, 0);
        int delta_us = t.tv_usec - last.tv_usec + (t.tv_sec - last.tv_sec) * 1000000;
        last = t;
        
        // Sync time
        if (delta_us > 2000)
        {
            if (byte == 0xc9)
            {
                // Sometimes the first byte is reported late
                pos = 0;
            } else {
                pos = -1;
            }
        }
        
//        printf("%2d %02x %5d\n", pos, byte, delta_us);
        if (pos >= 0 && pos < sizeof(packet))
        {
            packet[pos] = byte;
        }
        
        if (pos < (int)sizeof(packet))
        {
            ++pos;
        }
        
        if (pos == sizeof(packet))
        {
            if (packet[0] == 0xc9 && crc_check(packet, sizeof(packet)))
            {
                handle_packet(packet);
            }
            
            pos = 0;
        }
    }
}

void Status_Thread::handle_packet(const uint8_t *packet)
{
    uint8_t flags = packet[3];
    int id = flags >> 4;
    
    float battery = 5.0f * 3.3f * packet[1] / 255.0f;
    uint8_t rssi = packet[2];
    bool ball = packet[3] & 2;
    
    uint8_t expect_sequence = _last_sequence[id] + 1;
    int sequence = packet[4];
    _last_sequence[id] = sequence;
    
    {
        Robot *robot = _command_thread->robot(id);
        QMutexLocker ml(&robot->status_mutex);
        robot->_update_time = QTime::currentTime();
        robot->_battery = battery;
        robot->_rssi = rssi;
        robot->_ball = ball;
        robot->_charged = packet[3] & 1;
    }
    
    send();
}

void Status_Thread::send()
{
    Packet::RobotStatus status;
    for (int i = 0; i < 5; ++i)
    {
        Robot *robot = _command_thread->robot(i);
        
        QMutexLocker ml(&robot->status_mutex);
        status.robots[i].battery = robot->_battery;
        status.robots[i].rssi = robot->_rssi;
        status.robots[i].charged = robot->_charged;
        status.robots[i].ballPresent = robot->_ball;
    }
    
    //FIXME - timestamp
    _sender->send(status);
}
