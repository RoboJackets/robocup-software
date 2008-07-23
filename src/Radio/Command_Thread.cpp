#include <sys/time.h>
#include <math.h>
#include <termios.h>

#include "Command_Thread.hpp"
#include "Radio.hpp"
#include "crc.hpp"
#include "main.hpp"

using namespace Packet;

#include <QTime>

QTime last_time;

Robot::Robot(Command_Thread *thread, int id)
{
    last_time = QTime::currentTime();
	_thread = thread;
	_id = id;
	_locked = false;
    _battery = 0;
    _rssi = 0;
}

float Robot::battery() const
{
    QMutexLocker ml(&status_mutex);
    return _battery;
}

uint8_t Robot::rssi() const
{
    QMutexLocker ml(&status_mutex);
    return _rssi;
}

bool Robot::ball() const
{
    QMutexLocker ml(&status_mutex);
    return _ball;
}

////////

Send_Thread::Send_Thread(Command_Thread *ct)
{
    _command_thread = ct;
}

void Send_Thread::stop()
{
    _running = false;
    _command_thread->packet_wait.wakeAll();
    wait();
}

void Send_Thread::run()
{
    _running = true;
    _command_thread->mutex.lock();
    while (_running)
    {
        _command_thread->packet_wait.wait(&_command_thread->mutex);
        if (!_running)
        {
            break;
        }
        
        _command_thread->send_command();
    }
    _command_thread->mutex.unlock();
}

////////

Command_Thread::Command_Thread(Radio *radio, Team team)
{
	_radio = radio;
    _team = team;
    _running = true;
    _scan_id = 0;
    _scan_wait = 0;
    _cal_robot = -1;
    _send_thread = new Send_Thread(this);
    
    for (int i = 0; i < Num_Robots; ++i)
    {
        _robots[i] = new Robot(this, i);
    }
}

void Command_Thread::stop()
{
    _running = false;
    wait();
    
    _send_thread->stop();
}

void Command_Thread::send_cal(int robot, float p, float d)
{
    QMutexLocker ml(&mutex);
    _cal_robot = robot;
    _cal_p = p;
    _cal_d = d;
}

void Command_Thread::command_received(const CommData* data)
{
    mutex.lock();
    
	if (data)
	{
		if (data->mode != _command.mode)
		{
			// Mode changed, so reset the scan.
			_scan_id = 0;
            _scan_wait = 0;
		}
		
		_command = *data;
	} else {
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			_command.robots[i].valid = false;
		}
	}
    
    mutex.unlock();
    packet_wait.wakeAll();
}

void Command_Thread::run()
{
    _send_thread->start();
    
    struct timeval last_time;
    
    _receiver.addType(_team, this, &Command_Thread::command_received, 100);
    
    while (_running)
    {
        // Wait for a packet
        _receiver.receive();
    }
}

void Command_Thread::send_command()
{
    // Called from Send_Thread, don't need to lock the mutex.
    
    uint8_t packet[26];
    memset(packet, 0, sizeof(packet));
        
    // Motor speeds
    int i = 0;
    for (int robot = 0; robot < 5; ++robot)
    {
        if (_command.robots[robot].valid)
        {
            packet[i++] = _command.robots[robot].motor[2];
            packet[i++] = _command.robots[robot].motor[3];
            packet[i++] = _command.robots[robot].motor[0];
            packet[i++] = _command.robots[robot].motor[1];
        } else {
            for (int motor = 0; motor < 4; ++motor)
            {
                packet[i++] = 0;
            }
        }
    }
    
    Robot *robot = 0;
    switch (_command.mode)
    {
        case CommData::Fixed:
            robot = _robots[_command.robot];
            break;
        
        case CommData::ScanRobots:
            if (_scan_wait == 0)
            {
                _scan_wait = 5;
                _scan_id = (_scan_id + 1) % Num_Robots;
            } else {
                --_scan_wait;
            }
            robot = _robots[_scan_id];
            break;
    }
    //robot = _robots[1];
    
    const Packet::CommData::Robot &rc = _command.robots[robot->id()];
    
    // Calibration
    bool cal = false;
    if (_cal_robot >= 0 && _cal_robot < Num_Robots)
    {
        cal = true;
        robot = _robots[_cal_robot];
    }
    
    bool set_reverse = false;
    
    // Robot ID and one-touch flag
    bool one_touch = true;//rc.oneTouch;
    //FIXME - Always scan for kick/dribble
    packet[20] = ((cal | set_reverse) << 7) | (robot->id() << 4) |
                 ((one_touch | set_reverse) << 3) | (robot->id() & 7);
    
    if (debug)
    {
//        printf("select %02x mode %d #%d\n", packet[20], _command.mode, _command.robot);
    }
    
    if (cal)
    {
        int p = (int)roundf(_cal_p * 256);
        int d = (int)roundf(_cal_d * 256);
        packet[_cal_robot * 4] = p & 0xff;
        packet[_cal_robot * 4 + 1] = p >> 8;
        packet[_cal_robot * 4 + 2] = d & 0xff;
        packet[_cal_robot * 4 + 3] = d >> 8;
        if (debug)
        {
            printf("send cal %02x: %02x %02x %02x %02x\n",
                packet[20],
                packet[_cal_robot * 4],
                packet[_cal_robot * 4 + 1],
                packet[_cal_robot * 4 + 2],
                packet[_cal_robot * 4 + 3]);
        }
        _cal_robot = -1;
    }
    
    // Kick
    if (rc.valid)
    {
        packet[21] = rc.kick;
    }
    
    if (debug && packet[21])
    {
        printf("kick %d %02x\n", robot->id(), packet[21]);
    }
    
    // Dribble
    if (set_reverse)
    {
        packet[22] = 45;
    } else if (rc.valid)
    {
        packet[22] = -_command.robots[robot->id()].roller;
    }
    
    packet[23] = 0;//++_sequence;
    
    crc_set(packet, sizeof(packet));
    
    for (int i = 0; i < sizeof(packet); ++i)
    {
        fprintf(stderr, "%02x ", packet[i]);
        packet[i] ^= 0x55;
    }
    
    QTime t = QTime::currentTime();
    fprintf(stderr, "%4d", last_time.msecsTo(t));
    last_time = t;
    
    fprintf(stderr, "\n");
    
    _radio->serial()->write(packet, sizeof(packet));
}
