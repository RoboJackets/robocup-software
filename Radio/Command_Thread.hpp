#ifndef _COMMAND_THREAD_HPP_
#define _COMMAND_THREAD_HPP_

#include <QThread>
#include <QMutex>
#include <QTime>
#include <QWaitCondition>

#include <Packet/CommData.hpp>
#include <Packet/PacketReceiver.hpp>

#include <list>

class Command_Thread;
class Radio;

class Robot
{
public:
	Robot(Command_Thread *thread, int robot_id);

	int id() const { return _id; }
	
	bool locked() const { return _locked; }
	
    float battery() const;
    uint8_t rssi() const;
    bool ball() const;
    const QTime &update_time() const { return _update_time; }
    
protected:
    friend class Status_Thread;
    
    mutable QMutex status_mutex;

    QTime _update_time;
    float _battery;
    uint8_t _rssi;
    bool _ball;
    bool _charged;

	Command_Thread *_thread;
	int _id;
	bool _locked;
};

class Send_Thread: public QThread
{
public:
    Send_Thread(Command_Thread *ct);
    
    void stop();
    
protected:
    volatile bool _running;
    Command_Thread *_command_thread;
    
    virtual void run();
};

class Command_Thread: public QThread
{
public:
    static const int Num_Robots = 5;
    
    Command_Thread(Radio *radio, Team team);
    
    Radio *radio() const { return _radio; }
    Robot *robot(int n) const { return _robots[n]; }
    
    Packet::CommData command();
    
    void stop();

    void send_cal(int robot, float p, float d);
    
    // Sends a motion command to the robots.
    void send_command();
    
    QWaitCondition packet_wait;
    
    // Protects all data in this class.
    mutable QMutex mutex;
    
protected:
    friend class Send_Thread;
    
    virtual void run();
    
    void command_received(const Packet::CommData* data);
    
    Send_Thread *_send_thread;
    
    Radio *_radio;
    
    // Most recently received packets
    Packet::CommData _command;
    bool _new_command;
    
    // Last sequence number sent
    uint8_t _sequence;
    
    // True when the thread should keep running.
    volatile bool _running;
    
    Packet::PacketReceiver _receiver;
    
    Team _team;
    
    Robot *_robots[Num_Robots];
    
    // Which robot is selected in the next packet.
    // The meaning of this value depends on _control.mode.
    int _scan_id;
    
    // How many frames until the next robot is selected in a scan.
    int _scan_wait;
    
    // Calibration.  Needs to be sent when _cal_robot is valid [0, 4].
    int _cal_robot;
    float _cal_p, _cal_d;
};

#endif // _COMMAND_THREAD_HPP_
