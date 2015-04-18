#pragma once

#include <QThread>
#include <sys/types.h>
#include <pthread.h>

/**
 * QTimer uses busy waits and eats CPU time badly.
 * This timer provides a recurring timeout() signal with much less overhead.
 */
class FastTimer: public QThread
{
    Q_OBJECT;

public:
    FastTimer(QObject *parent = 0);
    virtual ~FastTimer();

    void start(int ms);
    void stop();

signals:
    void timeout();

protected:
    virtual void run();
    virtual bool event(QEvent *e);

    useconds_t _us;
    volatile bool _running;
    pthread_mutex_t _mutex;
    pthread_cond_t _cond;
};
