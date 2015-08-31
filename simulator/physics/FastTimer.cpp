#include "FastTimer.hpp"
#include <sys/time.h>
#include <stdio.h>
#include <QEvent>
#include <QApplication>

FastTimer::FastTimer(QObject* parent)
    : QThread(parent), _us(0), _running(false) {
    pthread_mutex_init(&_mutex, nullptr);
    pthread_cond_init(&_cond, nullptr);
}

FastTimer::~FastTimer() {
    if (_running) {
        stop();

        // The event loop may not be running any more
        pthread_mutex_lock(&_mutex);
        pthread_cond_signal(&_cond);
        pthread_mutex_unlock(&_mutex);

        //         terminate();
        // FIXME - What if the delay is really long?
        wait();
    }
}

void FastTimer::start(int ms) {
    _us = ms * 1000;
    if (!_running) {
        _running = true;
        QThread::start();
    }
}

void FastTimer::stop() { _running = false; }

void FastTimer::run() {
    struct timeval start;
    gettimeofday(&start, nullptr);
    pthread_mutex_lock(&_mutex);
    while (_running) {
        struct timeval t;
        gettimeofday(&t, nullptr);
        useconds_t us =
            (t.tv_sec - start.tv_sec) * 1000000 + t.tv_usec - start.tv_usec;
        start = t;

        if (_us > us) {
            usleep(_us - us);
            gettimeofday(&start, nullptr);
        }

        QApplication::instance()->postEvent(this, new QEvent(QEvent::User));
        pthread_cond_wait(&_cond, &_mutex);
    }
    pthread_mutex_unlock(&_mutex);
}

bool FastTimer::event(QEvent* e) {
    if (e->type() == QEvent::User) {
        timeout();
        pthread_mutex_lock(&_mutex);
        pthread_cond_signal(&_cond);
        pthread_mutex_unlock(&_mutex);
        e->accept();
        return true;
    } else {
        return QObject::event(e);
    }
}
