#pragma once

#include <QThread>

class Env;

class CommandReceiver: public QThread
{
    public:
        CommandReceiver(Env *env);
        ~CommandReceiver();
        
    protected:
        void run();
        
        Env *_env;
};
