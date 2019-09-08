#pragma once

/**
 * The base class for a module run by Processor. Override this and add it to
 * the modules vector in Processor to have it run in the update loop.
 */
class Module {
public:
    virtual void start(){};

    // The callback
    virtual void run() = 0;

    virtual void stop(){};
};
