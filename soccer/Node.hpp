#pragma once

/**
 * The base class for a module run by Processor. Override this and add it to
 * the nodes vector in Processor to have it run in the update loop.
 */
class Node {
public:
    virtual ~Node() {}

    virtual void start(){};

    // The callback
    virtual void run() = 0;

    virtual void stop(){};
};
