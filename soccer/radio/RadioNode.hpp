#pragma once

#include "Node.hpp"

class RadioNode : public Node {
public:
    RadioNode(Context* context, bool sim);

    void run() override;

private:
    Context* _context;
    bool _sim;
};