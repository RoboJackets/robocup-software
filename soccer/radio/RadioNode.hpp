#pragma once

#include "Node.hpp"
#include "Radio.hpp"

class RadioNode : public Node {
public:
    RadioNode(Context* context, bool sim, bool blueTeam);

    bool isOpen();
    void run() override;
    void switchTeam(bool blueTeam);

private:

    bool _blueTeam;
    Context* _context;
    std:unique_ptr<Radio> _radio;
    bool _simulation;
};