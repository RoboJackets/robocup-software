#pragma once

#include "Node.hpp"
#include "Radio.hpp"

class RadioNode : public Node {
public:
    RadioNode(Context* context, bool sim, bool blueTeam);

    bool isOpen();
    RJ::Time getLastRadioRxTime;
    void run() override;
    void switchTeam(bool blueTeam);

private:

    bool _blueTeam;
    Context* _context;
    RJ::Time _lastRadioRxTime;
    std::unique_ptr<Radio> _radio;
    bool _simulation;
};