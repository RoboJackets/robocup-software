#pragma once

#include "Context.hpp"
#include "Node.hpp"
#include "Radio.hpp"

class RadioNode : public Node {
public:
    RadioNode(Context* context, bool sim, bool blueTeam);

    bool isOpen();
    RJ::Time getLastRadioRxTime();
    Radio* getRadio();
    void run() override;
    void switchTeam(bool blueTeam);

private:
    Context* _context;
    RJ::Time _lastRadioRxTime;
    Radio* _radio{};
    bool _simulation;
};