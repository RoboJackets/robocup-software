#pragma once

#include "Node.hpp"
#include "Radio.hpp"

class RadioNode : public Node {
public:
    RadioNode(Context* context, bool sim);

    bool isOpen();
    void run() override;
    void switchTeam(bool blueTeam);

private:

    bool _blueTeam;
    Context* _context;
    Radio _radio;
    bool _simulation;
};