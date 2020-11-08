#pragma once

#include "context.hpp"
#include "node.hpp"
#include "radio.hpp"

class RadioNode : public Node {
public:
    RadioNode(Context* context, bool sim, bool blue_team);

    bool is_open();
    RJ::Time get_last_radio_rx_time();
    Radio* get_radio();
    void run() override;
    void switch_team(bool blue_team);

private:
    Context* context_;
    RJ::Time last_radio_rx_time_;
    Radio* radio_{};
    bool simulation_;
    bool was_blue_team_;
};