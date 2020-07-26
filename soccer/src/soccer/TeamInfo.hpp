#pragma once

#include <rj_protos/referee.pb.h>
#include <rj_common/time.hpp>

#include <iostream>
#include <string>
#include <vector>

// Information about a single team.
class TeamInfo {
public:
    // The team's name (empty string if operator has not typed anything).
    std::string name = "";
    // The number of goals scored by the team during normal play and overtime.
    uint score = 0;
    // The number of red cards issued to the team since the beginning of the
    // game.
    uint red_cards = 0;
    // The amount of time (in microseconds) left on each yellow card issued to
    // the team.
    // If no yellow cards are issued, this array has no elements.
    // Otherwise, times are ordered from smallest to largest.
    std::vector<RJ::Seconds> yellow_card_times;
    // The total number of yellow cards ever issued to the team.
    uint yellow_cards = 0;
    // The number of timeouts this team can still call.
    // If in a timeout right now, that timeout is excluded.
    uint timeouts_left = 0;
    // The duration of timeout this team can use.
    RJ::Seconds timeout_time{0};
    // The pattern number of this team's goalie.
    uint goalie = 0;

    bool operator==(const TeamInfo& other) {
        return name == other.name && score == other.score &&
               red_cards == other.red_cards &&
               yellow_card_times == other.yellow_card_times &&
               yellow_cards == other.yellow_cards &&
               timeouts_left == other.timeouts_left &&
               timeout_time == other.timeout_time && goalie == other.goalie;
    }

    bool operator!=(const TeamInfo& other) { return !(*this == other); }

    static TeamInfo from_refbox_packet(const SSL_Referee_TeamInfo& packet) {
        TeamInfo info;
        info.name = packet.name();
        info.score = packet.score();
        info.red_cards = packet.red_cards();
        info.yellow_cards = packet.yellow_cards();
        info.yellow_card_times.resize(info.yellow_cards);
        for (int i = 0; i < packet.yellow_card_times_size(); i++) {
            info.yellow_card_times[i] =
                std::chrono::microseconds(packet.yellow_card_times(i));
        }
        info.timeouts_left = packet.timeouts();
        info.timeout_time = std::chrono::microseconds(packet.timeout_time());
        info.goalie = packet.goalie();
        return info;
    }
};
