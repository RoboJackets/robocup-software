#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <protobuf/referee.pb.h>

// Information about a single team.
class TeamInfo {
public:
    // The team's name (empty string if operator has not typed anything).
    std::string name;
    // The number of goals scored by the team during normal play and overtime.
    uint score;
    // The number of red cards issued to the team since the beginning of the
    // game.
    uint red_cards;
    // The amount of time (in microseconds) left on each yellow card issued to
    // the team.
    // If no yellow cards are issued, this array has no elements.
    // Otherwise, times are ordered from smallest to largest.
    std::vector<uint> yellow_card_times;
    // The total number of yellow cards ever issued to the team.
    uint yellow_cards;
    // The number of timeouts this team can still call.
    // If in a timeout right now, that timeout is excluded.
    uint timeouts_left;
    // The number of microseconds of timeout this team can use.
    uint timeout_time;
    // The pattern number of this team's goalie.
    uint goalie;

    TeamInfo()
        : name(""),
          score(0),
          red_cards(0),
          yellow_cards(0),
          timeouts_left(0),
          timeout_time(0),
          goalie(0) {}

    void ParseRefboxPacket(SSL_Referee_TeamInfo packet) {
        name = packet.name();
        score = packet.score();
        red_cards = packet.red_cards();
        yellow_cards = packet.yellow_cards();
        yellow_card_times.resize(yellow_cards);
        for (int i = 0; i < packet.yellow_card_times_size(); i++)
            yellow_card_times[i] = packet.yellow_card_times(i);
        timeouts_left = packet.timeouts();
        timeout_time = packet.timeout_time();
        goalie = packet.goalie();
    }
};
