#include "rj_common/team_color.hpp"

TeamColor opponent(TeamColor team) {
    switch (team) {
    case TeamColor::kBlue:
        return TeamColor::kYellow;
    case TeamColor::kYellow:
        return TeamColor::kBlue;
    }
}
