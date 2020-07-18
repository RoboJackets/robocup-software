#include "rj_common/RefereeEnums.hpp"

namespace RefereeModuleEnums {

std::string stringFromStage(Stage s) {
    return SSL_Referee_Stage_Name(s);
}

std::string stringFromCommand(Command c) {
    return SSL_Referee_Command_Name(c);
}

}  // namespace RefereeModuleEnums
