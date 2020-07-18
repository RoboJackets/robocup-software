#pragma once

#include <string>
#include <rj_protos/referee.pb.h>

namespace RefereeModuleEnums {

using Stage = SSL_Referee_Stage;
using Command = SSL_Referee_Command;

std::string stringFromStage(Stage s);
std::string stringFromCommand(Command c);

}  // namespace RefereeModuleEnums
