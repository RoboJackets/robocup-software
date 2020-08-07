#pragma once

#include <rj_protos/referee.pb.h>

#include <string>

namespace RefereeModuleEnums {

using Stage = SSL_Referee_Stage;
using Command = SSL_Referee_Command;

std::string string_from_stage(Stage s);
std::string string_from_command(Command c);

}  // namespace RefereeModuleEnums
