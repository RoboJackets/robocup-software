#pragma once

#include <string>

#include <rj_protos/state/ssl_gc_referee_message.pb.h>

namespace referee_module_enums {

using Stage = Referee_Stage;
using Command = Referee_Command;

std::string string_from_stage(Stage s);
std::string string_from_command(Command c);

}  // namespace referee_module_enums