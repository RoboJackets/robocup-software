#include "rj_common/referee_enums.hpp"

namespace referee_module_enums {

std::string string_from_stage(Stage s) { return Referee_Stage_Name(s); }

std::string string_from_command(Command c) { return Referee_Command_Name(c); }

}  // namespace referee_module_enums