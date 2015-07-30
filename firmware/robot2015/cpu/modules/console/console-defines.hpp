
std::string Console::CONSOLE_USER = "jon";
std::string Console::CONSOLE_HOSTNAME = "robot";
std::string Console::CONSOLE_HEADER = Console::CONSOLE_USER + "@" + Console::CONSOLE_HOSTNAME + " $ ";

const std::string Console::RX_BUFFER_FULL_MSG = "RX BUFFER FULL";
const std::string Console::COMMAND_BREAK_MSG = "*BREAK*";

uint16_t Console::baudrate = 9600;
