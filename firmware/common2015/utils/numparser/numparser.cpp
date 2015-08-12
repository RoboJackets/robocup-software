#include "numparser.hpp"


bool isInt(std::string& s)
{
    std::string::const_iterator it = s.begin();

    while (it != s.end() && std::isdigit(*it)) ++it;

    return !s.empty() && it == s.end();
}
