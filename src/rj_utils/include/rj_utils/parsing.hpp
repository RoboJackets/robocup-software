/**
 * @file parsing.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Useful parsing functions
 * @version 0.1
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <string_view>

namespace rj_utils {

/**
 * @brief A number of nodes have a namespace of "/robot_{id}", where id is
 * their robot_id.  This function will parse the namespace and return the
 * robot_id of the node
 * 
 * @param name The namespace of the node
 * @return int The robot id of the node
 */
inline int parse_id_from_namespace(const char* name) {
    constexpr std::string_view kPrefix = "/robot_";

    std::string_view view { name };
    view.remove_prefix(kPrefix.size());

    int value = 0;
    for (char character : view) {
        if (character < '0' || character > '9') {
            break;
        }

        value = value * 10 + (character - '0');
    }

    return value;
}

} // namespace rj_utils