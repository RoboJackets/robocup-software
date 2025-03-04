#include "rc-fshare/git_version.hpp"

#include <iostream>

int main() {
    
    std::cout << "git_version_hash: " << git_version_hash << std::endl
        << "git_version_short_hash: " << git_version_short_hash << std::endl
        << "git_head_date: " << git_head_date << std::endl
        << "git_head_author: " << git_head_author << std::endl
        << "git_version_dirty: " << git_version_dirty << std::endl;


    return 0;
}
