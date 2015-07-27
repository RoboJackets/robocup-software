#include "firmware-check.hpp"

/**
 * Sets the string to the cortex firmware verison. This is taken from the `mbed interface` chip.
 */
void firmware_version(std::string &version)
{
    FILE *fwp = fopen("/local/MBED.HTM", "r");  // open the mbed's default file to determine the mbed firmware version
    char temp_buf[300];

    version.clear();    // clear the passed string to ensure it's empty

    uint16_t line_nbr = 0;

    while ( (fgets(temp_buf, 300, fwp) != NULL) & (line_nbr < 30) ) {
        std::string temp_string(temp_buf);
        line_nbr++;

        if ( temp_string.find("<meta ") != std::string::npos ) {

            // the word that we need to find the position of
            std::string search_word( "&firmware=" );

            // find the initial position from the keyword we're looking for
            std::size_t pos1 = temp_string.find(search_word) + search_word.length();

            // create substring starting AFTER `&firmware=` and going to the end of the line
            std::string sub_str( temp_string.substr(pos1) );

            // now find the position of the next `&` character from the sub string
            std::size_t pos2 = sub_str.find("&");

            // parse out the firmware's version number
            version = sub_str.substr(0, pos2);
            break;
        }
    }

    // close the file handle
    fclose(fwp);
}
