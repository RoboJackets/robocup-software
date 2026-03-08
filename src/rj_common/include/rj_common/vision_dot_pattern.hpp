#ifndef VISIONDOTPATTERN_HPP
#define VISIONDOTPATTERN_HPP
#include <QColor>

/*!
 * \brief Defines the dot pattern for the robot shells.
 *
 * This is based on the 2013 rules document section 4.9 (Figure 6).
 * The first index of the array is the shell number of the robot (0 - 11)
 * The second index is the index of the dot. Here, the dots are indexed
 * starting with the upper-left dot (per Figure 6) and proceding
 * clockwise around the robot.
 */
const QColor kDotPatternColors[16][4] = {
    {Qt::magenta, Qt::magenta, Qt::magenta, Qt::green}, // ID = 0
    {Qt::green, Qt::magenta, Qt::magenta, Qt::green}, // ID = 1
    {Qt::green, Qt::green, Qt::magenta, Qt::green}, // ID = 2
    {Qt::magenta, Qt::green, Qt::magenta, Qt::green}, // ID = 3
    {Qt::magenta, Qt::magenta, Qt::green, Qt::magenta}, // ID = 4
    {Qt::green, Qt::magenta, Qt::green, Qt::magenta}, // ID = 5
    {Qt::green, Qt::green, Qt::green, Qt::magenta}, // ID = 6
    {Qt::magenta, Qt::green, Qt::green, Qt::magenta}, // ID = 7
    {Qt::green, Qt::green, Qt::green, Qt::green}, // ID = 8
    {Qt::magenta, Qt::magenta, Qt::magenta, Qt::magenta}, // ID = 9
    {Qt::magenta, Qt::magenta, Qt::green, Qt::green}, // ID = 10
    {Qt::green, Qt::green, Qt::magenta, Qt::magenta}, // ID = 11
    {Qt::green, Qt::magenta, Qt::green, Qt::green}, // ID = 12
    {Qt::green, Qt::magenta, Qt::magenta, Qt::magenta}, // ID = 13
    {Qt::magenta, Qt::green, Qt::green, Qt::green}, // ID = 14
    {Qt::magenta, Qt::green, Qt::magenta, Qt::magenta} // ID = 15
};

#endif  // VISIONDOTPATTERN_HPP
