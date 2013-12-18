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
const QColor Dot_Pattern_Colors[12][4] = {
    {Qt::magenta, Qt::magenta, Qt:: magenta, Qt::green},
    {Qt::green, Qt::magenta, Qt:: magenta, Qt::green},
    {Qt::green, Qt::green, Qt:: magenta, Qt::green},
    {Qt::magenta, Qt::green, Qt:: magenta, Qt::green},
    {Qt::magenta, Qt::magenta, Qt:: green, Qt::magenta},
    {Qt::green, Qt::magenta, Qt:: green, Qt::magenta},
    {Qt::green, Qt::green, Qt:: green, Qt::magenta},
    {Qt::magenta, Qt::green, Qt:: green, Qt::magenta},
    {Qt::green, Qt::green, Qt:: green, Qt::green},
    {Qt::magenta, Qt::magenta, Qt:: magenta, Qt::magenta},
    {Qt::magenta, Qt::magenta, Qt:: green, Qt::green},
    {Qt::green, Qt::green, Qt:: magenta, Qt::magenta},
};

#endif // VISIONDOTPATTERN_HPP
