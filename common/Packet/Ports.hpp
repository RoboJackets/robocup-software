#ifndef PORTS_HPP
#define PORTS_HPP

enum
{
    /** Referee data port */
    RefPort = 0,

    /** Vision information */
    VisionPort = 1,

    /** Radio command */
    CommPort = 2,

    /** Motion control commands */
    MotionCmdPort = 3,

    /** Robot status from radio */
    RobotStatusPort = 4,

    /** motion logging information */
    LogMotionPort = 5,

    /** skill commands */
    SkillCmdPort = 6,

    /** skill completion */
    SkillStatusPort = 7,
    
    /** commands for the soccer simulator */
    SoccSimPort = 8
};

#endif /* PORTS_HPP */
