struct GameSettings {
    
    bool is_simulation;

    bool useOurHalf, useOpponentHalf;

    // True if we are running with a simulator.
    // This changes network communications.
    bool simulation;

    // True if we are blue.
    // False if we are yellow.
    bool blueTeam;

    // _teamTrans is used for positions, not angles.
    // _teamAngle is used for angles.
    float teamAngle;

    // Board ID of the robot to manually control or -1 if none
    int manualID;
    // Use multiple joysticks at once
    bool multipleManual;

    /// Measured framerate
    float framerate;

    // joystick damping
    bool dampedRotation;
    bool dampedTranslation;

    bool kickOnBreakBeam;

    // If true, rotates robot commands from the joystick based on its
    // orientation on the field
    bool useFieldOrientedManualDrive = false;

    bool initialized;

    bool paused;
};