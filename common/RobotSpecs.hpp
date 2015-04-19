
/**
 * Contains hardware parameters of the robot that differ from revision to revision.
 */
struct RobotSpecs {
    /// Total mass of robot in Kg
    float Mass;

    /// Moment of inertia of robot about the vertical axis.  Measured in Kg*m^2
    float MomentOfInertia;

    /// This is the "effective" wheel radius, which lies somewhere between the radius of the omniwheel body
    /// and the outermost distance measured that includes the rollers.
    float WheelRadius;

    /// Angle of each wheel axis relative to the +X axis which runs from the robot's center out it's right side.
    /// Wheels are numbered starting from the front right going counter-clockwise.
    float WheelAngles[4];

    /// Distance from center of robot to the center of the wheel.
    /// The wheels are concentric, so this values is the same for all wheels.
    float WheelDist;

    /// Gear ratio from motor to wheel.  motor_speed (rad/s) * GearRatio = wheel_speed (rad/s).
    float GearRatio;

    /// Viscous friction coefficient of wheel assembly.  torque_friction = motor_speed * WheelAssemblyFrictionCoefficient.
    float WheelAssemblyFrictionCoefficient;

    /// Rotational inertia of wheel assembly (Kg*m^2).  torque_accel = WheelAssemblyMomentOfInertia * motor_rot_accel.
    float WheelAssemblyMomentOfInertia;

    /// Max voltage available to put across each motor.
    float MotorMaxVoltage;

    /// Resistance of each motor phase (ohms).
    float MotorPhaseResistance;

    /// Back-emf constant of motor.  Multiply by motor speed to get voltage.
    float MotorBackEmfConstant;

    /// Motor torque constant.  Multiply by effective voltage across phase to get torque.
    float MotorTorqueConstant;
};
