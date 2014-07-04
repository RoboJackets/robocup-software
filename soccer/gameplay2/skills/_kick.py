import single_robot_behavior
import constants


# this is the abstract superclass for PivotKick and LineKick
class _Kick(single_robot_behavior.SingleRobotBehavior):

    def __init__(self):
        super().__init__(continuous=False)

        self.enable_kick = True
        self.use_chipper = False
        self.kick_power = constants.Robot.Kicker.MaxPower
        self.chip_power = constants.Robot.Chipper.MaxPower


    # Allows for different kicker/chipper settings, such as for
    # passing with lower power.
    # Default: full power
    @property
    def kick_power(self):
        return self._kick_power
    @kick_power.setter
    def kick_power(self, value):
        self._kick_power = value
    @property
    def chip_power(self):
        return self._chip_power
    @chip_power.setter
    def chip_power(self, value):
        self._chip_power = value


    # If false, uses straight kicker, if true, uses chipper (if available)
    # Default: False
    @property
    def use_chipper(self):
        return self._use_chipper
    @use_chipper.setter
    def use_chipper(self, value):
        self._use_chipper = value


    # If set to False, will get all ready to go, but won't kick/chip just yet
    # Can be used to synchronize between behaviors
    # Default: True
    @property
    def enable_kick(self):
        return self._enable_kick
    @enable_kick.setter
    def enable_kick(self, value):
        self._enable_kick = value


    def role_requirements(self):
        reqs = super().role_requirements()
        # TODO: prefer chipper?
        # FIXME: require ball carrying / kicking abilities
        return reqs

