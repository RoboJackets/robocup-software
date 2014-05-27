

class RoleRequirements:

    def __init__(self):
        self.pos = None
        self.has_ball = False
        self.has_chipper = False


    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        self._pos = value


    @property
    def has_ball(self):
        return self._has_ball
    @has_ball.setter
    def has_ball(self, value):
        self._has_ball = value


    @property
    def has_chipper(self):
        return self._has_chipper
    @has_chipper.setter
    def has_chipper(self, value):
        self._has_chipper = value
