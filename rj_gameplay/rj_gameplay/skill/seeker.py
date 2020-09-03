from abc import ABC, abstractmethod

import stp.skill as skill


class ISeeker(skill.ISkill, ABC):
    ...


class Seeker(ISeeker):
    def define(self):
        pass
