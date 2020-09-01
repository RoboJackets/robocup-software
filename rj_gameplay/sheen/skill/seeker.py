from abc import ABC, abstractmethod

import sheen.skill as skill


class ISeeker(skill.ISkill, ABC):
    ...


class Seeker(ISeeker):
    def define(self):
        pass
