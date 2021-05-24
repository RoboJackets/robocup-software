import pytest
import stp.role as role
import stp.skill as skill


class ITestSkill1(skill.ISkill):
    def __init__(self, name: str):
        self.name = name

    def define(self):
        pass

    def tick(self) -> None:
        pass


class ITestSkill2(skill.ISkill):
    def __init__(self, name: str):
        self.name = name

    def define(self):
        pass

    def tick(self) -> None:
        pass


class Skill1A(ITestSkill1):
    ...


class Skill1B(ITestSkill1):
    ...


class Skill2A(ITestSkill2):
    ...


class Skill2B(ITestSkill2):
    ...
