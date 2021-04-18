import stp.role as role
import stp.skill as skill
import stp.tactic as tactic

class BaseSkill(skill.ISkill):
    def __init__(self, name: str):
        self.name = name

    def define(self):
        pass

    def tick(self) -> None:
        pass


class ISkillA(BaseSkill):
    ...


class ISkillB(BaseSkill):
    ...


class ISkillC(BaseSkill):
    ...


class SkillA(ISkillA):
    ...


class SkillB(ISkillB):
    ...


class SkillC(ISkillC):
    ...
