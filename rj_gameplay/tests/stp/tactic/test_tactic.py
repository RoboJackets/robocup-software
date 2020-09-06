import stp.role as role
import stp.skill as skill
import stp.tactic as tactic


class BaseSkill(skill.ISkill):
    def __init__(self, name: str):
        self.name = name

    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
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


class Skills(tactic.SkillsEnum):
    SKILL_A = tactic.SkillEntry(ISkillA)
    SKILL_B = tactic.SkillEntry(ISkillB)
    SKILL_C = tactic.SkillEntry(ISkillC)


def test_skills_enum():
    # Create skill registry.
    skill_registry = skill.Registry()

    skill_a = SkillA("skill_a")
    skill_b = SkillB("skill_b")
    skill_c = SkillC("skill_c")

    skill_registry[ISkillA] = skill_a
    skill_registry[ISkillB] = skill_b
    skill_registry[ISkillC] = skill_c

    # Create the skill factory.
    skill_factory = skill.Factory(skill_registry)

    # Instantiate the skills enum with the skill factory.
    skills = Skills(skill_factory)

    # Check that the skills for each enum have been instantiated correctly.
    assert skills.SKILL_A.skill == skill_a
    assert skills.SKILL_B.skill == skill_b
    assert skills.SKILL_C.skill == skill_c
