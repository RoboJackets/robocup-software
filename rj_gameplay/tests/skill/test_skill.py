import pytest

import stp.skill as skill
import stp.role as role


class ITestSkill1(skill.ISkill):
    def __init__(self, name: str):
        self.name = name

    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
        pass


class ITestSkill2(skill.ISkill):
    def __init__(self, name: str):
        self.name = name

    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
        pass


class Skill1A(ITestSkill1):
    ...


class Skill1B(ITestSkill1):
    ...


class Skill2A(ITestSkill2):
    ...


class Skill2B(ITestSkill2):
    ...


def test_skill_registry() -> None:
    """ Tests basic functionality of skill.Registry.
    """
    registry = skill.Registry()

    skill_a1 = Skill1A("test_skill_a1")
    skill_a2 = Skill1A("test_skill_a2")
    skill_b1 = Skill1A("test_skill_b1")
    skill_b2 = Skill1A("test_skill_b2")

    assert ITestSkill1 not in registry
    assert ITestSkill2 not in registry

    with pytest.raises(KeyError):
        _: ITestSkill1 = registry[ITestSkill1]

    with pytest.raises(KeyError):
        _: ITestSkill2 = registry[ITestSkill2]

    registry[ITestSkill1] = skill_a1
    assert registry[ITestSkill1] == skill_a1
    assert registry[ITestSkill1] != skill_a2
    assert registry[ITestSkill1] != skill_b1
    assert registry[ITestSkill1] != skill_b2
    assert ITestSkill1 in registry
    assert ITestSkill2 not in registry

    registry[ITestSkill1] = skill_a2
    assert registry[ITestSkill1] != skill_a1
    assert registry[ITestSkill1] == skill_a2
    assert registry[ITestSkill1] != skill_b1
    assert registry[ITestSkill1] != skill_b2
    assert ITestSkill1 in registry
    assert ITestSkill2 not in registry

    del registry[ITestSkill1]
    assert ITestSkill1 not in registry
    assert ITestSkill2 not in registry

    with pytest.raises(KeyError):
        _: ITestSkill1 = registry[ITestSkill1]

    with pytest.raises(KeyError):
        _: ITestSkill2 = registry[ITestSkill2]


def test_skill_factory() -> None:
    """ Tests basic functionality of skill.Factory.
    """
    # Create the registry.
    registry = skill.Registry()

    skill_1a = Skill1A("skill_1a")
    skill_2b = Skill2B("skill_2b")

    registry[ITestSkill1] = skill_1a
    registry[ITestSkill2] = skill_2b

    # Create the factory.
    factory = skill.Factory(registry)

    assert factory.create(ITestSkill1) == skill_1a
    assert factory.create(ITestSkill2) == skill_2b
