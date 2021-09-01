"""This module contains the Stub skill for ISkill."""

import stp.skill as skill

class Stub(skill.ISkill):
    """Stub skill that does nothing."""

    def tick (self) -> None:
        pass
