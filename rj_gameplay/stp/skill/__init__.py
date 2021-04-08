""" This module contains data structures for the Skills level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Type, TypeVar

import stp.role as role


class ISkill(ABC):
    """ Interface for Skills. """

    @abstractmethod
    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        ...

    def create_requests(self, num_requests: int, **kwargs) -> List[role.RoleRequest]:
        """Creates a list of sane default RoleRequests.
        :param num_requests: Number of role requests to create.
        :return: A list of size num_requests of sane default RoleRequsts.
        """
        return [self.create_request(**kwargs) for _ in range(num_requests)]


