from typing import List, Set, Optional, Tuple, Type, TypeVar, Callable
import gameplay.robocup as robocup
import role
import gameplay.main as main

def role_assign(role_requests: List[RoleRequest]) -> None:
    unassigned_bots = main.our_robots()
    requests = sorted(role_requests, key = lambda: request, requests.get_priority().value)
    for request in requests:
        request.set_last_role()
        best_bot = None
        best_bot_cost = float('inf')
        if unassigned_bots:
            for bot in unassigned_bots:
                if (request.constriants(bot)):
                    bot_cost = request.cost(bot)
                    if (bot_cost < best_bot_cost):
                        best_bot = bot
                        best_bot_cost = bot_cost
            request.assign(best_bot)
            unassigned_bots.remove(best_bot)
        else:
            request.unassign()

