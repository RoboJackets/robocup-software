# Role Assignment {#t20174}


# Communications Recap

-   [Gitter](https://gitter.im/RoboJackets/robocup-software)
-   [RoboCup Training Repo](https://github.com/RoboJackets//robocup-training)
    -   Slides
    -   Docs


# Questions about Plays?

-   We're going to slow down a bit after last meeting
    -   **Plays are not the only area in RoboCup**
    -   They are a very high impact thing to cover though.
-   Topics Covered:
    -   Subbehaviors
    -   Creating Plays


## Additional Information

-   See the previous slide deck and docs for more information, they should be a pretty comprehensive overview of our play system.


# How do I get a specific robot?


## Dynamic Assignment

-   Our role assignment system picks the best robot for the task
-   You define your constraints, and we'll give you a robot if we can!
-   It's also some black magic&#x2026;


## But I want Robot X!

-   Override Role Requirements to add your constraints!
-   This next section is a massive oversimplification
-   See `role_assignment.py` for more information.


### Get the closest robot

```python
# From Move
def role_requirements(self):
    #Always call super first, otherwise parents might override your constraints!
    reqs = super().role_requirements()
    # Destination Shape is used for distances to points or segments
    reqs.destination_shape = self.pos
    return reqs
```


### Get the robot with all the hardware

```python
def role_requirements(self):
    reqs = super().role_requirements()
    reqs.require_kicking = True
    # try to be near the ball
    if main.ball().valid:
	reqs.destination_shape = main.ball().pos
    return reqs

```


### I really, really, want MY Robot!

```python
# From Goalie
def role_requirements(self):
    reqs = super().role_requirements()

    # This iteration is needed if we have
    # subbehaviors we want to have applied as well.
    for req in role_assignment \
	.iterate_role_requirements_tree_leaves(reqs):

	req.required_shell_id \
	    = self.shell_id if self.shell_id != None else -1
    return reqs
```


## But how does it choose the robot?

-   Actually Black Magic
-   A peek behind the curtain

```python
class RoleRequirements:
    def __init__(self):
	self.destination_shape = None      #Distance from destination
	self.has_ball = False
	self.chipper_preference_weight = 0 #importance of chipper
	self.required_shell_id = None
	self.previous_shell_id = None      #controls switching of robots
	self.required = False              #Can't be dropped, causes assignment failure if too many
	self.priority = 0                  #Higher is more important
	self.require_kicking = False
	self.robot_change_cost = 1.0
```

-   Every behavior adds one of these

-   Constraints you add modify the cost of that behavior for a specific robot

```python
if req.destination_shape != None:
   cost += req.position_cost_multiplier * req.destination_shape.dist_to(robot.pos)
```


## Docs

-   [These are docs for this section. Check subclass docs as well!](https://robojackets.github.io/robocup-software/classgameplay_1_1behavior_1_1_behavior.html)


# Capture Overview

-   Capture is a complicated behavior that utilizes role assignment


## Let's Look at Code!

-   <https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/skills/capture.py>


# Wrapping Up


# Assignment

-   Implement role assignment in Triangle Pass
-   If you missed last meeting, just do Triangle Pass instead


## Recomended Requirements

-   Closest robots to center-field
-   Closest robots to sideline
-   Closest robots to ball


## Tips

-   Check out the skills and tactics behaviors for more examples