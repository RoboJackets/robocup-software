
# Gameplay

This document covers the basics of our high-level soccer code including Plays, Behaviors, and game state evaluation.
This code resides in [soccer/gameplay](https://github.com/RoboJackets/robocup-software/tree/master/soccer/gameplay).

As with any complex system, it's important to have some well-defined structure to keep things manageable.
The `soccer` program is split up into several different layers for this purpose, with the gameplay layer being the most high-level.
The gameplay layer is managed by the \ref GameplayModule and evaluates the current state of the field (where the robots and ball are) and the state of the game (is it a kickoff, penalty kick, etc).
This information is contained in the c++ \ref SystemState "SystemState" class and the \ref GameState "GameState class", respectively.
The result of running the \ref GameplayModule is a motion command for each of the robots as well as kick and dribble commands.
Layers of the software stack below the gameplay layer ideally don't know anything about soccer and just orchestrate robot motion, radio communication, network communication, etc.

When the gameplay module is running, its job is to select the best play from a list of enabled plays by choosing the one with the lowest \ref gameplay.play.Play.score() "score()" value.
Plays are enabled and disabled through the GUI with the checkboxes next to play names.
See the annotated screenshot below for more info.


<img src="soccer-with-gameplay-annotations.png" width="1000"></img>

## Play Structure

The high-level strategy code is organized to be as modular as possible.
To do this, it's been split up into three main parts: [Skills](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/skills), [Tactics](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/tactics), and [Plays](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/plays).
There is one Goalie (optionally) and one \ref gameplay.play.Play "Play" object.

**Skills** are behaviors that apply to a single robot.
They include things like \ref gameplay.skills.capture.Capture "capturing the ball", \ref gameplay.skills.move.Move "moving to a particular position on the field", and \ref gameplay.skills.pivot_kick.PivotKick "kicking the ball".

**Tactics** can coordinate a single robot or many and generally encapsulate more complex behavior than **skills**.
This includes things such as \ref gameplay.tactics.coordinated_pass.CoordinatedPass "passing", \ref gameplay.tactics.defense.Defense "defense", and \ref gameplay.tactics.positions.goalie.Goalie "the goalie".

**Plays** are responsible for coordinating the whole team of robots (although some robots may be unused).
At a given time, the `soccer` program is running at most one play.

Used together, skills, tactics, and plays form a tree structure with the Play at the root and other behaviors below it.
The C++ `GameplayModule` tells the current play to run, which in turn tells each of its sub-behaviors to run.


## Gameplay structure

Every behavior in soccer is a state machine that subclasses the main \ref gameplay.fsm.StateMachine "StateMachine class".
This class has methods for adding states and transitions between them as well as utility methods for showing textual and graphical descriptions of a state machine and it's sub-machines.
One nifty usage of this feature is that we can easily view a diagram of every skill, tactic, and play in our library.

~~~{.sh}
# Run this in a terminal in the robocup-software folder to make the diagrams
$ make behavior-diagrams
~~~

After running the above command, open up the `soccer/gameplay/diagrams` folder and browse around to see a diagram for each behavior.
Below is the state diagram for the \ref gameplay.skills.pivot_kick.PivotKick "PivotKick" behavior.
A good exercise if you're new to writing plays is to compare the PivotKick __init__() method's state machine declarations to what you see in the diagram below.

![PivotKick state diagram](PivotKick-state-diagram.png)


## Creating a Play

Making a new play is as simple as adding a new python file somewhere inside the `soccer/gameplay/plays` directory and creating a subclass of `Play` inside of it.
There is no need to register the play, soccer will see the file in that folder and display it in the Plays tab in `soccer`.
Generally when writing a new play, it's a good idea to base its initial structure on an existing play.
A good example play to look at is the \ref gameplay.plays.testing.line_up "LineUp play".

Every play begins by declaring a python class that subclasses the Play class:

~~~{.py}
import play

class MyNewPlay(play.Play):
    def __init__(self):
        # call superclass constructor
        super().__init__(continuous=False)

        # TODO: declare states and transitions if needed
        #       see fsm.py for more info on these methods.

        # Most plays transition from Start to Running right away
        self.add_transition(behavior.Behavior.State.start,
                behavior.Behavior.State.running,
                lambda: True,
                'immediately')
~~~

After declaring the play, it's time to add in the appropriate states and state transitions to your play.
Every subclass of \ref gameplay.behavior.Behavior "the Behavior class" automatically inherits some pre-defined states including `Start`, `Running`, and `Completed` and is initially started in the `Start` state.
It's your job as the writer of a new play to define a state transition from `Start` to `Running` or a substate of `Running`.

The gameplay system automatically declares three methods for every state added to a behavior: on_enter_<NAME>, on_exit_<NAME>, execute_<NAME>.
Where <NAME> is the name of the state.
This allows us to conveniently execute code whenever we transition states or have code run repeatedly while we're in the state.

An incredibly simple example of a play that just moves a robot to a certain position on the field could be implemented as follows:

~~~{.py}
import play
import skills.move
import robocup

class MoveOneRobot(play.Play):
    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                behavior.Behavior.State.running,
                lambda: True,
                'immediately')

    def on_enter_running(self):
        # Add a "Move" subbehavior that tells a robot to a specified (x, y) location
        m = skills.move.Move(robocup.Point(0, 2))
        self.add_subbehavior(m, name='move', required=True)

    def on_exit_running(self):
        # When the running state is over, we remove the subbehavior
        self.remove_subbehavior('move')
~~~


## Role Assignment

When writing a play, you are defining a set of actions that should be taken by different robots on the field.
One important thing to note though is that you don't choose which robots will fulfill these roles directly.
Instead, you can define what attributes a robot should have in order for them to be a good fit for the role.
At each iteration of the main run loop, the \ref gameplay.role_assignment "role assignment system" examines the role requirements for each running behavior and uses an optimal matching algorithm (the [hungarian algorithm](https://en.wikipedia.org/wiki/Hungarian_algorithm)) to find the best robot to assign to each role.
In order to implement custom assignment logic for your behavior, you'll need to override the \ref gameplay.behavior.Behavior.role_requirements "role_requirements() method".


## RoboCup python module

We use a 3rd-party library called Boost Python to create an interface between the C++ code that makes up the majority of our `soccer` program and the gameplay system that's written in python.
Boost Python is used to create a python module called "robocup" that python code can *import* in order to access our C++ classes and functions.
The C++ classes and functions available to the python interface are created through "wrappers" in the [robocup-py.cpp](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/robocup-py.cpp) file.
The "robocup" python module is compiled as a part of our project when you run `make` and is placed in the `run` directory as `robocup.so`.
This can be imported like any other python module like so:

~~~{.sh}
cd robocup-software

# Ensure that the latest version of the 'robocup' python module is built
make

# The python module is placed in the 'run' directory as 'robocup.so'
cd run

# Run python interpreter and import the module
python3
import robocup

# Use the help function to see a list of available classes and functions
help(robocup)
~~~


## Visualization

Many plays provide visualizations for the actions they are performing to make it easier for the user to quickly see what's happening.
For example, the Defense tactic draws red triangles from opponent robots and the ball to our goal to help visualize our defense's effective coverage.
This functionality is provided by the many drawing methods provided by the \ref SystemState class.
