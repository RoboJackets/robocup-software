# Skills, Tactics, Plays {#t20172}


# Python Overview


## Using Python

```shell
$ python3
Python 3.5.3 (default, Jan 19 2017, 14:11:04)
[GCC 6.3.0 20170118] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> print("Hello RoboCup!")
Hello RoboCup!
```


## Basic Syntax

```python
# Hello world
print("Hello World!")

# Set a variable
myinteger = 1
mydouble = 1.0
mystr = "one"

print(myinteger)
print(mydouble)
print(mystr)
```

    Hello World!
    1
    1.0
    one


## IF Statements and Conditionals

```python
if True:
    print("RoboCup is great!")
if False:
    print("IGVC is great...")

if True:
    print("RoboCup is the best!")
elif True:
    print("RoboRacing is the best...")
else:
    print("IGVC is the best...")

a = 5
if a > 2:
    print("a is greater than 2!")
if a > 2 and a < 10:
    print("a is > 2 and < 10")
print("a is greater than 3" if a > 3 else "a is less than 3")
print("a is greater than 9" if a > 9 else "a is less than 9")
```

    RoboCup is great!
    RoboCup is the best!
    a is greater than 2!
    a is > 2 and < 10
    a is greater than 3
    a is less than 9


## Loops and range()

```python
a = 10
while a > 0:
    print(a, end=" ")
    a = a - 1
print("")

print(list(range(10)))
for i in range(10):
    print(i * i, end=" ")
print("")
```

    10 9 8 7 6 5 4 3 2 1 
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    0 1 4 9 16 25 36 49 64 81 


## Functions

```python
def func1(a, b):
    return a + b

print(func1(2, 4))

# Lambda Expressions ('advanced' topic)
def secret_func():
    return "RoboCup!"
def welcome(target):
    return "Welcome to " + target()

print(welcome(secret_func))
print(welcome(lambda: "RoboJackets!"))
# Lambda with arguments
# print(welcome(lambda arg1, arg2: "RoboJackets!"))
```

    6
    Welcome to RoboCup!
    Welcome to RoboJackets!


## Object Oriented Python

```python
class BaseClass():
    globalVar = 1
    # Constructor
    def __init__(self):
	print("Initializing a Base")
	self.local_var = 2

class ChildClass(BaseClass):
    def __init__(self):
	super().__init__() # call superclass constructor
	print("Initializing Child")
	print("Our local var is: " + str(self.local_var))

# When defining class methods, pass 'self' as the first arg.
# Self refers to the current class instance.
a = BaseClass()
print("---")
a = ChildClass()
```

    Initializing a Base
    ---
    Initializing a Base
    Initializing Child
    Our local var is: 2


## Additional Python Resources

-   [lightbot](http://lightbot.com/hocflash.html) - A Introduction to Programming Logic
-   [Python Beginner Hub](https://wiki.python.org/moin/BeginnersGuide/NonProgrammers)
-   [Python Syntax Overview](https://learnxinyminutes.com/docs/python/)
-   [A intro to python](http://thepythonguru.com/)


# State Machines


## What are they?

-   A state machine is a series of states
-   You can transition between them
-   A state could have multiple transition
-   A state transition only occurs if a condition is fulfilled


### Examples

-   A car engine is a state machine, each piston going between different internal states to move the car forward
-   A washing machine is a state machine, going between different states to cycle between wash, dry, etc.
-   [Wikipedia Page on State Machines](https://en.wikipedia.org/wiki/Finite-state_machine)


## Elevator

![img](https://i.imgur.com/KPv5sSk.png)


## Move

![img](http://i.imgur.com/gmIcPGq.png)


### Details

-   Every Play starts in a 'start' state
-   Most plays will instantly transition into a running state (in this case `behavior::running`)
-   This particular play will go into `behavior::completed` once we reach a target position
-   However, if we are ever bumped out of place, we are put back into the running state (to continue moving)
-   Another thing to notice here is that every state here is a `behavior::<thing>` state.
    -   These states are created by the state machine machinery we have set up.
    -   They are used to determine whether a state can be killed or not, or if it is waiting for something
    -   Most of the action will be done in a subclass of `bheavior::running` or `behavior::running` itself if you have a simple class.


## Pass Receive

![img](http://i.imgur.com/HAhoMC1.png)


### Details

-   This example is a bit more complicated, as we have multiple `running` states
-   Each one of these substates are classified as running by our machinery, since they subclass behavior::running
-   A brief explanation is: if we are running, and the ball is ever kicked, immediately receive, but if we have some time, try to align yourself properly at the designated receive coordinate.


## Passing

![img](http://i.imgur.com/OhWnSwT.png)


### Details

-   Here we have more running substates
-   A pass is fairly linear, as it has preparing -> kicking -> receiving states
-   However, if we 'timeout' in the preparing or kicking states, we fail the current behavior
    -   This can happen if our robot is ever stuck


## Additional Information on State Machines

-   While you *do not* need to know advanced state machine ideas, you need to be comfortable working with and parsing existing state machines from a diagram or from our code.
-   [Wikipedia Article](https://en.wikipedia.org/wiki/Finite-state_machine)
-   [A quick block post about state machines](http://blog.markshead.com/869/state-machines-computer-science/)
-   [You might be using state machines in a hacky way already&#x2026;](https://engineering.shopify.com/17488160-why-developers-should-be-force-fed-state-machines)
-   [Our Current State Machine Implementation](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/fsm.py)


# Plays, Tactics, Skills

-   A Basic Unit in our AI
-   Only one Play can run at a time


## Skill

-   Involves only *one* robot
-   Extremely basic building blocks
-   Examples
    -   Move
    -   Kick
    -   Face a direction
    -   Capture the ball
-   Located in `soccer/gameplay/skills/`


## Tactics

-   Involves multiple robots
-   Contains skills
-   Can contain unique behavior (but usually not)
-   Examples
    -   Pass
    -   Defend
    -   Line Up
-   Located in `soccer/gameplay/tactics/`


## Plays

-   Only one can run
-   Contains tactics
-   Examples
    -   Basic122 (basic offense)
    -   Two side attack (basic offense)
    -   Stopped Play
    -   Line Up
    -   Corner Kick
-   Located in `soccer/gameplay/plays/*/`


### Notes

-   Only plays are actually runnable in our model
    -   If you want to run a tactic, make a dummy play that runs that tactic on startup
-   For now, we'll only look at plays to keep things simple (maybe we'll get more complex later)


# Basic Plays and State Machines

-   Every Play is a State Machine as well!
-   Plays use State Machines to tell them what to do
-   This is a good thing, since we can have very complex behavior in a play


## Defining States

```python
# First create a state Enum (An enum is just a group of names)
class OurState(enum.Enum):
    start = 0
    processing = 1
    terminated = 2

# Then, register your states in our state machine class
# You must be in a play/tactic/skill for this to work
self.add_state(PlayName.OurState.start,
	       # This is the superclass of our state. Most of the time,
	       # this is 'running' (see below)
	       behavior.Behavior.State.start)

self.add_state(PlayName.OurState.processing,
	       behavior.Behavior.State.running)

self.add_state(PlayName.OurState.terminated,
	       behavior.Behavior.State.completed)

```


## Defining Transitions

```python
self.add_transition(
    # Start state for this transition
    behavior.Behavior.State.start,
    # End state for this transition
    PlayName.OurState.processing,
    # Condition for this transition (Replace 'True' with a conditional)
    lambda: True,
    # Documentation String
    'immediately')
```


## Defining Actions to Take In A State

```python
# Assuming we have the PlayName.OurState.processing state

# Action taken when entering this state
def on_enter_processing(self):
    print("We have begun our processing")

# Action taken every frame we are in the processing state
def execute_processing(self):
    print("Processing is Ongoing")

# Action taken when we exit the processing state
def on_exit_processing(self):
    print("Processing is Completed!")
```


# Your Assignment

-   Create a play that prints out which half of the field the ball is currently in
-   EX: Print out "TopHalf" when in the top half of the field, and "BottomHalf" otherwise.
-   Use state machines to print this out ONLY ON A TRANSITION. (Don't simply print out every frame)
-   Extra Credit: Can you come up with another cool thing to do with state machines?


## Tips

-   The field coordinates start at 0, 0; Which is our Goal.
-   Field Size Docs: ([http://bit.ly/2cLsUBL](http://bit.ly/2cLsUBL))
-   Ball Position Docs: ([http://bit.ly/2damxXA](http://bit.ly/2damxXA))
-   Move the template starter from `soccer/gameplay/plays/skel/which_half.py` to `soccer/gameplay/plays/testing`
-   Start by just printing the Y coordinate of the ball and work up from there


## Useful Tools

```python
# Gets the y position of the ball
main.ball().pos.y
# Gets the field length in meters
constants.Field.Length
```


## Exercise Details

-   [Link to Starter File](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/plays/skel/which_half.py)
-   Ask on [gitter](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/plays/skel/which_half.py) for help and answers!


## Answers

-   [Which Half Answers](https://github.com/RoboJackets/robocup-software/blob/master/soccer/gameplay/plays/testing/which_half.py)