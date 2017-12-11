# Subbehaviors {#t20173}


# Questions about Plays?

-   Topics Covered:
    -   Python
    -   State Machines
    -   Play/Tactic/Skill Structure
-   Last meeting was pretty dense
    -   RoboCup is complicated
    -   If you got it, great
    -   If you didn't, great
        -   We need you to ask questions when you don't understand something
-   And today, we're going into&#x2026;


## Additional Information

-   See the previous slide deck and docs for more information, they should be a pretty comprehensive overview of our play system.


# Subbehaviors!

-   A behavior is a generic name for any skill, play, or tactic.
-   Last meeting we learned about Plays/Tactics/Skills
    -   How do we put them together?
-   Subbehaviors allow you to reuse our behaviors in many plays.
    -   Example: We have *one* move play, used by kicker tactics, passing tactics, goalie, etc.


## Examples of Subbehaviors

-   Offense Play contains Passing Tactics, and Move Skills
-   Passing Tactic Contains Move and Kick Skills


## *Behavior* Tree!

```text
Basic122::running
    Mark::running[robot=4]
	move(0.767296, 6.51286)
	endVelocity(0, 0)
	face(0.160718, 9.15305)
    PivotKick::capturing
	Capture::course_approach[robot=2]
	    face(0.160718, 9.15305)
	    avoidBall(-1)
	    move(0.176939, 9.114)
	    endVelocity(0, 0)
    Mark::running[robot=1]
	move(-0.829012, 6.03814)
	endVelocity(0, 0)
	face(0.160718, 9.15305)
```


### Details

-   Here we have a tree of all the behaviors running
-   We have a basic122 play running
-   It is running 2 Marks, and a PivotKick in this frame
-   The marks are directly running robot commands, like move, and face.
-   The PivotKick is running a capture skill, which is running robot commands, like face, move, etc.


## *Behavior* Tree! (cont.)

```text
Basic122::running
    Defense::defending
	SubmissiveDefender::marking
	    Move::completed[robot=5]
		face(0.160718, 9.15305)
	SubmissiveDefender::marking
	    Move::completed[robot=3]
		face(0.160718, 9.15305)
	SubmissiveGoalie::block
	    Move::running[robot=0]
		face(0.160718, 9.15305)
		move(0.290916, 0.14)
		endVelocity(0, 0)
```


### Details

-   Pretty much same thing here
-   We've got a Defense tactic running, with SubmissiveDefender and Goalie Skills
-   These skills actually run robot commands.


# How do I use subbehaviors?


## Adding Subbehaviors

```python
self.add_subbehavior(skill_object,
		     "Name of Subbehavior",
		     required=False or True
		     priority=10) # A higher number is higher priority
```

-   If adding a `complex behavior` or `single robot complex behavior`, don't pass in values for `required` or `priority`


### Details

-   For more docs on this entire section see [this link.](https://robojackets.github.io/robocup-software/classgameplay_1_1single__robot__composite__behavior_1_1_single_robot_composite_behavior.html)
-   These subbehaviors show up in the behavior tree when you run your program.
-   This can be extremely useful when debugging state transitions or subbehavior assignments.


## Removing Subbehaviors

```python
self.remove_subbehavior('string name')

self.remove_all_subbehaviors()
```


## Getting Subbehavior Plays

```python
a_subbheavior = self.subbehavior_with_name('string name')
```


# Real Examples


## CoordinatedPass Tactic

```python
def on_enter_running(self):
    receiver = skills.pass_receive.PassReceive()
    receiver.receive_point = self.receive_point
    self.add_subbehavior(receiver,
			 'receiver',
			 required=self.receiver_required)

def on_exit_running(self):
    self.remove_subbehavior('receiver')
```


## Line Up Tactic

-   First State Machine is set up, then:

```python
# Triggered whenever the line changes
self.remove_all_subbehaviors()
for i in range(6):
    pt = self._line.get_pt(0) + (self.diff * float(i))
    self.add_subbehavior(
	skills.move.Move(pt),
	name="robot" + str(i),
	required=False,
	priority=6 - i)
```

```python
def execute_running(self):
    for i in range(6):
	pt = self._line.get_pt(0) + (self.diff * float(i))
	self.subbehavior_with_name("robot" + str(i)).pos = pt
```


# RoboCup Pro Tip

-   Find some code doing something like what you want
-   Tweak it until it works
-   It's less effective than working everything out, but it's great for beginners!


# Assignment

-   Create a Triangle Pass Play
-   Move 3 Robots into a triangle formation, and pass between them.
-   Starter code is in `soccer/gameplay/plays/skel/triangle_pass.py` (same as last time).
-   Move it to `soccer/gameplay/plays/testing/triangle_pass.py` to begin.


## Tips

1.  Use Move Skills to move your robots to the triangle initially
2.  Use the CoordinatedPass Tactic to pass between, setting receive points as sides of the triangle
3.  At a bare minimum, I would make setup and passing states. You may want to have a state for every side of the triangle (or not).