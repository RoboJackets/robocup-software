Tutorial
========

This page is meant to teach new RoboCup Software members the basics of what
they'll need to contribute to the team. It will introduce the Robot Operating
System (ROS), the command-line, GitHub, git, Python, C++, and the general shape
of our stack. Upon completion of this tutorial, you'll have the knowledge and
trust of your teammates to implement new features on your own.

No prior experience is assumed. However, a bit of stubbornness is required.
RoboCup SW has seen many members without any prior CS experience become valued
contributors, and many talented CS majors quit within a few weeks.

The tutorial is structured as follows:

.. contents::

There are some gaps intentionally left in the tutorial. This is to force you to
problem-solve on your own, simulating what it feels like to write a new
feature. If the tutorial was simply a bulleted list of commands to type, it
would not prove that you're ready to work on something meaningful on your own.

When you run into issues, your order of question-asking should be:

#. Google

   * Keywords, not full sentences

   * Error messages, if they come up

#. Google

   * Seriously

#. Fellow new members

#. Software lead

#. Anyone the SW lead takes advice from

This is not because older members don't want to help you, but because if older
members helped every new member with every question, they wouldn't have time to
make our robots better (nor would you learn as much). So try to resolve your
issue yourself, and expect to be asked "what have you tried already?" if you ask
for help.

0. Command-Line Basics
----------------------

If you've never heard of or used the command-line before, this website is
lengthy, but wonderful for beginners:

https://ubuntu.com/tutorials/command-line-for-beginners#1-overview

The rest of this tutorial assumes you have working knowledge of the
command-line: how to run an executable, change directories, move files, run
commands. So if you're uncomfortable with any of that, go through and follow
the exercises in the site above.

Two indispensable tips not mentioned in that link:
 * ``man [command]`` will pull up a manpage, which is an explanation of the
   command and all of its options. This usually only works on standarad Unix
   commands. For instance, you can find words in any file in a directory using
   ``grep``: try ``man grep`` to see its full potential.
 * ``[command or executable] --help`` will almost always return a prompt that
   tells you what the command does, and how you can modify it with options.
   Many custom command-line tools will have a --help output, if they don't have
   a man page.

1. Installation
---------------

See "Getting Started". That page will assume you have the Command-Line Basics
from above, as well as a working knowledge of Git (which you can get either
`online <https://rogerdudler.github.io/git-guide/>`_ or from "Contributing").

2. GitHub Basics
----------------

Now that you have everything installed, understand basic command-line
usage, and have at least seen git before, let's get started using
GitHub.

.. Note::

   git is a command line version-control tool. GitHub is a website to host
   shared files, and is well-integrated with git.

First, use git to create a new branch under this naming scheme: 

.. code-block:: bash

   git checkout -b "<your name>/robocup-sw-tutorial"

Then take a look at the defense play in
``rj_gameplay/rj_gameplay/play/defense.py``. Launch soccer (our UI) and the
ER-force simulator, then select this play as the test play to see it in action.
You should see 3 robots form a wall, 2 robots mark the opposing team, and 1
robot play goalie. 

Figure out which line(s) to change so that 4 robots form a wall instead of 3.
When done, take a screenshot of the four wallers; we'll use this later.

Now that you've made a change to the repo, run ``git status``. You should see
that whatever files you changed show up in red, which indicates that they are
unstaged. Stage the files you changed with ``git add`` (Google this if unsure!),
then commit them like so:

.. code-block:: bash

   git commit -m '<commit msg>'

.. note::

   <commit msg> should be a present-tense description of what you've changed.
   In this case, "change num wallers to 4" is fine.

   Without -m, git commit will open a nano editor instance, and ask you to type
   in a commit msg. -m is a bit faster.

When you commit, you should see our pre-commit hooks run. These are automated
programs that make your code comply with standardized style guidelines. If one
of the checks fails, simply re-add and re-commit and they should pass.

Now that you've committed, run ``git push`` to push your changes to the remote
server. This is how GitHub sees and displays your changes. If you run into any
errors at this step, read the error logs carefully (they often tell you what to
do), and Google if needed.

Finally, go to our GitHub page, click the "Pull Requests" tab, and create a new
draft pull request for your branch. When it asks you to fill in the PR
description, you can delete the whole template and write something simple like
"Completes RC SW tutorials". Add that screenshot of your four-waller setup as a
comment below your brand new PR. Nice work!


3. rj_gameplay and Python
-------------------------


4. ROS CLI Basics
-----------------

This section is our variation of the ROS 2 `"Beginner: CLI Tools" tutorials
<https://docs.ros.org/en/foxy/Tutorials.html#beginner-cli-tools>`_. We do
things slightly differently (and don't use all of the ROS 2 features described
in those tutorials), so this is intended to supplement those docs.

Before we get started, read all of the short "Background" sections for these pages:
 * Understanding ROS 2 nodes
 * Understanding ROS 2 topics
 * Understanding ROS 2 services
 * Understanding ROS 2 parameters
 * Understanding ROS 2 actions

These sections put together are only a couple hundred words, and contain very
neat animated diagrams that we can't recreate here.

Now that you have some background on what ROS is and how it works, let's
explore how we use ROS in our stack. (ROS is used in place of ROS 2 in the rest
of these docs, just know that we are referencing ROS 2 every time.)

First, open up our stack, same as you did in the installation guide. Then run

.. code-block::

   ros2 topic list

to see the list of topics. Let's look at what robot 0 is thinking. Run

.. code-block::

   ros2 topic echo /gameplay/robot_intent/robot_0

to see what's being published to that topic. You should see that robot 0 is
being given a motion_command to go to a certain position at a certain angle.
Feel free to try echoing other topics to see what they're publishing.

Now run ``ros2 topic info`` on the same topic to see what message type that
topic is publishing, and how many publishers and subscribers are listening to
it. For this topic, the message type is a subset of ``rj_msgs/``, which means
we wrote our own custom .msg file that this topic uses.

Your task for this section is to find the file that defines the message type
used by ``/gameplay/robot_intent/robot_0``. This will take you a long time if
you search for it manually and almost no time if you use a tool like ``find``.
Once you have the right file, screenshot it and add it to your GitHub PR as a
comment. Congrats! You now have a working understanding of ROS CLI tools.

5. rqt Basics
-----------------

The observant among you may have noticed that the last section only covered ROS
topics, even though it asked you to read about ROS nodes, services, parameters,
and actions as well. This was to set up the need to use ``rqt``, a graphical
interface for the many tools ROS includes.

To use it, open a new terminal, source ROS (like you do before running our
stack), and run ``rqt``. (This should have been installed with the rest of the
stack when you ran ``./util/ubuntu-setup``; if not, see `this guide
<http://wiki.ros.org/rqt/UserGuide/Install/Groovy>_`.)

The rqt window should pop up, like this:

// TODO: Kevin, add screenshot here

To replicate what we did in the last section, go to the top, click Plugins,
then find the Topic Monitor and enable it. This allows you to see both a list
of all topics, and see what messages are being published to them (by clicking
the checkbox).

Now launch the Node Graph. You should see a large, complex node diagram pop up.
Don't fret--most of the nodes are actually just duplicated across robot
numbers. (For instance, every robot has its own robot_intent, which you may
have noticed in the last section.) Hover over the topic that contains robot 0's
robot intent and figure out which nodes publish and subscribe to it, then post
your answer as a GitHub comment.

<see 8c>
<see 8d>


5. ROS and C++
--------------



6. Conclusion
-------------
