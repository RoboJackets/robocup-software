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


4. ROS Basics
-------------

5. ROS and C++
--------------

6. Conclusion
-------------
