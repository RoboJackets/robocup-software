# Contributing Guide
This guide targets new members looking for a guide on contributing to the
project. This guide will focus on the overall picture of effective coding and
contributions. It is not intended to be a technical guide.

## Initial Notes
Contributing to open source presents a unique set of challenges, especially when
dealing with large projects. Our project is approximately 50-80k lines of code,
and is still very small by some open source standards. 

This should not be discouraging, but rather informative. Most initial
contributions will be small; it takes time to learn the project structure and any
languages and skills each particular project requires. Contributions of any size
will be greatly appreciated, and you should have realistic expectations of what
a first contribution might be (if you dont believe me, you can look at my first
contribution [here](https://github.com/RoboJackets/robocup-software/commit/6ce98fc0f8d88b6d145700779e126c0f1b99bb92#diff-71a3477f37bd5b20744e292eda2e3fbc). It's two lines of 'code'). This guide will help you learn to navigate our code base, and 
work toward an initial contribution.

## Initial Prerequisites
Before starting, you'll need a Unix-like environment. This means you need to be 
running OSX or Ubuntu Linux (other flavors of Debian may work, but we do not
officially support them or FedoraCore).

You'll also need a Github account, which you can create [here](http://github.com).
Github is a web front-end for a program called Git, which allows multiple people to
work on and contrubute to the same code base, at the same time.

## Git
We use Git as our version control system (if you already know git, you can skip
ahead). A Version control system allows many people to code for the same project 
at the same time.

To ease new contributors into Git, I'll repeatedly use the analogy of the 
classroom test.

If you simply want to learn about the workflow we use, and are less interested
in learning the relationships between Git elements, you can skip to the "Overall
Workflow" section.

### Master
If Git is like a test, then you can think of the master branch as the final copy
you submit for grading. This copy should have the correct answer, should contain
the most effective or efficient solution, and should be highly neat and readable
for the graders. You probably shouldn't do any work here, but rather should
explore in other locations such as scrap paper. Once you have several solutions,
you can pick the one you like the best, and more neatly copy the work on to the
test you will hand in.

Our master branch can be found [here](https://github.com/RoboJackets/robocup-software).
The code in master is always neat and untouched. It can always compile. When you
first clone our codebase from git to view the simulator and soccer, you are
using the code in master. It is in all respects, the master copy from which all
other contributions are derived. Even if you are eventually given permission to
write to master, you should never do so.

### Forks
If the professor puts test questions on the whiteboard for the whole class, you
would not be allowed to solve the problems on the board; you would be expected
to copy any relevant information to your own paper and solve the problems there.
This gives you the freedom to play around with rephrasing and solving questions,
without distuirbing others.

Forking a repository on Github duplicates the project, but you are given full
write access to your own duplicate. Now you can delete, recreate, and add code
relevent to your contribution without harming the progress of others. This 
duplicate is known as your repository and every team member has a fork. This is 
different from the main repository which belongs to the team rather than an 
individual. Don't confuse the main repository with the master branch. The
shared, main repository can have incomplete features being worked on by 
everyone. The master in your fork and the master in the main repository should
always remain clean.

This also works well when you want a feature that only some people want. You 
don't have to move every contribution from your fork back into the main 
repository. When you have contributed something and want it placed in master, 
whoever is reviewing your code will look at your fork of the code and compare 
it to the master branch.

### Branches
Branches are like pieces of scrap paper. You can use them to organize your work 
and solutions to the test questions. You should not have work regarding 
different problems mixed across several pieces of paper; you may get your 
progress confused. You should use one (or several) peices of paper for each
problem you are trying to solve, but should never use one sheet for multiple
questions.

For RoboCub you should create a new branch for every new item you'd like to work
on and for every bug or issue you have to fix. This ensures your master branch
stays clean. You should never solve more than issue at a time and you should
never have changes or additions for multiple things in the same branch. 

### Remotes
If Git is like a test, then remotes would be cheating. A remote allows you to
view the solution(s) of another classmate, and pull those additions into your 
repostory as if they were on the classroom whiteboard.

In software this can be particularly useful if a team member is working on some
new code that may not be perfect yet, but isn't ready to be folded into master. 
This may happen when you cannot continue work without getting the somewhat 
related progress from someone else. You should understand that when pulling from 
someone else, they take no responsibility for any problems their updates may
cause for you. This is a decently advanced concept for those new to distributed 
environments, and won't be used too often. We encourage you learn more about 
this independently if interested. 

### Overall Workflow (important)
If you read the previous sections, you may be a little overwhelmed. This section
will describe how these elements interact to form a coherent workflow that will 
allow you to make contributions more easily.

Ensure you have a fork of the main repository and that you've cloned it onto
your desktop. 

You now have a copy of your repository's master branch avaliable to you. When
you have an idea of what you'd like to contribute, create a new branch before
starting work. Assume you want to write some radio firmware, so you name your
branch radioFirmware.

Your new branch contains a copy of the content of master. Make your additions 
and edits now, they will only affect the radioFirmware branch. When done add 
and commit the files.

You now have a branch with your contribution, but you haven't contributed until
the code makes it into the main repository. This involves several steps. First,
any changes others have made in the team's repository need to be merged into 
your code. If there are any conflics Git can't resolve automatically, it is 
your job to to solve those errors. By merging changes into your contribution, 
rather than the other way around, you ensure the act of bringing your code into 
the team's repository will go smoothly. This helps when another member of the 
team reviews your code as well.

Now that you have a merged branch, you should push the branch to your Github. 
From Github, you can make a pull request from your repository against the 
team's repository. This will notify an older team member that you are ready to
have your contribution reviewed. Requirements for pull request standards are 
listed in several sections below. The older team member may ask that you fix 
or touch up some things before the request is accepted. This is normal and 
common. Once the pull request meets standards, the older member will approve 
it, and you changes will be complete.

Keep in mind, you can have several branches at once. If you need to fix a bug 
for an existing contribution while working on a new one, you should checkout 
the master branch, and then create a new branch named bug fix. It is 
critically important that a pull request only address one thing at a time. If it 
does not, the request will not accepted until you have separated the items you 
have worked on. 

If you've done all this successfully, you are now an official contributor.

#### Example (with technical details)
Here we will work through a very possible scenario that may arise while 
contributing to the project. At this point, you should have created a Github 
account and forked the main RoboCup repository. You should also look at 
creating a ssh key for Github [here](https://help.github.com/articles/generating-ssh-keys/). 

1. Clone your repository.
2. You've decided to write some radio firmware. Create a new branch for radio
development using `git checkout -b radioFirmware`. You will automatically be
switched to the new branch,
3. Start reasearching and coding.
4. A bug in the path planning code has surfaced and the team wants you to try 
to fix it. You're still on the radioFirmware branch, but you should never work 
on more than one feature per branch. Return to the master branch using 
`git checkout master`. Now create a new branch for the bug fix like so 
`git checkout -b pathPlanningHotfix`.
5. Fix the buggy code.
6. Commit, push, and submit a pull request for the bug fix.
7. Switch back to the radioFirmware branch with `git checkout radioFirmware`. 
You can now (optionally) delete the pathPlanningHotfix branch once the pull 
request has been accepted.
8. Continue radio firmware development. If any more urgent problems arises, 
you can repeat steps 4-7.
9. Push the new radio firmware and submit a pull request.

## Pull Request Requirements
With large projects, originization and structure can break down fairly quickly 
resulting in some bad spaghetti code. We want to avoid this as much as 
possible. For this reason, every pull request will be reviewed by a more senior
member of the team, currently Justin Buchanan. If your request isn't accepted 
right away, don't take it personally. Often your code may work fine, but there 
are things you can add or refine. This helps keep our repository clean and will 
give you valuable experience participating in a code review process.

### Continuous Integration
Continuous Integration (CI) is a tool to help auto-detect problems before they 
are merged into the main respository and have a chance to cause problems. Every 
time you submit a pull request, the CI tool is run and it will assign a passing 
or failing mark to the request. If the CI fails, you will need to fix the error 
in your code before the code review. If the reason for the failure isn't 
obvious or it's a problem in the CI check itself, seek some help.

### Content 
It's important that the content of a pull request be kept clean and small. Pull
requests should be less than 1-2k lines of code. The code changes should 
reflect one and only one topic (e.g do not include two bug fixes in one pull 
request). Content should generally be kept to code and documentation, binary 
content, such as images, may be uploaded elsewhere. 

### Documentation
Code should be documented thoroughly. Generally speaking, you won't be here for 
more than 4(ish) years. Many students will come behind you and will need to use 
the code you've written. 

Each class or file you create should be documented as to what it contains and 
what purpose it serves.

Each function should have documentation containing it's purpose, what 
parameters is takes, and what values it returns. Error handling should be 
described as well: what errors will it produce, and what assumptions it makes 
regarding the validation the caller performs ahead of time. If relevant, state 
if the function may block for extended periods of time. If applicable, state if
the function is reentrant or accquires and releases locks.

If the overall set of code is complex and new, consider editing or adding to 
the wiki.

C/C++/Python is documented using doxygen. You can view the guide for writing doxygen 
comments and documentation [here](http://www.stack.nl/~dimitri/doxygen/manual/docblocks.html).

Text file types that are not supported by doxygen should still be documented 
using what ever commenting style that format supports.

### Style and Formatting
In order to keep the code more readable, code should be formatted and styled 
uniformly. This would be difficult to coordinate across multiple users, so we 
have a program that automatically restyles the code for you. If you submit a 
pull request before restyling the code, it will likely fail the CI check. You 
can auto-format the code by running `make pretty`. If you have a lot of code, 
you may have to run this a few times. You can check if the style is passing 
by running `make checkstyle`. If there are no errors, then you are good to go.

