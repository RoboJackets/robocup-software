# Beginner Git {#t2016git}

# Prerequisites

-   Have `git` installed via:
    -   `sudo apt install git` on ubuntu
    -   [Install for Windows from here](https://git-scm.com/download/win).
-   Know the following commands (run in `git bash` or in your terminal)
    -   `pwd`
    -   `cd`
    -   `ls`
    -   `mkdir`
    -   `rm`
    -   Learn these and more [here](https://help.ubuntu.com/community/UsingTheTerminal#File_.26_Directory_Commands)!
-   Have a text editor installed and be comfortable using it!
    -   Gui or terminal does not matter.

# `git`

> The Stupid Content Tracker

-   `git` is *NOT* GitHub.
-   GitHub is 'server git'

## Why `git` over another VCS?

-   Branching and Merging
-   ***FAST***
-   Powerful, and Flexible
-   Free (as in freedom, and as in beer)

## Why `git` over another client?

-   `git` is the 1st party front-end
-   `git` is the way it was designed to be experienced.
-   Learning `git` is the easiest way to understand what's going on.
-   `git` gives you ultimate, uninhibited access to your repository.

## `git` Vocabulary

| Name       | Definition                           |
|---------- |------------------------------------ |
| commit     | version                              |
| repository | collection of files to track         |
| branch     | a quick copy of your code to work on |
| remote     | any other git repository             |
| SHA/Hash   | Turns code into a random string      |

# Basic `git` Commands

-   Follow along on your own computer!

## Setting Up Git

```shell
# Let's git attribute your changes to you!
git config --global user.name "Jay Kamat"
git config --global user.email "jaygkamat@gmail.com"
```

## Creating a `git` Repo

```shell
# Create a folder named 'repo'
mkdir repo
# Move inside 'repo'
cd repo
# Create a git repository where we are
# (created inside the repo folder)

git init
# > Initialized empty Git repository in /tmp/repo/.git/
git status
# > On branch master
# > Initial commit
# > nothing to commit (create/copy files and use "git add" to track)

# Use cd .. to move up a directory
```

## Adding *Content*

```shell
# Create a file called file.txt with some text in it.
# Example:
echo "this is a line to be stored into git" > file.txt
git status
# > Untracked files:
# >   (use "git add <file>..." to include in what will be committed)
# >     file.txt
git add file.txt
# > Changes to be committed:
# >   (use "git rm --cached <file>..." to unstage)
# >         new file:   file.txt

# This message should reflect the change. We'll come back to this.
git commit -m "Added a small amount of content"
```

## Staging Area

![img](https://i.imgur.com/4nql3LO.jpg)

## Viewing History

-   `git` versions are 'hashes'
    -   Something like: `cbbd8d9a5e...`
    -   You don't need the full hash to refer to a commit
        -   Only use ~6 chars from the front.

```shell
# Show the latest commit
git show

# Show all commits (use q to quit)
git log
```

## `git` History

-   Every git version is linked to it's previous versions:
-   `v1 <- v2 <- v3 <- v4`
-   A version can have multiple previous versions
    -   Let's demonstrate this in&#x2026;

# `git` Branching

-   We'll discuss what we're doing in a bit
-   Let's create a new feature, but we know it's going to be buggy
    -   Let's create it in an isolated environment, a branch!

## Create a feature branch

```shell
# Create a myfeature branch
git branch myfeature
# Move to myfeature branch
git checkout myfeature

# Let's find out where we are
git status
# > On branch myfeature
# > nothing to commit, working tree clean
```

## Add a new 'feature'

```shell
echo "My new feature!" >> file.txt
git add file.txt
git commit -m 'My new feature is very cool, but buggy!'
```

## There's a bug in the released version!

-   Let's patch the version everyone is on, the master release!

```shell
# Switch back to the master branch
git checkout master
git status
# Look at the contents of file.txt at this point!

echo "My critical bugfix!" >> file.txt
git add file.txt
git commit -m "Critical bugfix number 1!"
```

## What just happened?

-   We first created our test file, which is our 'release version'
-   Then we created a feature branch, and made some 'buggy changes'
-   We then realized we need to fix a bug in the 'release version'
    -   We switched back to release, and 'fixed the bug'

## Let's take a closer look

```shell
# A much nicer git log that will visualize our history
git log --oneline --graph --all

git show <commit hash>
```

## Merging

-   Brings divergent branches together
-   Let's release our feature by putting it into the master branch

```shell
# We want to merge INTO master
git checkout master

# Merge myfeature into master
git merge myfeature
# > Auto-merging file.txt
# > CONFLICT (content): Merge conflict in file.txt
# > Automatic merge failed; fix conflicts and then commit the result.

# That's not good!
git status
# Edit file.txt to remove >>> <<< and ===
git add file.txt
git commit # Just save whatever pops up
git status
```

# `git` Remotes

-   How do we share our code with others?
-   Branching is great for me, but what if I want to let my friends work on a branch?

## Let's Play With Remotes

```shell
# let's start in a common state
git checkout master

# let's make a copy for 'our friend to have'
cd ..
# Clone from the repo folder to the repo2 folder
git clone repo/ repo2/
cd repo2
git status
# > On branch master
# > Your branch is up-to-date with 'origin/master'.
# > nothing to commit, working tree clean
git log --oneline --abbrev --graph --all
# *   8f12721 Merge branch 'myfeature'
# |\
# | * 2716a3b Add a new feature
# * | 625aaf5 Critical bugfix
# |/
# * cbbd8d9 one small change
```

## Let's Make a Change On 'our computer'

```shell
# Move into 'our repo'
cd ../repo
git status

# Edit file.txt with our changes
echo "A change I made on my computer" >> file.txt

git add file.txt
git commit -m 'Added files from my computer.'
```

## Let's Give This Change To Our Friend

```shell
# Pull new changes from repo's master into repo2's master
cd ../repo2
git pull ../repo master
# > From ../repo
# >  * branch            master     -> FETCH_HEAD
# > Updating 8f12721..2b05fa9
# > Fast-forward
# >  file.txt | 1 +
# >  1 file changed, 1 insertion(+)
```

## Shortcuts to make remotes faster

```shell
# Add a shortcut to interact with ../repo2
cd ../repo
git remote add repo2 ../repo2
# same as git pull ../repo2 master
git pull repo2 master
# > From ../repo2
# >  * branch            master     -> FETCH_HEAD
# > Already up-to-date.

# List existing remotes
git remote -v
# > repo2       ../repo2 (fetch)
# > repo2       ../repo2 (push)

# man git-remote for more options, like
# git remote set-url repo2 ../repo2
# git remote rm repo2
# git remote show repo2
```

## A note about `git fetch`

-   `git fetch` updates what we see in a remote without changing our files.
-   `git pull` updates our files with changes
-   `git fetch` stores those changes made without touching our state.

# `git` refs

-   `git` has a suite of commands for interacting with history.
    -   To use them, you will need to understand refs
-   A ref is anything that identifies a commit&#x2026;

## Examples of Refs

1.  A Commit Hash (eg: `cbbd8d9a5e1c64f84`)
    -   You can shorten with the first few chars
2.  Special: `HEAD`
    -   A Ref to the commit your repository is currently on
3.  Branch Names
    -   `master`, `myfeature`
4.  Remotes
    -   <remote>/<branch>
    -   `repo2/master`, `origin/master`
5.  Tags
    -   Aliases you can give to commits

## Ref Modifiers

-   Using `HEAD` as an example.
-   `HEAD~`; One commit before `HEAD`
-   `HEAD~~`; Two commits before `HEAD`
-   `HEAD~3`; Three commits before `HEAD`
-   `HEAD^<N>`; Choose the <Nth> parent if we're on a merge.

## Using Refs

```shell
# Show's the current commit, same as git show
git show HEAD

# Shows the commit that's one old
git show HEAD~

# Shows the commit thats two commits older than repo2's master
git show repo2/master~~
```

## Quickly See Files in a Ref

```shell
git checkout <ref>

# Does nothing, HEAD is our current ref
git checkout HEAD

# See the commit before head
git checkout HEAD~
# Warning about detached HEAD

# checkout to 2 commits before the remote's master branch.
git checkout repo2/master~~

# We also use checkout to switch branches (we can edit safely now)
git checkout master
```

## Change Where a Branch 'Points To'

```shell
# Git reset has many uses
# Run git status yourself!

# Move our branch to the previous commit, but don't change our files.
# Any diff is staged
git reset --soft HEAD~
# Go back to where we were
git pull repo2 master

# Move our branch to the previous commit, and change our files
# No diff is present (since we change files)
git reset --hard HEAD~
git pull repo2 master

# Like git reset --soft, but don't stage our files either.
git reset --mixed HEAD~
git pull repo2 master
# Complaint about unstaged files

# HEAD is the default if no ref is supplied
# This resets our files to head, trashing our 'unstaged work'
git reset --hard
git pull repo2 master
```

## Find the Diff Between Two Commits

```shell
# Finds the difference between one commit ago and the current commit
git diff HEAD~ HEAD

# Finds the difference between myfeature and master
git diff myfeature master

# Show unstaged changes
git diff

# Show only staged changes
git diff --staged
```

# Neat Tricks

## Quickly Make a Branch

```shell
git checkout -b mynewbranch

# Same as
git branch mynewbranch
git checkout mynewbranch
```

## Let Git Remember Your Remotes

```shell
git checkout master

# Right now you will need to always do
git push repo2 master

# If you run:
git push -u repo2 master
# Once, you can do
git push

# from now on, git will remember the remote and branch
# that you've paired with your local branch!
```

## Git Aliases

```shell
git config --global alias.s "status"

git s
# is now the same as
git status
```

## Tagging

```shell
git tag myrelease

# now you can do
git checkout myrelease
git reset --hard myrelease
git diff myrelease

# where myrelease refrences the commit you were on.
# Use git push --tags to push your tags as well.
```

## `git clean`

-   Use `git clean -f` to remove all *unstaged* files from your repo.
    -   THIS WILL REMOVE FILES ON YOUR FILESYSTEM.

# We Only Scratched The Surface

-   `git rebase`
-   `git revert`
-   `git submodule`
-   `git subtree`
-   `git reflog`
-   `git blame`
-   `git stash`
-   `git grep`
-   `git cherry-pick`
-   `git bisect`
-   `git apply`
-   `git merge-base`
-   `git archive`
-   `git am`

# Additional Resources

-   `man git`
-   `man git-command`
    -   eg: `man git-status`
-   [Online `git` Tutorial](https://git-scm.com/doc)

# BONUS: GitHub

## GitHub is just another remote!

```shell
git clone <repo url>

# Git will automatically add a remote for you called 'origin'
git remote -v

git pull origin master
git push origin master
git show origin/master
```

## Pull Request

-   A PR is a request for a maintainer to pull code into a repo you don't have access to
-   First fork the repo (this is a clone from a github repo to a github repo you have write access to)
-   Then clone your fork (you will now be able to commit/push to this fork)
-   Submit a PR from your fork into the target repo
-   Once accepted, the maintainer pulls code from your github fork to the main github repo.
