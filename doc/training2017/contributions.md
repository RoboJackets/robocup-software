# Contribution Overview {#t20177}


# Our Repositories

-   RoboJackets/robocup-software
    -   Our main software stack
-   RoboJackets/robocup-firmware
    -   Firmware for robots
    -   Firmware for base station
-   RoboJackets/robocup-common
    -   Libraries needed for both firmware and software
    -   Ex: Geometry, time libraries.
-   RoboJackets/rrt
    -   Path planning


# Big Picture

![img](https://i.imgur.com/VAK4mLY.png)


## Overview Picture

![img](https://cloud.GitHubusercontent.com/assets/4349709/11414363/8936f22e-93c2-11e5-9324-5c9055b1a4e4.jpg)


# Overview Of a Contribution


## Find an Issue

-   Can be on any of our repos
-   **Comment on the issue, let us know you're working on it**
-   Ask us so we can help you out to get started


## Fork the repo

![img]( https://i.imgur.com/FCv2gZH.png)


## Make a branch

```shell
# Clone your fork
git clone <your fork URL>

# Checkout a new branch
git checkout -b my-feature
```


## Make your changes

-   **Let us know if you get stuck**
-   Use documentation, stack overflow, google, etc
-   Test out your solution thoroughly
-   **Let us know if you get stuck**


## Add your changes to git

```shell

# Stage all files in the current directory '.'
git add .

# Make sure files you want saved are 'green'
git status

# Try to give a good status message!
git commit -m "Fixed my issue!"
```


## Push your changes up to your fork

```shell
git push -u origin <branch name>
```

-   Create a pull request on GitHub from your fork


## Creating a pull request on GitHub

![img]( https://i.imgur.com/P3BBw6m.png)

-   1. Go to the specific RoboJackets repository
-   2. Click on "Pull requests"
-   3. Click on "New pull request"


## Creating a pull request on GitHub

![img]( https://i.imgur.com/KxR8Wtz.png)

-   1. Click on compare across forks (So you can reference your fork)
-   2. Select the fork and branch on which you're working.
-   3. Click on "Create pull request"


## Creating a pull request on GitHub

![img]( https://i.imgur.com/n2yxtKe.png)


## Creating a pull request on GitHub

-   Write out a short description of what you fixed
    -   Mention the issue number
-   Request someone to review your pull request
-   Assign yourself so you can keep up


## Fix issues brought up

-   We'll review your changes, and give feedback on improvements that can be made.
-   Repeat the commit and push steps to revise your pull request.
    -   The Pull Request will automatically be updated


## Congrats!

-   Pick another issue to work on, or create a new issue!
