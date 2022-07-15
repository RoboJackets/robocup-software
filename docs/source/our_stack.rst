Our Stack
=========

This is a brief overview of our codebase. Since the codebase is constantly
changing, expect that some or many parts of these docs will be out-of-date.
The most up-to-date reference on our codebase is our code itself.

.. note::

    To see when any documentation page was last updated, click "Edit on GitHub" in
    the top right, then see the timestamp on the last commit.

...
TODO(Kevin): stub out the rest

Overview
--------

ROS2
----

Vision
------

Gameplay
--------

Motion Planning
---------------

Radio
-----

Referee
-------

UI
--
We use Qt for UI development. Code for it is located in ``soccer/src/soccer/ui``.
The ``qt`` directory contains the view as .ui files and images that are placed on the view. 
You could directly edit these .ui files based on their proper syntax to change our view, but I wouldn't recommend it. 
Instead install Qt designer and open .ui files using it to save yourself a lot of time.

After the view, all the C++ files act as the view-model for each view. 
Some of them are quite messy and do not follow our naming conventions, 
but if you follow the pattern of other methods, then you should be able to add new functionality to any part of our ui.
