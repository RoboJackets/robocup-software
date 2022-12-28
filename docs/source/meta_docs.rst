Meta-Docs
=========

Documentation on how to write documentation.

How to Write Documentation
--------------------------

If you're writing documentation, you're probably pretty good at writing code and
Googling your way through issues already. So here's some basic info that will
help you figure out what you need to do on your own.

Our docs are hosted by ReadTheDocs and are built from ReStructuredText
(``.rst``) to HTML files. This build process is done by Sphinx. The Python-side
API is done by a Sphinx extension named ``sphinx-autodoc``. As of the time of
this writing, there is no C++-side API docs yet, but in theory Sphinx + Doxygen
+ Breathe allow this functionality.

When you edit documentation, you only need to edit the ``.rst`` files in
docs/source/ and Sphinx will do the rest. If you add a new page, put it in the
``docs/source/`` folder, and make sure it's a valid ``.rst`` file. Then link to
it in the toctree in ``docs/source/index.rst``. (Toctree = table of contents
tree.) The toctree will preserve the order given.

You can check your work by building locally or asking whoever maintains these
docs (as of 2022, Kevin Fu) to add your branch as a version on the ReadTheDocs
dashboard.

**Building via RTD**

The documentation maintainer should go to the ReadTheDocs dashboard for our
repo, activate the version that points to the new doc development branch, and
build that version, then give you the link once it builds.

.. note::

    It may take a few minutes for your changes to show up.

**Local Building**

This section explains how to locally build our docs. Locally building the docs
is not worth the trouble unless you don't have access to the RTD dashboard and
whoever does can't be reached.

You must install all of the dependencies in ``docs/requirements.txt`` file. This
can be done with the  ``python3 -m pip install -r requirements.txt`` command.
After doing so, change directories to the ``docs`` folder in our code base and
``make html``. This will create multiple build files that should NOT be
committed to your branch nor ``ros2``.

.. note::

    DO NOT COMMIT THE BUILD FILES GENERATED WHEN LOCALLY BUILDING! Although
    .gitignore may take care of this issue, it is not a good practice to
    knowingly commit thousands of files that you have not worked on.

This will generate the docs with the changes made from your branch. When
committing to your branch, make sure to exclude the build files.

Finally, if you look at the source files for these docs, you'll notice they are
nicely formatted to have a max line length of 80 chars. With Vim you can do that
with a simple

.. code-block::

   gq<motion>

For instance: ``gqq`` formats the current line. The above paragraph was
formatted with ``gqap`` ("around paragraph"). Be careful of breaking links when
you do this. (Modern IDEs should also have a similar feature, but clearly the
author of these docs is a Vim user.)

For more detail, read the Log below.

Log
---

Kevin Fu, 5/24/2022

To save myself some time in the future:

The RTD dashboard points to the branch that latest builds under `"Advanced
Settings" <https://readthedocs.org/dashboard/rj-rc-software/advanced/>`.

Also, you can build multiple branches easily, for development of new
documentation. Just go to the dashboard, hit Versions, and add whichever branch.
It should build automatically, but you can check under Builds. (The alternate
versions will have different links, click Builds > <version name> > "view docs")

Kevin Fu, 5/7/2022

First, big thanks to Oswin So, who wrote `this PR
<https://github.com/RoboJackets/robocup-software/pull/1574>`_ setting up most of
the framework for these docs. That said, I've made some slight changes and it
was a pain to figure out, so I'm writing this doc to help anyone who needs to
change them in the future.

The biggest change was I added the sphinx-autodoc library, which automatically
generates API documentation based on Python docstrings. This will make it easier
to figure out what's going on in the codebase (since docs are searchable and
bookmark-able, while code isn't unless you have good command over a fuzzy
finder) and will hopefully encourage people to write docstrings. In theory,
Doxygen + Breathe should give the same functionality for C++ code that follows
the right format, but as of the time of this writing I haven't set that up yet.

Also, a lot has changed since Oswin set up Read The Docs support two years ago.
The largest change is that Read The Docs switched from building src/ to source/
by default. Between Oswin's PR and now, Michael Rodyushkin set up `static
building <https://github.com/RoboJackets/robocup-software/pull/1882>`_ according
to the new convention, without deleting the old src/conf.py file, leaving two
configuration files in our docs/ folder. This confused me for a while.

Long story short: Read The Docs (unsurprisingly) has good docs, so if this
documentation ever breaks, refer to `their docs
<https://docs.readthedocs.io/en/stable/tutorial/index.html>`_ to fix it.

After resolving that, Oswin's setup worked. To make sure these files stay up to
date on Read The Docs, though, I changed the GitHub webhook to point to a new
end URL (since the link in the PR above was broken). This is in our GitHub page
under Settings > Webhooks > https://readthedocs.io(...). Again, the tutorial by
ReadTheDocs is really good in case this ever breaks. This means I am now the
maintainer of RC Software's docs, indefinitely, so when I graduate and someone
reading this would like to take that role from me, Slack me.

My changes are in `PR #1897
<https://github.com/RoboJackets/robocup-software/pull/1897>`_.
