Contributing
============

Code
----

TODO(Kevin): docstrings, GitHub, etc.

Python docstrings: https://sphinx-rtd-tutorial.readthedocs.io/en/latest/docstrings.html

Docs
----

If you're writing documentation, you're probably pretty good at writing code
and Googling your way through issues already. So here's some basic info that
will help you figure out what you need to do on your own.

Our docs are hosted by ReadTheDocs and are built from ReStructuredText (``.rst``)
to HTML files. This build process is done by Sphinx. The Python-side API is
done by a Sphinx extension named ``sphinx-autodoc``. As of the time of this
writing, there is no C++-side API docs yet, but in theory Sphinx + Doxygen +
Breathe allow this functionality.

Aside from ensuring your docs are valid ``.rst`` files, you can check if your
changes look right in HTML format by simply using the ReadTheDocs page.
ReadTheDocs re-builds the documentation every time anyone pushes to a remote
branch, so simply push your changes to your branch and wait. It will take a
few minutes for your changes to show up.

Finally, if you look at the source files for these docs, you'll notice they are
nicely formatted to have a max line length of 80 chars. With Vim you can do
that with a simple

.. code-block::vim

   gq<motion>

For instance: ``gqq`` formats the current line. The above paragraph was
formatted with ``gqap`` ("around paragraph"). Be careful of breaking links when you do
this. (I'm sure modern IDEs have a similar feature, but I don't use an IDE, so
I wouldn't know. -Kevin Fu)

For more detail, read "Meta Docs", or use Google.
