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

When you edit documentation, you only need to edit the ``.rst`` files in
docs/source/ and Sphinx will do the rest. If you add a new page, put it in the
``docs/source/`` folder, and make sure it's a valid ``.rst`` file. Then link to
it in the toctree in ``docs/source/index.rst``. (Toctree = table of contents
tree.) The toctree will preserve the order given.

You can check if your changes look right in HTML format by building locally
with ``make docs`` and ``make open-docs``. This will open a static version of
the documentation that only exists on your local machine.

TODO(Kevin): check that this still works

.. note::

    It may take a few minutes for your changes to show up.

Finally, if you look at the source files for these docs, you'll notice they are
nicely formatted to have a max line length of 80 chars. With Vim you can do
that with a simple

.. code-block::

   gq<motion>

For instance: ``gqq`` formats the current line. The above paragraph was
formatted with ``gqap`` ("around paragraph"). Be careful of breaking links when
you do this. (Modern IDEs should also have a similar feature, but clearly the
author of these docs is a Vim user.)

For more detail, read "Meta Docs", or use Google.
