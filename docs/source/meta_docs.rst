Meta-Docs
=========

Documentation on how this documentation was set up.

Documentation for how to write documentation is under "Contributing".

Log
---

Kevin Fu, 5/7/2022 

First, big thanks to Oswin So, who wrote `this PR <https://github.com/RoboJackets/robocup-software/pull/1574>`_ 
setting up most of the framework for these docs. That said, I've made some
slight changes and it was a pain to figure out, so I'm writing this doc to help
anyone who needs to change them in the future.

The biggest change was I added the sphinx-autodoc library, which automatically
generates API documentation based on Python docstrings. This will make it
easier to figure out what's going on in the codebase (since docs are searchable
and bookmark-able, while code isn't unless you have good command over a fuzzy
finder) and will hopefully encourage people to write docstrings. In theory,
Doxygen + Breathe should give the same functionality for C++ code that follows
the right format, but as of the time of this writing I haven't set that up yet.

Also, a lot has changed since Oswin set up Read The Docs support two years
ago. The largest change is that Read The Docs switched from building src/ to
source/ by default. Between Oswin's PR and now, Michael Rodyushkin set up 
`static building <https://github.com/RoboJackets/robocup-software/pull/1882>`_ 
according to the new convention, without deleting the old src/conf.py file,
leaving two configuration files in our docs/ folder. This confused me for a
while.

Long story short: Read The Docs (unsurprisingly) has good docs, so if this
documentation ever breaks, refer to
`their docs <https://docs.readthedocs.io/en/stable/tutorial/index.html>`_
to fix it.

After resolving that, Oswin's setup worked. To make sure these files stay up to
date on Read The Docs, though, I changed the GitHub webhook to point to a new
end URL (since the link in the PR above was broken). This is in our GitHub page
under Settings > Webhooks > https://readthedocs.io(...). Again, the tutorial by
ReadTheDocs is really good in case this ever breaks. This means I am now the
maintainer of RC Software's docs, indefinitely, so when I graduate and someone
reading this would like to take that role from me, Slack me.

My changes are in `PR #1897 <https://github.com/RoboJackets/robocup-software/pull/1897>`_.
