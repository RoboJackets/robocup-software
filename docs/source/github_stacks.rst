GitHub Stacked PRs: Workflow
============================

Split big changes into a chain of small PRs. Each PR shows one layer, so reviews
are faster.

Rules
-----

* Stack when changes depend on each other. If they don't, open separate PRs from
  ``main``.
* Keep each PR to one logical change and under ~200 lines. (Not a hard rule, focus
  on decoupling your design into parts.)
* Put foundations at the bottom, e.g. schema → logic → API → UI.

One-time setup
--------------

.. code-block:: bash

   gh auth login
   gh extension install github/gh-stack

1. Create a stack
-----------------

.. code-block:: bash

    # --base ensures that the stack is built off that branch
    # for most use cases, use --base main
   gh stack init --base <branch_name> db-schema      # Initializes a new stack and enables rerere
   git add .
   git commit -m "Add orders table"

   gh stack add api-routes      # Adds a new layer on top of the current layer
   git add .
   git commit -m "Add orders endpoints"

Run ``gh stack add`` from the top branch of the stack.

2. Open the PRs
---------------

.. code-block:: bash

   gh stack submit

3. Handle review feedback
-------------------------

Make the fix on the branch that owns the code, not in a higher layer.

.. code-block:: bash

   gh stack checkout db-schema  # Switches to the layer that needs the fix
   git add .
   git commit -m "Address review feedback"

   # Cascading rebase across the stack:
   #   --upstack rebases from the current layer to the top layer
   #   --downstack rebases from the bottom layer to the current layer
   gh stack rebase --upstack

   gh stack push                # Pushes the updated layers
   gh stack top                 # Switches to the layer at the top of the stack

4. Merge from the bottom up
---------------------------

Merge the lowest PR on GitHub, or merge a higher PR to land everything below it
too. You can't merge a middle PR on its own.

5. Sync after each merge
------------------------

.. code-block:: bash

   gh stack sync --prune

If a rebase hits a conflict
---------------------------

Fix the files, then run:

.. code-block:: bash

   git add .
   gh stack rebase --continue   # Continues the rebase after your fix

To back out and restore all the branches:

.. code-block:: bash

   gh stack rebase --abort

Docs
----

* `Quickstart <https://docs.github.com/en/pull-requests/get-started/stacked-prs-quickstart>`__
* `CLI reference <https://docs.github.com/en/pull-requests/reference/stacked-prs-cli-commands>`__

----

:Written by: Yuvraj Dhadwal
:Last updated: November 1, 2026
