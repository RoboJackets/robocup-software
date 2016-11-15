
# Style

RoboCup Software uses Google Style guides for the most part, which can be found [at this website for c++](https://google.github.io/styleguide/cppguide.html), and [at this website for python](https://google.github.io/styleguide/pyguide.html). This docs page will cover a basic overview of these rules, as well as some of the differences between google and our style guides.

This guide is currently under construction.

# Overview

1. Wrap lines at 80 characters
2. Use 4 Spaces for Indentation
3. Comment your code, try to provide comments for every method.
4. Place spaces between operators
   - Ex: `4 + 3`, `(int) i`
5. Write readable code

# Autoformatting

We have two make targets dealing with formatting.

`make checkstyle` will check to see if your changes are up to style.

`make pretty` will use an autoformater to try and format your code.
