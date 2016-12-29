
# Style

RoboCup Software uses Google Style guides for the most part, which can be found [at this website for c++](https://google.github.io/styleguide/cppguide.html), and [at this website for python](https://google.github.io/styleguide/pyguide.html). This docs page will cover a basic overview of these rules, as well as some of the differences between google and our style guides.

Our style configs for the code formatting programs that we use [are located here](https://github.com/RoboJackets/style-configs).


# Overview

1. Wrap lines at 80 characters
2. Use 4 Spaces for Indentation
3. Comment your code, try to provide comments for every method.
   - Doxygen has a specific format for comments. More information on commenting via doxygen [can be found here](doc/Documentation.md).
4. Place spaces between operators
   - Ex: `4 + 3`, `(int) i`
5. Write readable code

Style problems are mostly solved by our formatting scripts, although they can't deal with everything.

# Autoformatting

We have two make targets dealing with formatting.

`make checkstyle` will check to see if your changes are up to style.

`make pretty` will use an autoformater to try and format your code.

# Variable Naming Conventions

## C++

```
// File names should be CamelCase
MyClass.cpp
MyStandaloneHeader.hpp
MyHeader.h

// Class names are CamelCase
MyClassName a;

// Variables should be camelCase with a lowercase first letter
myVariable;

// Private variables should be preceeded with an underscore
_privateData;

// Where possible, avoid C syntax, and use C++ replacements.
printf("Hello world\n");       // bad!
cout << "Hello World" << endl; // good.
```

## Python

```
# Filenames (and therefore module names) should be lowercase_and_underscore
my_class.py
import my_class

# Class names should be CamelCase
class MyPythonClass():

# object variables should be underscore_seperated
self.my_class_variable = 5

# Class or object variables should be CamelCase
ClassVariable = 5
```
