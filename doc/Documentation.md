
# Documentation

It's important to keep software projects well-documented so that newcomers can quickly get up-to-speed and have a reference when questions arise.  One common problem with software documentation is that it is easy for the documentation to get out-of-sync with the code, since the code is constantly updated and documenting it is often an afterthought.

One way to help with this is to put the documentation inline with the code, which is what we do for this project.  We use a program called [Doxygen](http://www.stack.nl/~dimitri/doxygen/) that parses specially-formatted comments in our code and turns them into a searchable [website](http://robojackets.github.io/robocup-software/) that can be easily viewed.  Note that this is very similar to a program called [javadoc](http://en.wikipedia.org/wiki/Javadoc) that many GT students are probably familiar with using in class.

Another way to improve documentation practices is by requiring code to be well-documented before merging GitHub [pull-requests](https://help.github.com/articles/using-pull-requests), which is something we're getting better at doing.


## Doxygen Comment Formats

Doxygen has support for many different languages, but the comment syntax differs a bit.  Below are a couple examples, but you should check out the [official Doxygen docs](http://www.stack.nl/~dimitri/doxygen/manual/docblocks.html) for more info.


### C++

\code
/**
 * a normal member taking two arguments and returning an integer value.
 * @param a an integer argument.
 * @param s a constant character pointer.
 * @see Test()
 * @see ~Test()
 * @see testMeToo()
 * @see publicVar()
 * @return The test results
 */
 int testMe(int a,const char *s);
\endcode


### Python

\code{.py}
## Documentation for a class.
#
#  More details.
class PyClass:

    ## The constructor.
    def __init__(self):
        self._memVar = 0;

    ## Documentation for a method.
    #  @param self The object pointer.
    def PyMethod(self):
        pass

    ## A class variable.
    classVar = 0;

    ## @var _memVar
    #  a member variable
\endcode

## Additional Documentation

In addition to turning inline code comments into documentation, Doxygen can also include docs in other formats.  This page that you are viewing right now and several others are written in [Markdown](http://daringfireball.net/projects/markdown/syntax).  See the files in `doc` for examples.


## Compiling the documentation

To build the documentation website, run `doxygen` from the root of the robocup-software project.  This will place a bunch of files in `api_docs/html`.  Open the `index.html` file in a browser to view the site.

Our documentation website automagically updates everytime someone pushes the master branch of the repository.  This is setup through [circle-ci](https://circleci.com) - see the `autoupdate-docs.sh` script to see how this is done.


## Further configuration

Doxgen looks at the `Doxyfile` in the root of our project to configure things such as which files to include and how to display the output.

