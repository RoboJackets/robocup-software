# style-configs

This repo contains config files that specify how code should be formatted, allowing us to easily keep code formatting consistent in an automated way.

Contents:

* .clang-format - Config file for [clang-format](http://clang.llvm.org/docs/ClangFormat.html), which formats C/C++ code
* .style.yapf - Config file for [yapf](https://github.com/google/yapf), which formats Python code

**Note**: These config files are used by multiple RoboJackets teams, so please consult with each of them before making changes.


## Usage

These configs can be used in another project to define how the code in that project should be formatted.  To add these configs to a project:

~~~{.sh}
# enter the project's directory
cd my-project

# add git submodule
git submodule add https://github.com/robojackets/style-configs

# symlink the config files to the root of the project
ln -s style-configs/.clang-format ./
ln -s style-configs/.style.yapf ./

# commit your changes
git add .
git commit -m 'added robojackets style configs'
~~~

After the configs are added to your project, you can use [stylize](https://github.com/justbuchanan/stylize) to format your code.  Note that by default `stylize` will change your files in-place, so it may be a good idea to commit any outstanding changes before you run it.

~~~{.sh}
# install stylize from pypi
pip install stylize

# see instructions for how to use stylize
stylize --help

# run stylize, specifying the config files and an exclude directory
stylize --clang_style=file --yapf_style=.style.yapf --exclude_dirs build

# show exactly what was changed by the formatter (assuming you committed code before running)
git diff
~~~
