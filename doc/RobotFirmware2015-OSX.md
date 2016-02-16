# OSX Firmware Setup

## Install [Homebrew](http://brew.sh)

This is included in util/osx-setup already

~~~
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
~~~


## Install Cross-Compilers


### Install the [GCC ARM](https://launchpad.net/gcc-arm-embedded) Compiler

The following has been moved to util/osx-packages.txt

Install the compiler we will use for compiling the firmware with the following commands.

~~~
brew tap PX4/homebrew-px4
brew update
brew install gcc-arm-none-eabi-49
~~~

If multiple versions of arm-none-eabi-gcc are installed, make sure you set the default version to 4.9. There's no need to do this if this is your first time installing the arm-none-eabi compiler. For those who have other version installed, use the following command to set the system's default one to use.

~~~
brew link --overwrite gcc-arm-none-eabi-49
~~~


### Install the [GCC AVR](https://gcc.gnu.org/wiki/avr-gcc) Compiler

The following has been moved to util/osx-packages.txt

Use the following commands to install the AVR toolchain for compiling the kicker board's firmware. 

~~~
brew tap osx-cross/avr
brew update
brew install avr-libc
~~~

If you plan on flashing a binary file directly to an AVR microcontroller (like the one used on the kicker board), you'll need [AVRDUDE](http://www.nongnu.org/avrdude). This isn't necessary unless you're using an [ISP programmer](https://en.wikipedia.org/wiki/In-system_programming). Nonetheless, use the following command to install [AVRDUDE](http://www.nongnu.org/avrdude).

~~~
brew install avrdude --with-usb
~~~


### Install Additional Required Dependencies

Moved to `util/osx-packages.txt`


### Clone the Repository

Once the above steps are complete, you can download a local copy of our code using [git](https://git-scm.com).

~~~
git clone https://github.com/robojackets/robocup-software
~~~


### Update Submodules

We use git [submodules](https://git-scm.com/docs/git-submodule) to help manage the source code for the mbed libraries. Before attempting to compile the firmware, make sure you run the following command to download the code for libraries that will be needed during compilation.

~~~
git submodule update --init
~~~


### Checkout the Firmware Branch

~~~
cd robocup-software
git checkout robot2015-firmware
~~~


### Install Python Packages

moved to `util/requirements2.txt`


### Force OpenSSL

Sometimes openssl causes issues with brew. Especially with a fresh install of brew. So the following command will make sure we're using the brew version of openssl

~~~
brew link --force openssl
~~~
