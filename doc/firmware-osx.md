# OSX Firmware Setup

## Install [Homebrew](http://brew.sh/)
```
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```


## Install Cross-Compilers

### Install the [GCC ARM Compiler](https://launchpad.net/gcc-arm-embedded)
```
brew tap brew tap PX4/homebrew-px4
brew update
brew install gcc-arm-none-eabi-49
```

### Install the GCC AVR compiler](https://gcc.gnu.org/wiki/avr-gcc)
```
brew tap osx-cross/avr
brew update
brew install gcc-avr-48
```


### Install Additional Required Dependencies
```
brew install git
brew install ecache
```

### Disable SIP
Set `/usr/bin/cc` & `/usr/bin/c++` to the gcc versions located in /usr/bin/local
  -- version 4.9.3 should show with both 'c++ --version' and 'cc --version'
  -- this requires disabling osx 
    - reboot your mac, holding 'Command + R' until the Apple logo with a loading bar appears
    - open a terminal from the menu up top
    - run the command 'csrutil disable', it should give you a confirmation message saying it was successfull
    - now you can remove the symbolic links in /usr/bin and set them yourself to point to gcc versions


### Clone the Repository
```
git clone https://github.com/robojackets/robocup-software
```

### Update Submodules
```
git submodule update --init
```

### Install Python Packages
```
pip install colorama jinja1
```


-- if multiple versions of arm-none-eabi-gcc are installed, make sure you set the default version to 4.9
`brew link --overwrite gcc-arm-none-eabi-49`
