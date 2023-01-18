
# Themes

RoboCup Software has built in Theme support, via QT Stylesheets. By default no themes will be used.

# Theme Selection

To select a theme, from the top bar:

`View->Change Style Sheet-><Name of Stylesheet>`

To keep your theme permanently add the following to your bashrc

```sh
export SOCCER_THEME="ALLCAPSTHEMENAME"
# for example
export SOCCER_THEME="DARK"
```

# Writing Themes

Adding your own theme to soccer takes a few steps:

1. Write the stylesheet (if starting from scratch, copy an existing stylesheet file from `./soccer/ui/themes`.
   - Copy this stylesheet into the `./soccer/ui/themes` folder
2. Edit `MainWindow.ui` to add an entry for your theme in the themes menu.
    - You can do this in qtcreator
    - Also, you will need to add triggers in MainWindow.cpp to actually trigger your theme.
    - In MainWindow.cpp add your new button to the radio group
3. Edit `StyleSheetManager.cpp` and add your theme to the map of tables present there.
4. You're set! See the existing themes for examples.
