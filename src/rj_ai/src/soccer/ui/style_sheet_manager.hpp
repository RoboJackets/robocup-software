#pragma once

#include <QtWidgets>

/**
 * A static class to help manage stylesheets for soccer
 */
class StyleSheetManager {
private:
    /**
     * Sets the style sheet to the qt style file specified by path
     *
     * @param window the QMainWindow to apply the style to
     * @param path The path to the style file to apply
     * Should be relative to the run directory.
     *
     * @see changeStyleSheet()
     */
    static void setStyleSheet(QMainWindow* window, const QString& path);

public:
    /**
     * Changes the style sheet to the style specified by name
     *
     * @param window the window to apply the style to
     * @param name the name of the style to apply. We must know about this style
     * To add new styles, add them to the hashmap in StyleSheetManager
     */
    static void changeStyleSheet(QMainWindow* window, const QString& name);
};
