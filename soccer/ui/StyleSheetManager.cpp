
#include "StyleSheetManager.hpp"
#include <Utils.hpp>
#include <map>

// To add a new style sheet, declare the static variable
std::map<QString, QString> filePaths = {
    // Add new entries here:
    {"DARK", "../soccer/ui/themes/QTDark.stylesheet"},
    {"DARCULIZED", "../soccer/ui/themes/darculized.stylesheet"},
    {"1337H4X0R", "../soccer/ui/themes/1337h4x0r.stylesheet"},
    {"NYAN", "../soccer/ui/themes/nyan.stylesheet"}};

void StyleSheetManager::changeStyleSheet(QMainWindow* window, QString name) {
    if (filePaths.count(name)) {
        // Found an element
        setStyleSheet(window, filePaths[name]);
    } else {
        // Default to no style sheet if we didn't find anything
        window->setStyleSheet("");
    }
}

void StyleSheetManager::setStyleSheet(QMainWindow* window, QString path) {
    // Normalize file paths to work regardless of location of pwd
    QFile file(ApplicationRunDirectory().filePath(path));
    file.open(QFile::ReadOnly);
    QString styleSheet = file.readAll();
    window->setStyleSheet(styleSheet);
    file.close();
}
