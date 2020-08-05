
#include "StyleSheetManager.hpp"

#include <map>

#include <rj_common/qt_utils.hpp>

// To add a new style sheet, declare the static variable
std::map<QString, QString> file_paths = {
    // Add new entries here:
    {"DARK", "../soccer/ui/qt/themes/QTDark.stylesheet"},
    {"DARCULIZED", "../soccer/ui/qt/themes/darculized.stylesheet"},
    {"1337H4X0R", "../soccer/ui/qt/themes/1337h4x0r.stylesheet"},
    {"NYAN", "../soccer/ui/qt/themes/nyan.stylesheet"}};

void StyleSheetManager::changeStyleSheet(QMainWindow* window,
                                         const QString& name) {
    if (file_paths.count(name) != 0u) {
        // Found an element
        setStyleSheet(window, file_paths[name]);
    } else {
        // Default to no style sheet if we didn't find anything
        window->setStyleSheet("");
    }
}

void StyleSheetManager::setStyleSheet(QMainWindow* window,
                                      const QString& path) {
    // Normalize file paths to work regardless of location of pwd
    QFile file(ApplicationRunDirectory().filePath(path));
    file.open(QFile::ReadOnly);
    QString style_sheet = file.readAll();
    window->setStyleSheet(style_sheet);
    file.close();
}
