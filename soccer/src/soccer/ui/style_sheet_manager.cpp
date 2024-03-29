
#include "style_sheet_manager.hpp"

#include <map>

#include <rj_common/qt_utils.hpp>

// To add a new style sheet, declare the static variable
std::map<QString, QString> filePaths = {
    // Add new entries here:
    {"DARK", "../../../soccer/src/soccer/ui/qt/themes/QTDark.stylesheet"},
    {"DARCULIZED", "../../../soccer/src/soccer/ui/qt/themes/darculized.stylesheet"},
    {"1337H4X0R", "../../../soccer/src/soccer/ui/qt/themes/1337h4x0r.stylesheet"},
    {"NYAN", "../../../soccer/src/soccer/ui/qt/themes/nyan.stylesheet"}};

void StyleSheetManager::changeStyleSheet(QMainWindow* window, const QString& name) {
    if (filePaths.count(name) != 0u) {
        // Found an element
        setStyleSheet(window, filePaths[name]);
    } else {
        // Default to no style sheet if we didn't find anything
        window->setStyleSheet("");
    }
}

void StyleSheetManager::setStyleSheet(QMainWindow* window, const QString& path) {
    // Normalize file paths to work regardless of location of pwd
    QFile file(application_run_directory().filePath(path));
    file.open(QFile::ReadOnly);
    QString styleSheet = file.readAll();
    window->setStyleSheet(styleSheet);
    file.close();
}
