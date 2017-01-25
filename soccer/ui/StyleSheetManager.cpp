
#include "StyleSheetManager.hpp"
#include <Utils.hpp>
#include <map>

// To add a new style sheet, declare the static variable
std::map<string, QString> filePaths = {
        {"NONE", QString("")},
        {"DARK", QString("../soccer/ui/QTDark.stylesheet")}
};

void StyleSheetManager::changeStyleSheet(QMainWindow* window, QString name) {
	// Add new cases here
	if (name == "NONE") {
		window->setStyleSheet("");
	} else if (name == "DARK") {
		setStyleSheet(window, filePaths["DARK"]);
	}
}

void StyleSheetManager::setStyleSheet(QMainWindow* window, QString path) {
	QFile file(ApplicationRunDirectory().filePath(path));
	file.open(QFile::ReadOnly);
	QString styleSheet = file.readAll();
	window->setStyleSheet(styleSheet);
    file.close();
}
