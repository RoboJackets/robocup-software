
#include "StyleSheetManager.hpp"
#include <Utils.hpp>

// To add a new style sheet, declare the static variable 
// in the header file and initialize the path here
QString StyleSheetManager::DARK = "../soccer/ui/QTDark.stylesheet";
QString StyleSheetManager::NONE = "";

void StyleSheetManager::changeStyleSheet(QMainWindow* window, QString name) {
	// Add new cases here

	if (name.compare("NONE") == 0) {
		window->setStyleSheet(NONE);
	} else if (name.compare("DARK") == 0) {
		setStyleSheet(window, DARK);
	}
}

void StyleSheetManager::setStyleSheet(QMainWindow* window, QString path) {
	QFile file(ApplicationRunDirectory().filePath(path));
	file.open(QFile::ReadOnly);
	QString styleSheet = file.readAll();
	window->setStyleSheet(styleSheet);
    file.close();
}