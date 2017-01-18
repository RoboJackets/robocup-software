
#include "StyleSheetManager.hpp"
#include <Utils.hpp>

QString StyleSheetManager::DARK = "../soccer/ui/QTDark.stylesheet";
QString StyleSheetManager::NONE = "";

void StyleSheetManager::changeStyleSheet(QMainWindow* window, QString name) {
	if (name.compare("NONE") == 0) {
		window->setStyleSheet(NONE);
	}
	else if (name.compare("DARK") == 0) {
		QFile file(ApplicationRunDirectory().filePath(DARK));
    	file.open(QFile::ReadOnly);
  		QString styleSheet = file.readAll();
  		window->setStyleSheet(styleSheet);
	    file.close();
	}
}