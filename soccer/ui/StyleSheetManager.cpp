
#include "StyleSheetManager.hpp"
#include <Utils.hpp>

StyleSheetManager::StyleSheetManager(MainWindow* window) {
	win =  window;
	DARK = "../soccer/ui/QTDark.stylesheet";
	NONE = "";
}

void StyleSheetManager::changeStyleSheet(QString name) {
	if (name.compare("") == 0) {
		win->setStyleSheet(NONE);
	}
	else if (name.compare("DARK") == 0) {
		QFile file(ApplicationRunDirectory().filePath(DARK));
    	file.open(QFile::ReadOnly);
  		QString styleSheet = file.readAll();
  		win->setStyleSheet(styleSheet);
	    file.close();
	}
}