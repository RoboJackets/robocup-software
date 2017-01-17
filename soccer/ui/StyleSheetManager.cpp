
#include "StyleSheetManager.hpp"
#include <Utils.hpp>

StyleSheetManager::StyleSheetManager(QMainWindow* window) {
	win =  window;
}

void StyleSheetManager::changeStyleSheet(QString name) {
	if (name.compare("NONE") == 0) {
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