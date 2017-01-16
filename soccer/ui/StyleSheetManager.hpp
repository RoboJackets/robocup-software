
#include <QtWidgets>
using namespace std;

class StyleSheetManager {
	const QString DARK;
	const QString NONE;

	public:
		StyleSheetManager(MainWindow* window);
		QMainWindow* win;
		void changeStyleSheet(QString name);
};