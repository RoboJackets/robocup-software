
#include <QtWidgets>
using namespace std;

class StyleSheetManager {
	const QString DARK= "../soccer/ui/QTDark.stylesheet";
	const QString NONE = "";

	public:
		StyleSheetManager(QMainWindow* window);
		QMainWindow* win;
		void changeStyleSheet(QString name);
};