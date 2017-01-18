
#include <QtWidgets>
using namespace std;


class StyleSheetManager {
	static QString DARK;
	static QString NONE;

	public:
		static void changeStyleSheet(QMainWindow* window, QString name);
};