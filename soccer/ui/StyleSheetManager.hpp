
#include <QtWidgets>
using namespace std;


class StyleSheetManager {
	// To add a new style sheet, declare a new variable here
	// and add it to the C++ file
	static QString DARK;
	static QString NONE;

	static void setStyleSheet(QMainWindow* window, QString path);

	public:
		static void changeStyleSheet(QMainWindow* window, QString name);
};