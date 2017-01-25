
#include <QtWidgets>
using namespace std;


class StyleSheetManager {

	static void setStyleSheet(QMainWindow* window, QString path);

	public:
		static void changeStyleSheet(QMainWindow* window, QString name);
};