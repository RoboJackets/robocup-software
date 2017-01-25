#pragma once

#include <QtWidgets>

class StyleSheetManager {
private:
    static void setStyleSheet(QMainWindow* window, QString path);

public:
    static void changeStyleSheet(QMainWindow* window, QString name);
};
