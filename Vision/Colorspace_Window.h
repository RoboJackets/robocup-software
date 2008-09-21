#ifndef _COLORSPACE_WINDOW_H_
#define _COLORSPACE_WINDOW_H_

#include "ui_colorspace_window.h"

class Colorspace_Window: public QDialog
{
    Q_OBJECT;
    
public:
    Colorspace_Window(QWidget *parent = 0);

    void colorseg(Vision::Colorseg *colorseg);

Q_SIGNALS:
    void color_changed(int n);

public Q_SLOTS:
    void update_view();
    void show_bin(int n);
    void show_color(QRgb color);

protected Q_SLOTS:
    void on_fill_clicked();
    void on_show_color_currentIndexChanged(int n);

protected:
    Ui_Colorspace_Window ui;
};

#endif // _COLORSPACE_WINDOW_H_

