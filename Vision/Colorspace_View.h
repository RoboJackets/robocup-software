#pragma once

#include <QWidget>

#include "vision/Colorseg.h"

class Colorspace_View: public QWidget
{
    Q_OBJECT;
    
public:
    Colorspace_View(QWidget *parent = 0);
    
    Vision::Colorseg *colorseg;
    
    QColor block_color(int x, int y) const;
    int get_scroll_pos(QRgb color) const;
    
public Q_SLOTS:
    // Selects which color component to scroll.
    // 0=red, 1=green, 2=blue.
    void scroll_color(int n);
    
    void scroll_pos(int pos);
    
    // Selects which color bin (0..NumColors) to show.
    void show_color(int n);

Q_SIGNALS:
    void scroll(int delta);
    
protected:
    int _scroll_color;
    int _scroll_pos;
    int _color;
    
    void paintEvent(QPaintEvent *e);
    void wheelEvent(QWheelEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
};
