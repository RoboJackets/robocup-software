#ifndef _PATTERN_WINDOW_H_
#define _PATTERN_WINDOW_H_

#include "ui_pattern_window.h"
#include "vision/Vector_ID.h"

class Vector_Model;

class Pattern_Window: public QDialog
{
    Q_OBJECT;
    
public:
    Pattern_Window(QWidget *parent = 0);

    void update_offsets();
    
protected Q_SLOTS:
    void on_load_image_clicked();
    void on_vectors_customContextMenuRequested(const QPoint &pt);
    void on_view_vector_clicked(int i0, int i1);

protected:
    Ui_Pattern_Window ui;
    Vision::Vector_Pattern _pattern;
    Vector_Model *_model;
};

#endif // _PATTERN_WINDOW_H_
