#ifndef _PATTERN_VIEW_H_
#define _PATTERN_VIEW_H_

#include "vision/Colorseg.h"
#include "vision/Spanner.h"
#include "vision/Distortion.h"
#include "vision/Transform.h"
#include "vision/Vector_ID.h"

#include <QWidget>

class Pattern_View: public QWidget
{
    Q_OBJECT;
    
public:
    Pattern_View(QWidget *parent = 0);
    ~Pattern_View();
    
    void load_image(const QString &filename);
    
    Vision::Dot_ID *dot_id() const { return _dot_id; }
    const std::vector<Vision::Dot_ID::Dot> &dots() const { return _dot_id->dots(); }
    Vision::Group *center_group() const { return _center_group; }
    
    Vision::Vector_Pattern *pattern;

Q_SIGNALS:
    void vector_clicked(int i0, int i1);
    
protected:
    QImage _image;
    Vision::Colorseg *_colorseg;
    Vision::Distortion *_distortion;
    Vision::Transform *_transform;
    Vision::Spanner *_spanner[Vision::Num_Colors];
    Vision::Dot_ID *_dot_id;
    Vision::Group *_center_group;
    int _click_dot;
    int _i0;
    Geometry::Point2d _drag_pos;
    
    void draw_vector(QPainter &p, const Geometry::Point2d &p0, const Geometry::Point2d &p1);
    
    void paintEvent(QPaintEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
};

#endif // _PATTERN_VIEW_H_
