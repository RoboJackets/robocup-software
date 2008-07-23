#include "Pattern_View.h"
#include "Pattern_View.moc"
#include "Image.h"

#include <vector>
#include <stdexcept>

#include <QPainter>
#include <QMouseEvent>

#include <boost/foreach.hpp>

using namespace std;
using namespace boost;
using namespace Vision;

Pattern_View::Pattern_View(QWidget *parent): QWidget(parent)
{
    _click_dot = 0;
    pattern = 0;
    
    QPalette p = palette();
    p.setColor(QPalette::Background, Qt::black);
    setPalette(p);
    setAutoFillBackground(true);
    
    // Create dummy distortion and transform objects for the spanners
    _distortion = new Distortion();
    _transform = new Transform();
    
    // Create part of a vision pipeline
    _colorseg = new Colorseg(0);
    for (int i = 0; i < Num_Colors; ++i)
    {
        _spanner[i] = new Spanner(_colorseg, i);
        
        _spanner[i]->distortion = _distortion;
        _spanner[i]->transform = _transform;
        
        // Allow large groups since the reference images are assumed to have no noise
        // and may be enlarged.
        _spanner[i]->max_group_width = 1000;
        _spanner[i]->max_group_height = 1000;
    }
    _dot_id = new Dot_ID(0, Blue);
    _dot_id->spanner = _spanner;
    
    // Detect only standard colors
    _colorseg->lut(QColor(Qt::blue).rgb()) |= 1 << Blue;
    _colorseg->lut(QColor(Qt::yellow).rgb()) |= 1 << Yellow;
    _colorseg->lut(QColor(Qt::green).rgb()) |= 1 << Green;
    _colorseg->lut(QColor(Qt::red).rgb()) |= 1 << Pink;
    _colorseg->lut(QColor(Qt::white).rgb()) |= 1 << White;
    
#if 0
    printf("center %.1f, %.1f\n", _image.width() / 2.0, _image.height() / 2.0);
    for (int i = 0; i < Num_Colors; ++i)
    {
        const std::list<Group *> &groups = _spanner[i]->groups();
        printf("color %d has %5d pixels and %d groups\n", i, _colorseg->color_count[i], (int)groups.size());
        
        BOOST_FOREACH(const Group *group, groups)
        {
            printf("    %.1f, %.1f\n", group->center.x, group->center.y);
        }
    }
#endif
}

Pattern_View::~Pattern_View()
{
    delete _colorseg;
    for (int i = 0; i < Num_Colors; ++i)
    {
        delete _spanner[i];
    }
}

void Pattern_View::load_image(const QString &filename)
{
    _image = QImage(filename);
    
    int w = _image.width();
    int h = _image.height();
    
    // Use conventional coordinates, with the origin in the lower left corner.
    _transform->image_point[0] = Geometry::Point2d(0, h);
    _transform->image_point[1] = Geometry::Point2d(w, h);
    _transform->image_point[2] = Geometry::Point2d(0, 0);
    
    _transform->world_point[0] = Geometry::Point2d(0, 0);
    _transform->world_point[1] = Geometry::Point2d(w, 0);
    _transform->world_point[2] = Geometry::Point2d(0, h);
    
    if (!_transform->calculate_matrix(_distortion))
    {
        throw logic_error("Pattern_View::load_image: bad transform image points");
    }
    
    Image img(_image);
    _colorseg->run(&img);

    for (int i = 0; i < Num_Colors; ++i)
    {
        _spanner[i]->run();
    }
    
    if (_spanner[Blue]->groups().size() == 1)
    {
        _center_group = _spanner[Blue]->groups().front();
        _dot_id->collect(_center_group);
        _dot_id->remove_center();
        _dot_id->align_largest_gap();
        _dot_id->assign_dot_ids();
    }
    
    update();
}

void Pattern_View::draw_vector(QPainter &p, const Geometry::Point2d &p0, const Geometry::Point2d &p1)
{
    Geometry::Point2d t = (p1 - p0).norm() * 10;
    Geometry::Point2d n = t.perpCCW();
    
    Geometry::Point2d a0 = p1 - t + n;
    Geometry::Point2d a1 = p1 - t - n;
    
    p.drawLine(p0.x, p0.y, p1.x, p1.y);
    p.drawLine(p1.x, p1.y, a0.x, a0.y);
    p.drawLine(p1.x, p1.y, a1.x, a1.y);
}

void Pattern_View::paintEvent(QPaintEvent *e)
{
    QPainter p(this);
    p.drawImage(QPoint(0, 0), _image);
    
    p.setPen(Qt::black);
    const vector<Dot_ID::Dot> &dots = _dot_id->dots();
    for (unsigned int i = 0; i < dots.size(); ++i)
    {
        const Vector_ID::Dot &dot = dots[i];
        float x = dot.group->raw_center.x;
        float y = dot.group->raw_center.y;
        
        p.drawText(QPointF(x, y), QString::number(i));
        p.drawLine(QPointF(x - 5, y - 5), QPointF(x + 5, y + 5));
        p.drawLine(QPointF(x - 5, y + 5), QPointF(x + 5, y - 5));
    }
    
    p.setPen(Qt::red);
    if (pattern)
    {
        for (unsigned int i = 0; i < pattern->pairs.size(); ++i)
        {
            int i0 = pattern->pairs[i].i0;
            int i1 = pattern->pairs[i].i1;
            
            draw_vector(p, dots[i0].group->raw_center, dots[i1].group->raw_center);
        }
    }
    
    if (_click_dot == 1)
    {
        draw_vector(p, dots[_i0].group->raw_center, _drag_pos);
    }
}

void Pattern_View::mouseMoveEvent(QMouseEvent *e)
{
    if (_click_dot == 1)
    {
        _drag_pos = Geometry::Point2d(e->x(), e->y());
        update();
    }
}

void Pattern_View::mousePressEvent(QMouseEvent *e)
{
    // Find which dot was clicked
    int n = -1;
    const vector<Dot_ID::Dot> &dots = _dot_id->dots();
    for (unsigned int i = 0; i < dots.size(); ++i)
    {
        const Group *group = dots[i].group;
        if (e->x() >= group->min_x && e->x() <= group->max_x && e->y() >= group->min_y && e->y() <= group->max_y)
        {
            n = i;
            break;
        }
    }
    
    if (n >= 0)
    {
        ++_click_dot;
        if (_click_dot == 1)
        {
            // First click
            _i0 = n;
            setMouseTracking(true);
        } else if (_click_dot == 2)
        {
            // Second click
            vector_clicked(_i0, n);
            setMouseTracking(false);
            _click_dot = 0;
        }
    }
}
