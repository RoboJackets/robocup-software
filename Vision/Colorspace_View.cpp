//FIXME - Use bit counts from Colorseg

#include "Colorspace_View.h"
#include "Colorspace_View.moc"

#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>

Colorspace_View::Colorspace_View(QWidget *parent): QWidget(parent)
{
    colorseg = 0;
    _scroll_color = 0;
    _scroll_pos = 0;
    _color = 0;
    
    QPalette p = palette();
    p.setColor(QPalette::Background, Qt::black);
    setPalette(p);
    setAutoFillBackground(true);
}

QColor Colorspace_View::block_color(int x, int y) const
{
    int s = _scroll_pos * 16;
    x *= 16;
    y *= 16;
    
    switch (_scroll_color)
    {
        case 0:
            return QColor(s, x, y);
        
        case 1:
            return QColor(x, s, y);
        
        case 2:
            return QColor(x, y, s);
        
        default:
            return QColor();
    }
}

int Colorspace_View::get_scroll_pos(QRgb color) const
{
    int pos;
    switch (_scroll_color)
    {
        case 0:
            pos = qRed(color);
            break;
        
        case 1:
            pos = qGreen(color);
            break;
        
        case 2:
            pos = qBlue(color);
            break;
        
        default:
            return 0;
    }
    
    return pos / 16;
}

void Colorspace_View::paintEvent(QPaintEvent *e)
{
    QPainter p(this);
    
    int w = width() / 16, h = height() / 16;
    
    for (int x = 0; x < 16; ++x)
    {
        for (int y = 0; y < 16; ++y)
        {
            QColor color = block_color(x, y);
            
            if (colorseg && (colorseg->lut(color.rgb()) & (1 << _color)))
            {
                p.setBrush(color);
            } else {
                p.setBrush(Qt::NoBrush);
            }
            
            p.setPen(color);
            p.drawEllipse(x * w + 2, y * h + 2, w - 4, h - 4);
        }
    }
}

void Colorspace_View::wheelEvent(QWheelEvent *e)
{
    scroll(_scroll_pos - e->delta() / 120);
    e->accept();
}

void Colorspace_View::mousePressEvent(QMouseEvent *e)
{
    if (e->buttons() == Qt::LeftButton)
    {
        QColor color = block_color(e->x() / 16, e->y() / 16);
        colorseg->lut(color.rgb()) |= 1 << _color;
        update();
    } else if (e->buttons() == Qt::RightButton)
    {
        QColor color = block_color(e->x() / 16, e->y() / 16);
        colorseg->lut(color.rgb()) &= ~(1 << _color);
        update();
    }
}

void Colorspace_View::mouseMoveEvent(QMouseEvent *e)
{
    mousePressEvent(e);
}

void Colorspace_View::scroll_color(int n)
{
    _scroll_color = n;
    update();
}

void Colorspace_View::scroll_pos(int pos)
{
    _scroll_pos = pos;
    update();
}

void Colorspace_View::show_color(int n)
{
    _color = n;
    update();
}
