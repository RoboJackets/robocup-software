#include "Dummy.h"

#include <stdexcept>

using namespace std;

Camera::Dummy::Dummy()
{
    _open = false;
    _size = QSize(640, 480);
}

QString Camera::Dummy::name()
{
    return "Dummy";
}

void Camera::Dummy::open()
{
    if (_open)
    {
        throw runtime_error("Camera::Dummy::open: already open\n");
    }
    
    _frame = Image(_size.width(), _size.height(), 4);
    _open = true;
}

void Camera::Dummy::close()
{
    if (!_open)
    {
        printf("Camera::Dummy::close: not open\n");
    }
    
    _frame = Image();
    _open = false;
}

bool Camera::Dummy::is_open()
{
    return _open;
}

QWidget *Camera::Dummy::configuration()
{
    //FIXME - size
    return 0;
}

QSize Camera::Dummy::size()
{
    return _size;
}

const Image *Camera::Dummy::read_frame()
{
    return &_frame;
}
