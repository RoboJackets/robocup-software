//FIXME - For flexibility, set scrollbar maximum based on number of bits when the scroll color changes.

#include "Colorspace_Window.h"
#include "Colorspace_Window.moc"

Colorspace_Window::Colorspace_Window(QWidget *parent): QDialog(parent)
{
    ui.setupUi(this);

    for (int i = 0; i < Vision::In_Use_Colors; ++i)
    {
        ui.show_color->addItem(Vision::color_name[i]);
    }
    
    connect(ui.scroll_color, SIGNAL(currentIndexChanged(int)), ui.view, SLOT(scroll_color(int)));
    connect(ui.show_color, SIGNAL(currentIndexChanged(int)), ui.view, SLOT(show_color(int)));
    connect(ui.scrollbar, SIGNAL(valueChanged(int)), ui.view, SLOT(scroll_pos(int)));
    connect(ui.view, SIGNAL(scroll(int)), ui.scrollbar, SLOT(setValue(int)));
}

void Colorspace_Window::colorseg(Vision::Colorseg *colorseg)
{
    ui.view->colorseg = colorseg;
}

void Colorspace_Window::on_fill_clicked()
{
    printf("FIXME: fill\n");
}

void Colorspace_Window::update_view()
{
    ui.view->update();
}

void Colorspace_Window::show_bin(int n)
{
    ui.show_color->setCurrentIndex(n);
}

void Colorspace_Window::show_color(QRgb color)
{
    int pos = ui.view->get_scroll_pos(color);
    ui.scrollbar->setValue(pos);
    printf("show %08x %3d\n", color, pos);
}

void Colorspace_Window::on_show_color_currentIndexChanged(int n)
{
    color_changed(n);
}
