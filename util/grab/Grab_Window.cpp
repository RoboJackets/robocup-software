#include "Grab_Window.hpp"
#include "Receive_Thread.hpp"

#include <QFileDialog>

Grab_Window::Grab_Window(QWidget *parent): QDialog(parent)
{
    rt = 0;
    
    ui.setupUi(this);
    
    connect(&timer, SIGNAL(timeout()), SLOT(on_grab_clicked()));
}

void Grab_Window::on_grab_clicked()
{
    rt->grab();
    ui.num_records->setText(QString("%1 records").arg(rt->record.size()));
}

void Grab_Window::on_clear_clicked()
{
    rt->record.clear();
    ui.num_records->setText("0 records");
}

void Grab_Window::on_periodic_toggled(bool on)
{
    if (on)
    {
        timer.start(ui.period->value());
    } else {
        timer.stop();
    }
}

void Grab_Window::on_period_valueChanged(int ms)
{
    if (timer.isActive())
    {
        timer.stop();
        timer.start(ms);
    }
}

void Grab_Window::on_save_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this, "Save Recorded Play", QString(), "Plays (*.play);;All files (*)");
    if (!filename.isNull())
    {
        FILE *fp = fopen(filename.toAscii().constData(), "wt");
        if (fp)
        {
            rt->record.write(fp);
            fclose(fp);
        }
    }
}
