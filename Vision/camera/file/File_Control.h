#pragma once

#include <QWidget>

#include "ui_file_control.h"

namespace Camera
{
    class File;
    
    class File_Control: public QWidget
    {
        Q_OBJECT
        
    public:
        File_Control(File *file, QWidget *parent = 0);
    
    protected Q_SLOTS:
        void restart();
        void play();
        void step();
        void seek();
    
    protected:
        Ui_File_Control ui;
        
        File *_file;
        
        void setup_slider();
    };
}
