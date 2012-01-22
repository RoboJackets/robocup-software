#pragma once

 #include <QGLWidget>

 /**
  * Widget to support viewing of the simulation in 3D
  */
 class SimViewer : public QGLWidget
 {
     Q_OBJECT

 public:
     SimViewer(QWidget *parent = 0);
     ~SimViewer();

     QSize minimumSizeHint() const;
     QSize sizeHint() const;

 protected:
     void initializeGL();
     void paintGL();
     void resizeGL(int width, int height);

 };
