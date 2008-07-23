#ifndef _DCAM_CONFIG_H_
#define _DCAM_CONFIG_H_

#include <QWidget>
#include <QDomElement>

#include <dc1394/control.h>
#include <vector>

namespace Vision
{
    class DCam;
};

class QCheckBox;
class QSlider;
class QLabel;
class QGridLayout;

class DCam_Feature: public QObject
{
    Q_OBJECT;

public:
    DCam_Feature(dc1394camera_t *camera, dc1394feature_info_t *feature, QWidget *parent, QGridLayout *layout, int row);

    void update_value();
    
    int rows;

protected slots:
    void auto_changed(bool on);
    void slider0_changed(int value);
    void slider1_changed(int value);

protected:
    // Camera
    dc1394camera_t *_camera;
    
    // Feature ID
    dc1394feature_t _feature;
    
    // Auto/manual checkbox
    QCheckBox *auto_check;
    
    // The second slider is only used for white balance.
    // The sliders are blue/U and red/V.
    QSlider *slider[2];
    
    // A label for each slider
    QLabel *pos_label[2];
};

class DCam_Config: public QWidget
{
    Q_OBJECT;

public:
    DCam_Config(Camera::DCam *dcam, QWidget *parent = 0);

    void load(QDomElement element);
    void save(QDomElement element);

    void update_values();

    QWidget *feature_grid;

protected:
    Camera::DCam *_dcam;
    dc1394camera_t *_camera;
    dc1394featureset_t features;
    
    std::vector<DCam_Feature *> feature_objects;
};

#endif // _DCAM_CONFIG_H_
