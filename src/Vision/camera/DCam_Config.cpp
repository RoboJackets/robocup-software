#include "DCam.h"
#include "DCam_Config.moc"
#include "DCam_Config.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QTextStream>

#include <dc1394/control.h>

const char *feature_name(dc1394feature_t feature)
{
    switch (feature)
    {
        case DC1394_FEATURE_BRIGHTNESS:
            return "Brightness";
        case DC1394_FEATURE_EXPOSURE:
            return "Brightness";
        case DC1394_FEATURE_SHARPNESS:
            return "Sharpness";
        case DC1394_FEATURE_WHITE_BALANCE:
            return "White Balance";
        case DC1394_FEATURE_HUE:
            return "Hue";
        case DC1394_FEATURE_SATURATION:
            return "Saturation";
        case DC1394_FEATURE_GAMMA:
            return "Gamma";
        case DC1394_FEATURE_SHUTTER:
            return "Shutter";
        case DC1394_FEATURE_GAIN:
            return "Gain";
        case DC1394_FEATURE_IRIS:
            return "Iris";
        case DC1394_FEATURE_FOCUS:
            return "Focus";
        case DC1394_FEATURE_TEMPERATURE:
            return "Temperature";
        case DC1394_FEATURE_TRIGGER:
            return "Trigger";
        case DC1394_FEATURE_TRIGGER_DELAY:
            return "Trigger Delay";
        case DC1394_FEATURE_WHITE_SHADING:
            return "White Shading";
        case DC1394_FEATURE_FRAME_RATE:
            return "Framerate";
        case DC1394_FEATURE_ZOOM:
            return "Zoom";
        case DC1394_FEATURE_PAN:
            return "Pan";
        case DC1394_FEATURE_TILT:
            return "Tilt";
        case DC1394_FEATURE_OPTICAL_FILTER:
            return "Optical Filter";
        case DC1394_FEATURE_CAPTURE_SIZE:
            return "Capture Size";
        case DC1394_FEATURE_CAPTURE_QUALITY:
            return "Capture Quality";
        default:
            return "Unknown";
    }
}

DCam_Feature::DCam_Feature(dc1394camera_t *camera, dc1394feature_info_t *info, QWidget *parent, QGridLayout *layout, int row): QObject(parent)
{
    _camera = camera;
    _feature = info->id;

    // Label
    QLabel *label = new QLabel(QString(feature_name(info->id)) + ":", parent);
    layout->addWidget(label, row, 0, 1, 4);
    
    // Auto/manual combobox
    auto_check = new QCheckBox("Auto", parent);
    layout->addWidget(auto_check, row + 1, 3);

#if 0
    if (!info->auto_capable || !info->manual_capable)
    {
        auto_check->setEnabled(false);
    }
#endif

    dc1394feature_mode_t mode;
    dc1394_feature_get_mode(_camera, _feature, &mode);
    if (mode == DC1394_FEATURE_MODE_AUTO)
    {
        auto_check->setCheckState(Qt::Checked);
    }

    // Value
    if (_feature == DC1394_FEATURE_WHITE_BALANCE)
    {
        // White balance (2 sliders)
        
        layout->addWidget(new QLabel("B/U", parent), row + 1, 0);
        layout->addWidget(new QLabel("R/V", parent), row + 2, 0);
        
        for (int i = 0; i < 2; i++)
        {
            slider[i] = new QSlider(Qt::Horizontal, parent);
            layout->addWidget(slider[i], row + 1 + i, 1);
            slider[i]->setMinimum(info->min);
            slider[i]->setMaximum(info->max);
        
            pos_label[i] = new QLabel(parent);
            pos_label[i]->setMinimumSize(30, 10);
            layout->addWidget(pos_label[i], row + 1 + i, 2);
        }
        
        slider[0]->setSliderPosition(info->BU_value);
        pos_label[0]->setText(QString::number(info->BU_value));
        
        slider[1]->setSliderPosition(info->RV_value);
        pos_label[1]->setText(QString::number(info->RV_value));
        
        rows = 3;
    } else {
        // Single slider
        slider[0] = new QSlider(Qt::Horizontal, parent);
        layout->addWidget(slider[0], row + 1, 0, 1, 2);
    
        slider[0]->setMinimum(info->min);
        slider[0]->setMaximum(info->max);
        slider[0]->setSliderPosition(info->value);
        
        if (_feature == DC1394_FEATURE_SHUTTER)
        	slider[0]->setMaximum(255);
        
        pos_label[0] = new QLabel(QString::number(info->value), parent);
        pos_label[0]->setMinimumSize(30, 10);
        layout->addWidget(pos_label[0], row + 1, 2);
        
        slider[1] = 0;
        pos_label[1] = 0;
        
        rows = 2;
    }

    connect(auto_check, SIGNAL(toggled(bool)), SLOT(auto_changed(bool)));
    connect(slider[0], SIGNAL(valueChanged(int)), SLOT(slider0_changed(int)));
    if (slider[1])
    {
        connect(slider[1], SIGNAL(valueChanged(int)), SLOT(slider1_changed(int)));
    }
}

void DCam_Feature::auto_changed(bool on)
{
    dc1394feature_mode_t mode = on ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL;
    dc1394_feature_set_mode(_camera, _feature, mode);
}

void DCam_Feature::slider0_changed(int value)
{
    pos_label[0]->setText(QString::number(value));
    
    if (_feature == DC1394_FEATURE_WHITE_BALANCE)
    {
        // White balance
        dc1394_feature_whitebalance_set_value(_camera, slider[0]->sliderPosition(), slider[1]->sliderPosition());
    } else {
        // Everything else
        dc1394_feature_set_value(_camera, _feature, value);
    }
}

void DCam_Feature::slider1_changed(int value)
{
    pos_label[1]->setText(QString::number(value));
    
    // This is only called for white balance
    dc1394_feature_whitebalance_set_value(_camera, slider[0]->sliderPosition(), slider[1]->sliderPosition());
}

void DCam_Feature::update_value()
{
    if (_feature == DC1394_FEATURE_WHITE_BALANCE)
    {
        // White balance
        uint32_t bu = 0, rv = 0;
        dc1394_feature_whitebalance_get_value(_camera, &bu, &rv);
        slider[0]->setSliderPosition(bu);
        slider[1]->setSliderPosition(rv);
    } else {
        // Everything else
        uint32_t value = 0;
        dc1394_feature_get_value(_camera, _feature, &value);
        slider[0]->setSliderPosition(value);
    }
    
    dc1394feature_mode_t mode = DC1394_FEATURE_MODE_MANUAL;
    dc1394_feature_get_mode(_camera, _feature, &mode);
    if (mode == DC1394_FEATURE_MODE_AUTO)
    {
        auto_check->setCheckState(Qt::Checked);
    } else {
        auto_check->setCheckState(Qt::Unchecked);
    }
}

////////

DCam_Config::DCam_Config(Camera::DCam *dcam, QWidget *parent): QWidget(parent)
{
    _dcam = dcam;
    
    setWindowTitle(_dcam->name());
    
    _camera = _dcam->camera();
    dc1394_feature_get_all(_camera, &features);
    
    QVBoxLayout *vbox = new QVBoxLayout(this);
    vbox->setMargin(0);
    vbox->setSpacing(0);
    
    feature_grid = new QWidget(this);
    vbox->addWidget(feature_grid);
    QGridLayout *grid = new QGridLayout(feature_grid);
    grid->setSpacing(2);
    
    int row = 0;
    for (int i = DC1394_FEATURE_MIN; i <= DC1394_FEATURE_MAX; i++)
    {
        dc1394feature_info_t *feature = &features.feature[i - DC1394_FEATURE_MIN];
        
        if (feature->available)
        {
            DCam_Feature *obj = new DCam_Feature(_camera, feature, this, grid, row);
            row += obj->rows;
            feature_objects.push_back(obj);
        }
    }
}

void DCam_Config::load(QDomElement element)
{
	for (QDomElement feature_element = element.firstChildElement(); !feature_element.isNull(); feature_element = feature_element.nextSiblingElement())
	{
		if (feature_element.tagName() != "feature")
		{
			continue;
		}
    	
        dc1394feature_t feature = (dc1394feature_t)feature_element.attribute("id", "-1").toInt();
        if (feature < 0)
        {
        	continue;
        }
        
        int auto_active = feature_element.attribute("auto", "0").toInt();
        dc1394_feature_set_mode(_camera, feature, auto_active ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL);
        
        if (feature == DC1394_FEATURE_WHITE_BALANCE)
        {
            int BU_value = feature_element.attribute("bu_value", "0").toInt();
            int RV_value = feature_element.attribute("rv_value", "0").toInt();
            
            dc1394_feature_whitebalance_set_value(_camera, BU_value, RV_value);
        } else {
            int value = feature_element.attribute("value", "0").toInt();
            
            dc1394_feature_set_value(_camera, feature, value);
        }
	}
    
    update_values();
}

void DCam_Config::save(QDomElement element)
{
    dc1394_feature_get_all(_camera, &features);
    
    for (int i = DC1394_FEATURE_MIN; i <= DC1394_FEATURE_MAX; i++)
    {
        dc1394feature_info_t *feature = &features.feature[i - DC1394_FEATURE_MIN];

        if (feature->available)
        {
            dc1394feature_mode_t mode;
            dc1394_feature_get_mode(_camera, (dc1394feature_t)i, &mode);
            bool is_auto = (mode == DC1394_FEATURE_MODE_AUTO);
            
        	QDomElement feature_element = element.ownerDocument().createElement("feature");
        	element.appendChild(feature_element);
            feature_element.setAttribute("id", feature->id);
            feature_element.setAttribute("auto", is_auto);
            
            if (feature->id == DC1394_FEATURE_WHITE_BALANCE)
            {
            	feature_element.setAttribute("bu_value", feature->BU_value);
            	feature_element.setAttribute("rv_value", feature->RV_value);
            } else {
            	feature_element.setAttribute("value", feature->value);
            }
        }
    }
}

void DCam_Config::update_values()
{
    foreach(DCam_Feature *feature, feature_objects)
    {
        feature->update_value();
    }
}
