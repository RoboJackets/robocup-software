#include "Config_File.h"

#include "camera/dcam/DCam.h"
#include "camera/dcam/DCam_Config.h"
#include "camera/prosilica/Prosilica.hpp"
#include "camera/file/File.h"

#include "Camera_Thread.h"
#include "Transform_Setup.h"
#include "vision/Process.h"
#include "vision/Colorseg.h"
#include "vision/Distortion.h"
#include "vision/Transform.h"
#include "Camera_Window.h"

#include <QFile>
#include <QDomDocument>
#include <QTextStream>

#include <stdexcept>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

bool read_param(QDomElement element, QString name, float &value)
{
	QString text = element.attribute(name);
	if (!text.isNull())
	{
		bool ok = false;
		float x = text.toFloat(&ok);
		if (ok)
		{
			value = x;

			return true;
		} else {
			printf("Bad float value for %s: \"%s\"\n", name.toAscii().constData(), text.toAscii().constData());
		}
	}

	return false;
}

bool read_param(QDomElement element, QString name, int &value)
{
	QString text = element.attribute(name);
	if (!text.isNull())
	{
		bool ok = false;
		int x = text.toInt(&ok);
		if (ok)
		{
			value = x;

			return true;
		} else {
			printf("Bad int value for %s: \"%s\"\n", name.toAscii().constData(), text.toAscii().constData());
		}
	}
    
	return false;
}

////////

Config_File::Config_File(QString filename)
{
	_dcam = 0;
	_video = 0;
	_camera_thread = 0;
	_win = 0;

    load(filename);
}

Config_File::~Config_File()
{
	delete _win;
}

void Config_File::load(QString filename)
{
	if (!filename.isNull())
	{
		_filename = filename;
	}

	QFile file(_filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		throw runtime_error(str(format("Can't read from %s") % _filename.toStdString()));
	}

	QString error;
	int err_line = 0, err_col = 0;
	if (!_document.setContent(&file, &error, &err_line, &err_col))
	{
		file.close();
		throw runtime_error(str(format("%s:%d:%d: %s")
				% _filename.toStdString()
				% err_line
				% err_col
				% error.toStdString()));
	}
	file.close();

	QDomElement element = _document.documentElement();
	if (element.tagName() == "dcam")
	{
		load_dcam(element);
	}
	else if (element.tagName() == "video")
	{
		load_video(element);
	}
	else if (element.tagName() == "prosilica")
	{
		load_prosilica(element);
	}
	else
	{
		throw runtime_error(str(format("Unexpected root element <%s>") % element.tagName().toStdString()));
	}
}

void Config_File::save(QString filename)
{
	if (!filename.isNull())
	{
		_filename = filename;
	}

	QDomElement root = _document.documentElement();

	// Update DCam features
	if (_dcam)
	{
		QDomElement features = child_element(root, "features");
		remove_children(features);
		_dcam->dcam_config()->save(features);
	}

	// Update distortion
	QDomElement distortion_element = child_element(root, "distortion");
	remove_children(distortion_element);
	_camera_thread->process->distortion->save(distortion_element);

	// Update transformations
	QDomElement ball_transform_element = child_element(root, "ball_transform");
	remove_children(ball_transform_element);
	_camera_thread->process->ball_transform->save(ball_transform_element);

	QDomElement robot_transform_element = child_element(root, "robot_transform");
	remove_children(robot_transform_element);
	_camera_thread->process->robot_transform->save(robot_transform_element);

	// Update reference points
	QDomElement ref_element = child_element(root, "ref_points");
	remove_children(ref_element);
	for (int i = 0; i < 3; ++i)
	{
		QDomElement point_element = _document.createElement("point");
		ref_element.appendChild(point_element);

		const Geometry::Point2d &point = _camera_thread->process->ball_transform->world_point[i];

		point_element.setAttribute("x", point.x);
		point_element.setAttribute("y", point.y);
	}

	// Update spanners
	save_spanners(root);

	// Update colorseg
	QDomElement colorseg_element = child_element(root, "colorseg");
	remove_children(colorseg_element);
	_camera_thread->process->colorseg->save(colorseg_element);

    QFile file(_filename);
    if (!file.open(QIODevice::WriteOnly))
    {
    	throw runtime_error(str(format("Can't write to %s") % _filename.toStdString()));
    }

    QTextStream ts(&file);
    _document.save(ts, 4);

    file.close();
}

QDomElement Config_File::child_element(QDomNode node, QString name)
{
	QDomElement child = node.firstChildElement(name);
	if (child.isNull())
	{
		child = _document.createElement(name);
		node.appendChild(child);
	}

	return child;
}

void Config_File::remove_children(QDomNode node)
{
	QDomNodeList children = node.childNodes();
	while (children.size())
	{
		node.removeChild(children.item(0));
	}
}

void Config_File::save_spanners(QDomElement root)
{
	QDomElement element;
	while (!(element = root.firstChildElement("spanner")).isNull())
	{
		root.removeChild(element);
	}

	for (int i = 0; i < Vision::Num_Colors; ++i)
	{
		element = _document.createElement("spanner");
		root.appendChild(element);
		_camera_thread->process->spanner[i]->save(element);
	}
}

void Config_File::load_dcam(QDomElement &element)
{
	if (!_camera_thread)
	{
	    QDomAttr name_attr = element.attributeNode("name");
	    if (name_attr.isNull())
	    {
	        missing_attr(element, "name");
	        return;
	    }

	    QDomAttr guid_attr = element.attributeNode("guid");
	    if (guid_attr.isNull())
	    {
	        missing_attr(element, "guid");
	        return;
	    }

	    bool ok = false;
	    QString guid_text = guid_attr.value();
	    uint64_t guid = guid_text.toULongLong(&ok, 16);
	    if (!ok)
	    {
	        error_prefix(element);
	        printf("Invalid guid \"%s\"\n", guid_text.toAscii().constData());
	        return;
	    }

	    _dcam = new Camera::DCam(guid);
	    _dcam->name(name_attr.value());
	    try
	    {
	    	_dcam->open();
	    } catch (exception &ex)
	    {
	        delete _dcam;
	        printf("Failed to open camera %016llx: %s\n", (unsigned long long)guid, ex.what());
	        return;
	    }

	    start_camera(_dcam);
	}

    for (QDomElement child = element.firstChildElement(); !child.isNull(); child = child.nextSiblingElement())
    {
        if (child.tagName() == "features")
        {
    		_dcam->dcam_config()->load(child);
        } else {
        	load_camera_common(child);
        }
    }
}

void Config_File::load_prosilica(QDomElement &element)
{
	//prevent loading of camera if already running
	if (!_camera_thread)
	{
		QDomAttr name_attr = element.attributeNode("name");
		if (name_attr.isNull())
		{
			missing_attr(element, "name");
			return;
		}

		QDomAttr uid_attr = element.attributeNode("uid");
		if (uid_attr.isNull())
		{
			missing_attr(element, "uid");
			return;
		}

		bool ok = false;
		QString uid_text = uid_attr.value();
		unsigned int uid = uid_text.toUInt(&ok, 10);
		if (!ok)
		{
			error_prefix(element);
			printf("Invalid uid \"%s\"\n", uid_text.toAscii().constData());
			return;
		}

		//printf("Loaded %u\n",uid);
		_prosilica = new Camera::Prosilica(uid);
		_prosilica->name(name_attr.value());

		try
		{
			_prosilica->open();
		}
		catch (exception &ex)
		{
			delete _prosilica;
			printf("Failed to open camera %u: %s\n", uid, ex.what());
			return;
		}

		start_camera(_prosilica);
	}

	for (QDomElement child = element.firstChildElement(); !child.isNull(); child = child.nextSiblingElement())
	{
		if (child.tagName() == "features")
		{
			//_dcam->dcam_config()->load(child);
		}
		else
		{
			load_camera_common(child);
		}
	}
}

void Config_File::load_video(QDomElement &element)
{
	if (!_camera_thread)
	{
        QDomAttr name_attr = element.attributeNode("name");
        if (name_attr.isNull())
        {
            missing_attr(element, "name");
            return;
        }

	    QDomAttr file_attr = element.attributeNode("file");
	    if (file_attr.isNull())
	    {
	        missing_attr(element, "file");
	        return;
	    }

	    //FIXME - Relative to config file path
	    Camera::File *cam = new Camera::File(file_attr.value().toAscii().constData());
        cam->name(name_attr.value());
	    try
	    {
	        cam->open();
	        cam->read_frame_from_file();
	    } catch (exception &ex)
	    {
	        error_prefix(element);
	        printf("%s\n", ex.what());
	    }

	    start_camera(cam);
	}

    for (QDomElement child = element.firstChildElement(); !child.isNull(); child = child.nextSiblingElement())
    {
        load_camera_common(child);
    }
}

void Config_File::load_camera_common(QDomElement &element)
{
	if (element.tagName() == "colorseg")
	{
		try
		{
			_camera_thread->process->colorseg->load(element);
		} catch (exception &ex)
		{
			printf("%s\n", ex.what());
		}
	} else if (element.tagName() == "paused")
	{
		_camera_thread->pause(true);
	} else if (element.tagName() == "spanner")
	{
		int color = 0;
		if (read_param(element, "color", color) && color >= 0 && color < Vision::Num_Colors)
		{
			_camera_thread->process->spanner[color]->load(element);
		}
		_win->set_spanner_parameters();
	} else if (element.tagName() == "ball_transform")
	{
		try
		{
			_camera_thread->process->ball_transform->load(element);
		} catch (exception &ex)
		{
			printf("%s\n", ex.what());
		}
	} else if (element.tagName() == "robot_transform")
	{
		try
		{
			_camera_thread->process->robot_transform->load(element);
		} catch (exception &ex)
		{
			printf("%s\n", ex.what());
		}
	} else if (element.tagName() == "distortion")
	{
		_camera_thread->process->distortion->load(element);
	} else if (element.tagName() == "ref_points")
	{
		int n = 0;
	    for (QDomElement child = element.firstChildElement(); !child.isNull(); child = child.nextSiblingElement())
	    {
	    	if (child.tagName() == "point")
	    	{
	    		if (n < 3)
	    		{
	    		    QDomAttr x_attr = child.attributeNode("x");
	    		    if (x_attr.isNull())
	    		    {
	    		        missing_attr(child, "x");
	    		        return;
	    		    }

	    		    QDomAttr y_attr = child.attributeNode("y");
	    		    if (y_attr.isNull())
	    		    {
	    		        missing_attr(child, "y");
	    		        return;
	    		    }

	    		    bool ok = false;
	    		    float x = x_attr.value().toFloat(&ok);
	    		    if (!ok)
	    		    {
	    		    	printf("Invalid x coordinate\n");
	    		    	continue;
	    		    }

	    		    float y = y_attr.value().toFloat(&ok);
	    		    if (!ok)
	    		    {
	    		    	printf("Invalid y coordinate\n");
	    		    	continue;
	    		    }

	    		    _camera_thread->process->ball_transform->world_point[n].x = x;
	    		    _camera_thread->process->ball_transform->world_point[n].y = y;

	    		    _camera_thread->process->robot_transform->world_point[n].x = x;
	    		    _camera_thread->process->robot_transform->world_point[n].y = y;

	    			++n;
	    		} else {
	    			error_prefix(element);
	    			printf("too many reference points (need exactly 3)");
	    		}
	    	} else {
	    		unknown_element(child);
	    	}
	    }

	    _win->ui.ref_table->resizeColumnsToContents();

	    if (n != 3)
	    {
			error_prefix(element);
			printf("too few reference points (need exactly 3)");
	    }
	} else {
	    unknown_element(element);
	}
}

void Config_File::error_prefix(const QDomElement &element)
{
    printf("%s:%d:%d: ", _filename.toAscii().constData(),
        element.lineNumber(), element.columnNumber());
}

void Config_File::unknown_element(const QDomElement &element)
{
    error_prefix(element);
    printf("Unknown element \"%s\"\n",
        element.tagName().toAscii().constData());
}

void Config_File::missing_attr(const QDomElement &element, const char *name)
{
    error_prefix(element);
    printf("\"%s\" element is missing \"%s\" attribute\n", element.tagName().toAscii().constData(), name);
}

void Config_File::start_camera(Camera::Base *cam)
{	
    _camera_thread = new Camera_Thread();
    _camera_thread->camera(cam);
    _camera_thread->update_time(33);
    _camera_thread->start();

    _win = new Camera_Window(this);
    _win->showMaximized();
}
