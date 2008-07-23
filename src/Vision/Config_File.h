#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_

#include <QDomDocument>

class Camera_Thread;
class Camera_Window;

namespace Camera
{
    class Base;
    class DCam;
    class Prosilica;
};

namespace Vision
{
    class Colorseg;
}

namespace Camera
{
	class DCam;
	class File;
}

// If element has an attribute with the given name and the attribute's text is
// a valid number of the appropriate type, value is set to that number.
// Otherwise value is unchanged.
//
// Returns true iff value was changed.
bool read_param(QDomElement element, QString name, float &value);
bool read_param(QDomElement element, QString name, int &value);

class Config_File
{
public:
    Config_File(QString filename);
    
    void load(QString filename = QString());
    void save(QString filename = QString());
    
    Camera_Thread *camera_thread() const { return _camera_thread; }
    Camera_Window *window() const { return _win; }
    
    const QString &filename() const { return _filename; }
    
protected:
    QString _filename;
    QDomDocument _document;
    
    Camera_Thread *_camera_thread;
    Camera_Window *_win;
    
    // Cameras
    // Only one of these will be non-null.
    Camera::DCam *_dcam;
    Camera::File *_video;
    Camera::Prosilica *_prosilica;
    
    // Returns the named element child of the given node.
    // If no such element exists, one is created.
    QDomElement child_element(QDomNode node, QString name);
    
    // Removes all children of the given node
    void remove_children(QDomNode node);
    
    void save_spanners(QDomElement root);
    
    // Parsers
    void load_dcam(QDomElement &element);
    void load_prosilica(QDomElement &element);
    void load_video(QDomElement &element);
    void load_colorseg(QDomElement &element);
    void load_camera_common(QDomElement &element);
    
    // Error messages
    void error_prefix(const QDomElement &element);
    void unknown_element(const QDomElement &element);
    void missing_attr(const QDomElement &element, const char *name);

    // Helpers
    void start_camera(Camera::Base *cam);
};

#endif // _CONFIG_FILE_H_
