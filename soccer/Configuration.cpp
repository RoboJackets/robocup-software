#include "Configuration.hpp"

#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <stdio.h>
#include <assert.h>


std::list<Configurable*>* Configurable::_configurables;

// Role for tree column zero for storing ConfigItem pointers.
static const int ConfigItemRole = Qt::UserRole;

Q_DECLARE_METATYPE(ConfigItem*)  // FIXME: verify this

ConfigItem::ConfigItem(Configuration* config, const QString& name,
                       std::string description) {
    _config = config;
    _treeItem = nullptr;
    _path = name.split('/');
    _description = description;
}

ConfigItem::~ConfigItem() {
    if (_treeItem) {
        // FIXME - Things are getting deleted in a non-GUI thread
        // 		delete _treeItem;
        _treeItem = nullptr;
    }
}

void ConfigItem::valueChanged(const QString& str) {
    if (_treeItem) {
        _treeItem->setText(1, str);
    }
}

void ConfigItem::setupItem() { _treeItem->setText(1, toString()); }

////////

ConfigBool::ConfigBool(Configuration* tree, QString name, bool value,
                       std::string description)
    : ConfigItem(tree, name, description) {
    _value = value;
    addItem();
}

QString ConfigBool::toString() { return _value ? "true" : "false"; }

bool ConfigBool::value() {
    if (_treeItem) {
        _value = _treeItem->checkState(1) == Qt::Checked;
    }
    return _value;
}

void ConfigBool::setValue(const QString& str) {
    if (str == "true") {
        _value = true;
    } else if (str == "false") {
        _value = false;
    } else {
        // Use what's in the tree in case this was called because the user
        // clicked on it
        _value = (_treeItem->checkState(1) == Qt::Checked);
    }
    setupItem();
}

void ConfigBool::setupItem() {
    if (_treeItem) {
        _treeItem->setCheckState(1, _value ? Qt::Checked : Qt::Unchecked);
        // FIXME - Can't change the checkbox anymore.  Why not?
        _treeItem->setFlags(_treeItem->flags() | Qt::ItemIsUserCheckable);
    }
}

////////

ConfigInt::ConfigInt(Configuration* config, QString name, int value,
                     std::string description)
    : ConfigItem(config, name, description) {
    _value = value;
    addItem();
}

QString ConfigInt::toString() { return QString::number(_value); }

void ConfigInt::setValue(const QString& str) { _value = str.toInt(); }

////////

ConfigDouble::ConfigDouble(Configuration* config, QString name, double value,
                           std::string description)
    : ConfigItem(config, name, description) {
    _value = value;
    addItem();
}

QString ConfigDouble::toString() { return QString::number(_value); }

void ConfigDouble::setValue(const QString& str) { _value = str.toDouble(); }

Configuration::Configuration() {
    _tree = nullptr;

    // Create the XML root element
    _doc.appendChild(_doc.createElement("config"));
}

std::shared_ptr<Configuration> Configuration::FromRegisteredConfigurables() {
    std::shared_ptr<Configuration> config = std::make_shared<Configuration>();
    for (Configurable* obj : Configurable::configurables()) {
        obj->createConfiguration(config.get());
    }
    return config;
}

void Configuration::addItem(ConfigItem* item) {
    _allItems.push_back(item);

    if (_tree) {
        addToTree(item);
    }
}

void Configuration::addToTree(ConfigItem* item) {
    // Find the parent of this item by following the tree through all but the
    // last item in the path
    QTreeWidgetItem* parent = _tree->invisibleRootItem();
    const QStringList& path = item->path();
    QStringList::const_iterator last = path.end();
    --last;
    for (QStringList::const_iterator i = path.begin(); i != last; ++i) {
        QTreeWidgetItem* next = nullptr;
        for (int j = 0; j < parent->childCount(); ++j) {
            QTreeWidgetItem* child = parent->child(j);
            if (child->text(0) == *i) {
                next = child;
                break;
            }
        }

        if (!next) {
            // Create this item
            next = new QTreeWidgetItem(parent);
            next->setText(0, *i);
        }

        parent = next;
    }

    // Create a tree item
    item->_treeItem = new QTreeWidgetItem(parent);
    item->_treeItem->setFlags(item->_treeItem->flags() | Qt::ItemIsEditable);
    item->_treeItem->setData(0, ConfigItemRole, QVariant::fromValue(item));
    item->_treeItem->setText(0, path.back());

    if (item->_description != "") {
        item->_treeItem->setToolTip(0, QString(item->_description.c_str()));
    }

    item->setupItem();
}

void Configuration::tree(QTreeWidget* tree) {
    assert(!_tree);

    _tree = tree;
    connect(_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
            SLOT(itemChanged(QTreeWidgetItem*, int)));

    // Add items that were created before we got a tree
    for (ConfigItem* item : _allItems) {
        addToTree(item);
    }
}

ConfigItem* Configuration::configItem(QTreeWidgetItem* ti) {
    return ti->data(0, ConfigItemRole).value<ConfigItem*>();
}

void Configuration::itemChanged(QTreeWidgetItem* item, int column) {
    if (column == 1) {
        ConfigItem* ci = configItem(item);
        if (ci) {
            ci->setValue(item->text(1));
        }
    }
}

ConfigItem* Configuration::nameLookup(const QString& name) const {
    QStringList path = name.split('/');
    for (ConfigItem* item : _allItems) {
        if (item->path() == path) {
            return item;
        }
    }
    return nullptr;
}

bool Configuration::load(const QString& filename, QString& error) {
    QFile file(filename);
    if (!file.open(QFile::ReadOnly)) {
        error = file.errorString();
        return false;
    }

    QDomDocument newDoc;
    QString domError;
    int errorLine = 0, errorColumn = 0;
    if (!newDoc.setContent(&file, &domError, &errorLine, &errorColumn)) {
        error = QString("%1:%2: %3")
                    .arg(QString::number(errorLine),
                         QString::number(errorColumn), domError);
        return false;
    }

    QDomElement root = newDoc.firstChildElement("config");
    if (root.isNull()) {
        error = "XML does not contain root <config> element";
        return false;
    }

    _doc = newDoc;
    for (ConfigItem* item : _allItems) {
        QDomElement el = root;
        for (QString str : item->path()) {
            bool isInt = false;
            int value = str.toInt(&isInt);
            if (isInt) {
                str = QString("item%1").arg(value);
            } else {
                // Sanitize the string
                str.replace(' ', '_');
            }

            // Find the element for the next part of the path
            el = el.firstChildElement(str);
            if (el.isNull()) {
                break;
            }
        }

        if (!el.isNull()) {
            QString str = el.attribute("value");
            if (!str.isNull()) {
                item->setValue(str);
                item->valueChanged(str);
            }
        }
    }

    return true;
}

bool Configuration::save(const QString& filename, QString& error) {
    QFile file(filename);
    if (!file.open(QFile::WriteOnly)) {
        error = file.errorString();
        return false;
    }

    QDomElement root = _doc.firstChildElement("config");

    // Update the DOM
    // FIXME - Remove superfluous vector elements
    for (ConfigItem* item : _allItems) {
        QDomElement el = root;
        for (QString str : item->path()) {
            bool isInt = false;
            int value = str.toInt(&isInt);
            if (isInt) {
                str = QString("item%1").arg(value);
            } else {
                // Sanitize the string
                str.replace(' ', '_');
            }

            // Find the element for the next part of the path
            QDomElement child = el.firstChildElement(str);
            if (child.isNull()) {
                child = _doc.createElement(str);
                el.appendChild(child);
            }

            el = child;
        }

        // Set the value
        el.setAttribute("value", item->toString());
    }

    // Write XML
    QByteArray data = _doc.toByteArray();
    bool ok = (file.write(data) == data.size());
    if (!ok) {
        // Didn't write all the data
        error = file.errorString();
    }

    return ok;
}

Configurable::Configurable() {
    if (!_configurables) {
        _configurables = new std::list<Configurable*>();
    }

    _configurables->push_back(this);
}

const std::list<Configurable*>& Configurable::configurables() {
    if (!_configurables) {
        _configurables = new std::list<Configurable*>();
    }

    return *_configurables;
}
