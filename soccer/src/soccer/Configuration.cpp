#include "Configuration.hpp"

#include <cassert>
#include <cstdio>
#include <iostream>
#include <utility>

#include <QTreeWidget>
#include <QTreeWidgetItem>

std::list<Configurable*>* Configurable::configurables_list;

// Role for tree column zero for storing ConfigItem pointers.
static const int kConfigItemRole = Qt::UserRole;

Q_DECLARE_METATYPE(ConfigItem*)  // FIXME: verify this NOLINT

ConfigItem::ConfigItem(Configuration* config, const QString& name, std::string description)
    : config_{config},
      tree_item_{nullptr},
      path_{name.split('/')},
      description_{std::move(description)} {}

ConfigItem::~ConfigItem() {
    if (tree_item_ != nullptr) {
        // FIXME - Things are getting deleted in a non-GUI thread
        // 		delete tree_item_;
        tree_item_ = nullptr;
    }
}

void ConfigItem::value_changed(const QString& str) {
    if (tree_item_ != nullptr) {
        tree_item_->setText(1, str);
    }
}

void ConfigItem::setup_item() { tree_item_->setText(1, to_string()); }

////////

ConfigBool::ConfigBool(Configuration* tree, const QString& name, bool value,
                       const std::string& description)
    : ConfigItem(tree, name, description), value_{value} {
    add_item();
}

QString ConfigBool::to_string() { return value_ ? "true" : "false"; }

bool ConfigBool::value() {
    if (tree_item_ != nullptr) {
        value_ = tree_item_->checkState(1) == Qt::Checked;
    }
    return value_;
}

void ConfigBool::set_value_string(const QString& str) {
    if (str == "true") {
        value_ = true;
    } else if (str == "false") {
        value_ = false;
    } else {
        // Use what's in the tree in case this was called because the user
        // clicked on it
        value_ = (tree_item_->checkState(1) == Qt::Checked);
    }
    setup_item();
}

void ConfigBool::setup_item() {
    if (tree_item_ != nullptr) {
        tree_item_->setCheckState(1, value_ ? Qt::Checked : Qt::Unchecked);
        // FIXME - Can't change the checkbox anymore.  Why not?
        tree_item_->setFlags(tree_item_->flags() | Qt::ItemIsUserCheckable);
    }
}

////////

ConfigInt::ConfigInt(Configuration* config, const QString& name, int value,
                     const std::string& description)
    : ConfigItem(config, name, description), value_{value} {
    add_item();
}

QString ConfigInt::to_string() { return QString::number(value_); }

void ConfigInt::set_value_string(const QString& str) { value_ = str.toInt(); }

////////

ConfigDouble::ConfigDouble(Configuration* config, const QString& name, double value,
                           const std::string& description)
    : ConfigItem(config, name, description), value_{value} {
    add_item();
}

QString ConfigDouble::to_string() { return QString::number(value_); }

void ConfigDouble::set_value_string(const QString& str) { value_ = str.toDouble(); }

Configuration::Configuration() {
    tree_ = nullptr;

    // Create the XML root element
    doc_.appendChild(doc_.createElement("config"));
}

std::shared_ptr<Configuration> Configuration::from_registered_configurables() {
    std::shared_ptr<Configuration> config = std::make_shared<Configuration>();
    for (Configurable* obj : Configurable::configurables()) {
        obj->create_configuration(config.get());
    }
    return config;
}

void Configuration::add_item(ConfigItem* item) {
    all_items_.push_back(item);

    if (tree_ != nullptr) {
        add_to_tree(item);
    }
}

void Configuration::add_to_tree(ConfigItem* item) {
    // Find the parent of this item by following the tree through all but the
    // last item in the path
    QTreeWidgetItem* parent = tree_->invisibleRootItem();
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

        if (next == nullptr) {
            // Create this item
            next = new QTreeWidgetItem(parent);  // NOLINT
            next->setText(0, *i);
        }

        parent = next;
    }

    // Create a tree item
    item->tree_item_ = new QTreeWidgetItem(parent);  // NOLINT
    item->tree_item_->setFlags(item->tree_item_->flags() | Qt::ItemIsEditable);
    item->tree_item_->setData(0, kConfigItemRole, QVariant::fromValue(item));
    item->tree_item_->setText(0, path.back());

    if (item->description_.empty()) {
        item->tree_item_->setToolTip(0, QString(item->description_.c_str()));
    }

    item->setup_item();
}

void Configuration::tree(QTreeWidget* tree) {
    assert(tree_ == nullptr);

    tree_ = tree;
    connect(tree_, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
            SLOT(itemChanged(QTreeWidgetItem*, int)));

    // Add items that were created before we got a tree
    for (ConfigItem* item : all_items_) {
        add_to_tree(item);
    }
}

ConfigItem* Configuration::config_item(QTreeWidgetItem* ti) {
    return ti->data(0, kConfigItemRole).value<ConfigItem*>();
}

void Configuration::itemChanged(QTreeWidgetItem* item, int column) {
    if (column == 1) {
        ConfigItem* ci = config_item(item);
        if (ci != nullptr) {
            ci->set_value_string(item->text(1));
        }
    }
}

ConfigItem* Configuration::name_lookup(const std::string& name) const {
    QStringList path = QString::fromStdString(name).split('/');
    for (ConfigItem* item : all_items_) {
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

    QDomDocument new_doc;
    QString dom_error;
    int error_line = 0;
    int error_column = 0;
    if (!new_doc.setContent(&file, &dom_error, &error_line, &error_column)) {
        error = QString("%1:%2: %3")
                    .arg(QString::number(error_line), QString::number(error_column), dom_error);
        return false;
    }

    QDomElement root = new_doc.firstChildElement("config");
    if (root.isNull()) {
        error = "XML does not contain root <config> element";
        return false;
    }

    doc_ = new_doc;
    for (ConfigItem* item : all_items_) {
        QDomElement el = root;
        for (QString str : item->path()) {
            bool is_int = false;
            int value = str.toInt(&is_int);
            if (is_int) {
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
                item->set_value_string(str);
                item->value_changed(str);
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

    QDomElement root = doc_.firstChildElement("config");

    // Update the DOM
    // FIXME - Remove superfluous vector elements
    for (ConfigItem* item : all_items_) {
        QDomElement el = root;
        for (QString str : item->path()) {
            bool is_int = false;
            int value = str.toInt(&is_int);
            if (is_int) {
                str = QString("item%1").arg(value);
            } else {
                // Sanitize the string
                str.replace(' ', '_');
            }

            // Find the element for the next part of the path
            QDomElement child = el.firstChildElement(str);
            if (child.isNull()) {
                child = doc_.createElement(str);
                el.appendChild(child);
            }

            el = child;
        }

        // Set the value
        el.setAttribute("value", item->to_string());
    }

    // Write XML
    QByteArray data = doc_.toByteArray();
    bool ok = (file.write(data) == data.size());
    if (!ok) {
        // Didn't write all the data
        error = file.errorString();
    }

    return ok;
}

Configurable::Configurable() {
    if (configurables_list == nullptr) {
        configurables_list = new std::list<Configurable*>();
    }

    configurables_list->push_back(this);
}

const std::list<Configurable*>& Configurable::configurables() {
    if (configurables_list == nullptr) {
        configurables_list = new std::list<Configurable*>();
    }

    return *configurables_list;
}
