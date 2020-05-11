#pragma once

#include <QDomDocument>
#include <QFile>
#include <QStringList>
#include <memory>
#include <vector>

class QTreeWidget;
class QTreeWidgetItem;
class Configuration;
class ConfigItem;

/**
 * static variable for the program, extremely general
 */
class Configuration : public QObject {
    Q_OBJECT;

public:
    Configuration();

    /// Initializes a config object and adds all registered configurables to it.
    static std::shared_ptr<Configuration> FromRegisteredConfigurables();

    void tree(QTreeWidget* tree);

    [[nodiscard]] QTreeWidget* tree() const { return _tree; }

    // name lookup - returns pointer if exists, null otherwise
    [[nodiscard]] ConfigItem* nameLookup(const std::string& name) const;

    bool load(const QString& filename, QString& error);
    bool save(const QString& filename, QString& error);

protected Q_SLOTS:
    void itemChanged(QTreeWidgetItem* item, int column);

protected:
    friend class ConfigItem;

    // Called by ConfigItem's constructor
    void addItem(ConfigItem* item);

    void addToTree(ConfigItem* item);

    QList<ConfigItem*> _allItems;

    QTreeWidget* _tree;

    static ConfigItem* configItem(QTreeWidgetItem* ti);

    QDomDocument _doc;
};

/**
 * Base class for items in configuration: this is constructed through
 * functions in the Configuration class which ensure new items are added
 * to the tree properly
 */
class ConfigItem {
protected:
    ConfigItem(Configuration* config, const QString& name,
               std::string description = "");

    ConfigItem(const ConfigItem&) = delete;
    ConfigItem& operator=(const ConfigItem&) = delete;
    ConfigItem(ConfigItem&&) = delete;
    ConfigItem& operator=(ConfigItem&&) = delete;

public:
    virtual ~ConfigItem();

    // A ConfigItem's name is a sequence of path segments separated by '/'.
    // The path is a list of these segments in order.
    [[nodiscard]] const QStringList& path() const { return _path; }

    /// Returns the same name that was passed to the constructor
    [[nodiscard]] std::string name() const {
        return _path.join('/').toStdString();
    }

    virtual QString toString() = 0;

    // Called by Configuration when the user changes the value
    virtual void setValueString(const QString& str) = 0;

protected:
    friend class Configuration;

    void addItem() { _config->addItem(this); }

    // Called when the tree item is first created
    virtual void setupItem();

    // Called by subclasses to update item text
    void valueChanged(const QString& str);

    QStringList _path;
    Configuration* _config;
    QTreeWidgetItem* _treeItem;
    std::string _description;
};

class ConfigBool : public ConfigItem {
public:
    ConfigBool(Configuration* tree, const QString& name, bool value = false,
               const std::string& description = "");

    bool value();

    void setValue(bool val) {
        _value = val;
        setupItem();
    }

    operator bool() { return value(); }

    bool operator=(bool x) {
        setValue(x);
        return x;
    }

    QString toString() override;
    void setValueString(const QString& str) override;

protected:
    friend class Configuration;
    void setupItem() override;

    bool _value;
};

class ConfigInt : public ConfigItem {
public:
    ConfigInt(Configuration* config, const QString& name, int value = 0,
              const std::string& description = "");

    QString toString() override;
    void setValueString(const QString& str) override;

    operator int() const { return _value; }

    int operator=(int x) {
        setValue(x);
        return x;
    }

    [[nodiscard]] int value() const { return _value; }

    void setValue(int v) {
        _value = v;
        valueChanged(QString::number(v));
    }

protected:
    friend class Configuration;
    int _value;
};

class ConfigDouble : public ConfigItem {
public:
    ConfigDouble(Configuration* config, const QString& name, double value = 0,
                 const std::string& description = "");

    QString toString() override;
    void setValueString(const QString& str) override;

    operator double() const { return _value; }

    double operator=(double x) {
        setValue(x);
        return x;
    }

    [[nodiscard]] double value() const { return _value; }

    void setValue(double v) {
        _value = v;
        valueChanged(QString::number(v));
    }

protected:
    friend class Configuration;
    double _value;
};

#define REGISTER_CONFIGURABLE(x) static ConfigurableImpl<x> x##__configurable;

/**
 *
 */
class Configurable {
public:
    Configurable();
    virtual ~Configurable() = default;

    virtual void createConfiguration(Configuration* cfg) const = 0;

    static const std::list<Configurable*>& configurables();

private:
    /// Global list of all registered configurables
    static std::list<Configurable*>* _configurables;
};

/**
 * a template for making configurables
 * the implementing object is responsible for handling configuration
 */
template <class T>
class ConfigurableImpl : public Configurable {
public:
    void createConfiguration(Configuration* cfg) const override {
        T::createConfiguration(cfg);
    }
};
