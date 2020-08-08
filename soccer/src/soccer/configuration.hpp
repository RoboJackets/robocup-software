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
    static std::shared_ptr<Configuration> from_registered_configurables();

    void tree(QTreeWidget* tree);

    [[nodiscard]] QTreeWidget* tree() const { return tree_; }

    // name lookup - returns pointer if exists, null otherwise
    [[nodiscard]] ConfigItem* name_lookup(const std::string& name) const;

    bool load(const QString& filename, QString& error);
    bool save(const QString& filename, QString& error);

protected Q_SLOTS:
    // NOLINTNEXTLINE(readability-identifier-naming)
    void itemChanged(QTreeWidgetItem* item, int column);

protected:
    friend class ConfigItem;

    // Called by ConfigItem's constructor
    void add_item(ConfigItem* item);

    void add_to_tree(ConfigItem* item);

    QList<ConfigItem*> all_items_;

    QTreeWidget* tree_;

    static ConfigItem* config_item(QTreeWidgetItem* ti);

    QDomDocument doc_;
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
    [[nodiscard]] const QStringList& path() const { return path_; }

    /// Returns the same name that was passed to the constructor
    [[nodiscard]] std::string name() const {
        return path_.join('/').toStdString();
    }

    virtual QString to_string() = 0;

    // Called by Configuration when the user changes the value
    virtual void set_value_string(const QString& str) = 0;

protected:
    friend class Configuration;

    void add_item() { config_->add_item(this); }

    // Called when the tree item is first created
    virtual void setup_item();

    // Called by subclasses to update item text
    void value_changed(const QString& str);

    QStringList path_;
    Configuration* config_;
    QTreeWidgetItem* tree_item_;
    std::string description_;
};

class ConfigBool : public ConfigItem {
public:
    ConfigBool(Configuration* tree, const QString& name, bool value = false,
               const std::string& description = "");

    bool value();

    void set_value(bool val) {
        value_ = val;
        setup_item();
    }

    operator bool() { return value(); }

    bool operator=(bool x) {
        set_value(x);
        return x;
    }

    QString to_string() override;
    void set_value_string(const QString& str) override;

protected:
    friend class Configuration;
    void setup_item() override;

    bool value_;
};

class ConfigInt : public ConfigItem {
public:
    ConfigInt(Configuration* config, const QString& name, int value = 0,
              const std::string& description = "");

    QString to_string() override;
    void set_value_string(const QString& str) override;

    operator int() const { return value_; }

    int operator=(int x) {
        set_value(x);
        return x;
    }

    [[nodiscard]] int value() const { return value_; }

    void set_value(int v) {
        value_ = v;
        value_changed(QString::number(v));
    }

protected:
    friend class Configuration;
    int value_;
};

class ConfigDouble : public ConfigItem {
public:
    ConfigDouble(Configuration* config, const QString& name, double value = 0,
                 const std::string& description = "");

    QString to_string() override;
    void set_value_string(const QString& str) override;

    operator double() const { return value_; }

    double operator=(double x) {
        set_value(x);
        return x;
    }

    [[nodiscard]] double value() const { return value_; }

    void set_value(double v) {
        value_ = v;
        value_changed(QString::number(v));
    }

protected:
    friend class Configuration;
    double value_;
};

#define REGISTER_CONFIGURABLE(x) static ConfigurableImpl<x> x##__configurable;

/**
 *
 */
class Configurable {
public:
    Configurable();
    virtual ~Configurable() = default;

    virtual void create_configuration(Configuration* cfg) const = 0;

    static const std::list<Configurable*>& configurables();

private:
    /// Global list of all registered configurables
    static std::list<Configurable*>* configurables_list;
};

/**
 * a template for making configurables
 * the implementing object is responsible for handling configuration
 */
template <class T>
class ConfigurableImpl : public Configurable {
public:
    void create_configuration(Configuration* cfg) const override {
        T::create_configuration(cfg);
    }
};
