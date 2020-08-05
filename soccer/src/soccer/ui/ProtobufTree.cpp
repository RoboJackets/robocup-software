#include "ProtobufTree.hpp"

#include <cstdio>
#include <iostream>

#include <QContextMenuEvent>
#include <QDockWidget>
#include <QMainWindow>
#include <QMenu>
#include <QTimer>
#include <google/protobuf/descriptor.h>

#include "StripChart.hpp"

using namespace std;
using namespace google::protobuf;

// Map from protobuf field ID to tree item
using FieldMap = QMap<int, QTreeWidgetItem*>;
Q_DECLARE_METATYPE(FieldMap)
Q_DECLARE_METATYPE(const FieldDescriptor*)

// Roles
enum {
    FieldMapRole =
        Qt::UserRole,  // Column_Tag: Holds a FieldMap for this item's children
    IsMessageRole,     // Column_Tag: true if this item is a message
    FieldDescriptorRole  // Column_Tag: FieldDescriptor* for this field, if
                         // applicable
};

ProtobufTree::ProtobufTree(QWidget* parent) : QTreeWidget(parent) {
    _first = true;
    _history = nullptr;
    mainWindow = nullptr;
    updateTimer = nullptr;
}

bool ProtobufTree::message(const google::protobuf::Message& msg) {
    // Update items
    bool ret = addTreeData(invisibleRootItem(), msg);

    // If this was the first time items were added, resize all columns
    if (_first && ret) {
        _first = false;

        for (int i = 0; i < columnCount(); ++i) {
            resizeColumnToContents(i);
        }

        setColumnHidden(ProtobufTree::Column_Tag, true);
    }

    return ret;
}

bool ProtobufTree::addTreeData(QTreeWidgetItem* parent,
                               const google::protobuf::Message& msg) {
    const Reflection* ref = msg.GetReflection();

    // Get fields in the message
    vector<const FieldDescriptor*> fields;
    ref->ListFields(msg, &fields);

    // Get map of field numbers to child items
    auto field_map = parent->data(Column_Tag, FieldMapRole).value<FieldMap>();

    bool new_fields = false;

    // Clear data for missing fields
    const Descriptor* desc = msg.GetDescriptor();
    for (FieldMap::const_iterator i = field_map.begin(); i != field_map.end();
         ++i) {
        const FieldDescriptor* field = desc->FindFieldByNumber(i.key());
        if (field == nullptr) {
            // Field has left the descriptor - should never happen
            printf("Lost field %s.%d\n", desc->name().c_str(), i.key());
            continue;
        }

        QTreeWidgetItem* item = i.value();

        bool has_data;
        if (field->is_repeated()) {
            has_data = (ref->FieldSize(msg, field) != 0);

            if (!has_data && (item->childCount() != 0)) {
                // Remove and delete children
                for (QTreeWidgetItem* child : item->takeChildren()) {
                    delete child;
                }
            }
        } else {
            has_data = ref->HasField(msg, field);
        }

        if (!has_data) {
            item->setText(Column_Value, QString());
            item->setData(Column_Value, Qt::CheckStateRole, QVariant());
        }
    }

    for (const FieldDescriptor* field : fields) {
        // Get the item for this field if the field has been seen before
        QTreeWidgetItem* item;
        FieldMap::iterator field_iter = field_map.find(field->number());
        if (field_iter != field_map.end()) {
            // Field is already in parent
            item = *field_iter;
        } else {
            // New field
            item = new QTreeWidgetItem(parent);
            field_map.insert(field->number(), item);

            item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
            parent->setData(Column_Tag, FieldMapRole,
                            QVariant::fromValue<FieldMap>(field_map));
            item->setData(Column_Tag, Qt::DisplayRole, field->number());
            item->setData(Column_Tag, FieldDescriptorRole,
                          QVariant::fromValue(field));
            item->setText(Column_Field, QString::fromStdString(field->name()));

            if (field->type() == FieldDescriptor::TYPE_MESSAGE &&
                !field->is_repeated()) {
                // Singular messages are expanded by default
                expandItem(item);
            }

            new_fields = true;
        }

        if (field->is_repeated()) {
            // Repeated field
            int n = ref->FieldSize(msg, field);

            // Show the number of elements as the value for the field itself
            item->setData(Column_Value, Qt::DisplayRole, n);

            // Make sure we have enough children
            int children = item->childCount();
            if (children < n) {
                // Add children
                for (int i = children; i < n; ++i) {
                    auto* child = new QTreeWidgetItem(item);
                    child->setText(Column_Field, QString("[%1]").arg(i));

                    child->setData(Column_Tag, FieldDescriptorRole,
                                   field != nullptr);

                    // For repeated items, the tag column holds the index in the
                    // field
                    child->setData(Column_Tag, Qt::DisplayRole, i);

                    // A FieldMap is not used here because the items don't
                    // actually have tags. The item's position in its parent is
                    // its position in the repeated field.
                }

                new_fields = true;
            } else if (children > n) {
                // Remove excess children
                // Internally, QTreeWidgetItem stores a QList of children.
                // Hopefully this is efficient.
                QList<QTreeWidgetItem*> kids = item->takeChildren();
                for (int i = 0; i < (children - n); ++i) {
                    delete kids.back();
                    kids.pop_back();
                }
                item->addChildren(kids);
            }

            // Set data for children
            for (int i = 0; i < n; ++i) {
                QTreeWidgetItem* child = item->child(i);

                switch (field->type()) {
                    case FieldDescriptor::TYPE_INT32:
                    case FieldDescriptor::TYPE_SINT32:
                    case FieldDescriptor::TYPE_FIXED32:
                    case FieldDescriptor::TYPE_SFIXED32:
                        child->setData(Column_Value, Qt::DisplayRole,
                                       ref->GetRepeatedInt32(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_INT64:
                    case FieldDescriptor::TYPE_SINT64:
                    case FieldDescriptor::TYPE_FIXED64:
                    case FieldDescriptor::TYPE_SFIXED64:
                        child->setData(
                            Column_Value, Qt::DisplayRole,
                            (qlonglong)ref->GetRepeatedInt64(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_UINT32:
                        child->setData(Column_Value, Qt::DisplayRole,
                                       ref->GetRepeatedUInt32(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_UINT64:
                        child->setData(
                            Column_Value, Qt::DisplayRole,
                            (qulonglong)ref->GetRepeatedUInt64(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_FLOAT:
                        child->setData(Column_Value, Qt::DisplayRole,
                                       ref->GetRepeatedFloat(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_DOUBLE:
                        child->setData(Column_Value, Qt::DisplayRole,
                                       ref->GetRepeatedDouble(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_BOOL:
                        child->setCheckState(Column_Value,
                                             ref->GetRepeatedBool(msg, field, i)
                                                 ? Qt::Checked
                                                 : Qt::Unchecked);
                        break;

                    case FieldDescriptor::TYPE_ENUM: {
                        const EnumValueDescriptor* ev =
                            ref->GetRepeatedEnum(msg, field, i);
                        child->setText(Column_Value,
                                       QString::fromStdString(ev->name()));
                        break;
                    }

                    case FieldDescriptor::TYPE_STRING:
                        child->setText(Column_Value, QString::fromStdString(
                                                         ref->GetRepeatedString(
                                                             msg, field, i)));
                        break;

                    case FieldDescriptor::TYPE_MESSAGE:
                        child->setData(Column_Tag, IsMessageRole, true);
                        new_fields |= addTreeData(
                            child, ref->GetRepeatedMessage(msg, field, i));
                        break;

                    case FieldDescriptor::TYPE_BYTES:
                        addBytes(child, ref->GetRepeatedString(msg, field, i));
                        break;

                    default:
                        child->setText(Column_Value,
                                       QString("??? %1").arg(field->type()));
                        break;
                }
            }
        } else {
            switch (field->type()) {
                case FieldDescriptor::TYPE_INT32:
                case FieldDescriptor::TYPE_SINT32:
                case FieldDescriptor::TYPE_FIXED32:
                case FieldDescriptor::TYPE_SFIXED32:
                    item->setData(Column_Value, Qt::DisplayRole,
                                  ref->GetInt32(msg, field));
                    break;

                case FieldDescriptor::TYPE_INT64:
                case FieldDescriptor::TYPE_SINT64:
                case FieldDescriptor::TYPE_FIXED64:
                case FieldDescriptor::TYPE_SFIXED64:
                    item->setData(Column_Value, Qt::DisplayRole,
                                  (qlonglong)ref->GetInt64(msg, field));
                    break;

                case FieldDescriptor::TYPE_UINT32:
                    item->setData(Column_Value, Qt::DisplayRole,
                                  ref->GetUInt32(msg, field));
                    break;

                case FieldDescriptor::TYPE_UINT64:
                    item->setData(Column_Value, Qt::DisplayRole,
                                  (qulonglong)ref->GetUInt64(msg, field));
                    break;

                case FieldDescriptor::TYPE_FLOAT:
                    item->setData(Column_Value, Qt::DisplayRole,
                                  ref->GetFloat(msg, field));
                    break;

                case FieldDescriptor::TYPE_DOUBLE:
                    item->setData(Column_Value, Qt::DisplayRole,
                                  ref->GetDouble(msg, field));
                    break;

                case FieldDescriptor::TYPE_BOOL:
                    item->setCheckState(Column_Value, ref->GetBool(msg, field)
                                                          ? Qt::Checked
                                                          : Qt::Unchecked);
                    break;

                case FieldDescriptor::TYPE_ENUM: {
                    const EnumValueDescriptor* ev = ref->GetEnum(msg, field);
                    item->setText(Column_Value,
                                  QString::fromStdString(ev->name()));
                    break;
                }

                case FieldDescriptor::TYPE_STRING:
                    item->setText(
                        Column_Value,
                        QString::fromStdString(ref->GetString(msg, field)));
                    break;

                case FieldDescriptor::TYPE_MESSAGE:
                    item->setData(Column_Tag, IsMessageRole, true);
                    new_fields |=
                        addTreeData(item, ref->GetMessage(msg, field));
                    break;

                case FieldDescriptor::TYPE_BYTES:
                    addBytes(item, ref->GetString(msg, field));
                    break;

                default:
                    item->setText(Column_Value,
                                  QString("??? %1").arg(field->type()));
                    break;
            }
        }
    }

    return new_fields;
}

void ProtobufTree::addBytes(QTreeWidgetItem* parent, const std::string& bytes) {
    int n = bytes.size();
    parent->setText(Column_Value, QString("%1 bytes").arg(n));

    int children = parent->childCount();
    if (children < n) {
        // Add children
        for (int i = children; i < n; ++i) {
            auto* child = new QTreeWidgetItem(parent);
            child->setText(Column_Field, QString("[%1]").arg(i));

            // For bytes, the tag column holds the index in the field
            child->setData(Column_Tag, Qt::DisplayRole, i);
        }
    } else if (children > n) {
        // Remove children
        for (int i = n; i < children; ++i) {
            parent->removeChild(parent->child(i));
        }
    }

    // Set data
    for (int i = 0; i < n; ++i) {
        QTreeWidgetItem* item = parent->child(i);
        QString text;
        text.sprintf("0x%02x", bytes[i]);
        item->setText(Column_Value, text);
    }
}

void ProtobufTree::expandMessages(QTreeWidgetItem* item) {
    if (item == nullptr) {
        item = invisibleRootItem();
    }

    expandItem(item);

    for (int i = 0; i < item->childCount(); ++i) {
        QTreeWidgetItem* child = item->child(i);

        if (child->data(Column_Tag, IsMessageRole).toBool()) {
            expandMessages(child);
        }
    }
}

void ProtobufTree::expandSubtree(QTreeWidgetItem* item) {
    expandItem(item);
    for (int i = 0; i < item->childCount(); ++i) {
        QTreeWidgetItem* child = item->child(i);
        expandSubtree(child);
    }
}

void ProtobufTree::collapseSubtree(QTreeWidgetItem* item) {
    collapseItem(item);
    for (int i = 0; i < item->childCount(); ++i) {
        QTreeWidgetItem* child = item->child(i);
        collapseSubtree(child);
    }
}

void ProtobufTree::contextMenuEvent(QContextMenuEvent* e) {
    QMenu menu;

    QAction *expand_item_action = nullptr, *collapse_item_action = nullptr;
    QTreeWidgetItem* item = itemAt(e->pos());
    if (item != nullptr) {
        expand_item_action = menu.addAction("Expand");
        collapse_item_action = menu.addAction("Collapse");
        menu.addSeparator();
    }

    QAction* expand_messages_action = menu.addAction("Expand Only Messages");
    menu.addSeparator();

    QAction* expand_action = menu.addAction("Expand All");
    QAction* collapse_action = menu.addAction("Collapse All");
    menu.addSeparator();

    QAction* show_tags = menu.addAction("Show tags");
    show_tags->setCheckable(true);
    show_tags->setChecked(!isColumnHidden(Column_Tag));

    menu.addSeparator();

    QAction* chart_action = nullptr;
    QAction* export_action = nullptr;
    QList<QAction*> chart_menu_actions;

    QList<QDockWidget*> dock_widgets;
    const FieldDescriptor* field = nullptr;
    if ((mainWindow != nullptr) && (item != nullptr)) {
        field = item->data(Column_Tag, FieldDescriptorRole)
                    .value<const FieldDescriptor*>();
        if (field != nullptr) {
            int t = field->type();
            if (t == FieldDescriptor::TYPE_FLOAT ||
                t == FieldDescriptor::TYPE_DOUBLE ||
                (t == FieldDescriptor::TYPE_MESSAGE &&
                 field->message_type()->name() == "Point")) {
                dock_widgets = mainWindow->findChildren<QDockWidget*>();
                if (!dock_widgets.isEmpty()) {
                    QMenu* chart_menu = menu.addMenu("Chart");
                    chart_action = chart_menu->addAction("New Chart");
                    export_action = chart_menu->addAction("Export Chart");
                    chart_menu->addSeparator();
                    for (int i = 0; i < dock_widgets.size(); ++i) {
                        chart_menu_actions.append(chart_menu->addAction(
                            QString("Add to '%1'")
                                .arg(dock_widgets[0]->windowTitle())));
                    }
                } else {
                    chart_action = menu.addAction("New Chart");
                }
            }
        }
    }

    QAction* act = menu.exec(mapToGlobal(e->pos()));
    if (act == expand_messages_action) {
        collapseAll();
        expandMessages();
    } else if (act == expand_action) {
        expandAll();
    } else if (act == collapse_action) {
        collapseAll();
    } else if (act == show_tags) {
        setColumnHidden(Column_Tag, !show_tags->isChecked());
    } else if ((expand_item_action != nullptr) && act == expand_item_action) {
        expandSubtree(item);
    } else if ((collapse_item_action != nullptr) &&
               act == collapse_item_action) {
        collapseSubtree(item);
    } else if ((chart_action != nullptr) && act == chart_action) {
        // Find the path from LogFrame to the chosen item
        QVector<int> path;
        QStringList names;
        for (QTreeWidgetItem* i = item; i != nullptr; i = i->parent()) {
            int tag = i->data(Column_Tag, Qt::DisplayRole).toInt();
            path.push_back(tag);
            names.append(i->text(Column_Field));
        }

        reverse(path.begin(), path.end());
        reverse(names.begin(), names.end());

        auto* dock = new QDockWidget(names.join("."), mainWindow);
        auto* chart = new StripChart(dock);
        chart->history(_history);

        if (field->type() == FieldDescriptor::TYPE_MESSAGE) {
            auto* f = new Chart::PointMagnitude;
            f->path = path;
            f->name = names.join(".");
            chart->function(f);
        } else {
            auto* f = new Chart::NumericField;
            f->path = path;
            f->name = names.join(".");
            chart->function(f);
        }
        dock->setAttribute(Qt::WA_DeleteOnClose);
        dock->setWidget(chart);
        mainWindow->addDockWidget(Qt::BottomDockWidgetArea, dock);
        if (updateTimer != nullptr) {
            connect(updateTimer, SIGNAL(timeout()), chart, SLOT(update()));
        }
    } else if ((export_action != nullptr) && act == export_action) {
        // If export button was pressed

        auto* chart = new StripChart();
        chart->history(_history);

        // Loop through all open charts and add their data to one chart
        for (auto& dock_widget : dock_widgets) {
            auto* c_chart = dynamic_cast<StripChart*>(dock_widget->widget());
            QList<Chart::Function*> functions = c_chart->getFunctions();
            for (auto& function : functions) {
                chart->function(function);
            }
        }

        // export that chart
        chart->exportChart();

    } else if (!chart_menu_actions.empty()) {
        int i = chart_menu_actions.indexOf(act);
        if (i != -1) {
            auto* chart = dynamic_cast<StripChart*>(dock_widgets[i]->widget());
            QVector<int> path;
            QStringList names;
            for (QTreeWidgetItem* i = item; i != nullptr; i = i->parent()) {
                int tag = i->data(Column_Tag, Qt::DisplayRole).toInt();
                path.push_back(tag);
                names.append(i->text(Column_Field));
            }

            reverse(path.begin(), path.end());
            reverse(names.begin(), names.end());

            dock_widgets[i]->setWindowTitle(dock_widgets[i]->windowTitle() +
                                            ", " + names.join("."));
            chart->history(_history);

            if (field->type() == FieldDescriptor::TYPE_MESSAGE) {
                auto* f = new Chart::PointMagnitude;
                f->path = path;
                f->name = names.join(".");
                chart->function(f);
            } else {
                auto* f = new Chart::NumericField;
                f->path = path;
                f->name = names.join(".");
                chart->function(f);
            }

            if (updateTimer != nullptr) {
                connect(updateTimer, SIGNAL(timeout()), chart, SLOT(update()));
            }
        }
    }
}
