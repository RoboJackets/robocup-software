//FIXME - The + on rawVision flickers, and sometimes stays off, even though it never loses all its children.

#include "ProtobufTree.hpp"
#include <ProtobufTree.moc>

#include <QMenu>
#include <QContextMenuEvent>

#include <stdio.h>
#include <boost/foreach.hpp>
#include <google/protobuf/descriptor.h>

using namespace std;
using namespace google::protobuf;

// Map from protobuf field ID to tree item
typedef QMap<int, QTreeWidgetItem *> FieldMap;
Q_DECLARE_METATYPE(FieldMap)

// Roles
enum
{
	FieldMapRole = Qt::UserRole,	// Column_Tag: Holds a FieldMap for this item's children
	IsMessageRole					// Column_Tag: true if this item is a message
};

ProtobufTree::ProtobufTree(QWidget *parent):
	QTreeWidget(parent)
{
	setColumnHidden(ProtobufTree::Column_Tag, true);
}

bool ProtobufTree::message(const google::protobuf::Message& msg)
{
	bool first = (topLevelItemCount() == 0);
	
	// Update items
	bool ret = addTreeData(invisibleRootItem(), msg);
	
	// If this was the first time items were added, resize all columns
	if (first && topLevelItemCount())
	{
		for (int i = 0; i < columnCount(); ++i)
		{
			resizeColumnToContents(i);
		}
	}
	
	return ret;
}

bool ProtobufTree::addTreeData(QTreeWidgetItem *parent, const google::protobuf::Message& msg)
{
	const Reflection *ref = msg.GetReflection();
	
	// Get fields in the message
	vector<const FieldDescriptor *> fields;
	ref->ListFields(msg, &fields);
	
	// Get map of field numbers to child items
	FieldMap fieldMap = parent->data(Column_Field, FieldMapRole).value<FieldMap>();
	
	bool newFields = false;
	
	// Clear data for missing fields
	const Descriptor *desc = msg.GetDescriptor();
	for (FieldMap::const_iterator i = fieldMap.begin(); i != fieldMap.end(); ++i)
	{
		const FieldDescriptor *field = desc->FindFieldByNumber(i.key());
		if (!field)
		{
			// Field has left the decriptor - should never happen
			printf("Lost field %s.%d\n", desc->name().c_str(), i.key());
			continue;
		}
		
		QTreeWidgetItem *item = i.value();
		
		bool hasData;
		if (field->is_repeated())
		{
			hasData = ref->FieldSize(msg, field);
			
			if (!hasData && item->childCount())
			{
				// Remove and delete children
				BOOST_FOREACH(QTreeWidgetItem *child, item->takeChildren())
				{
					delete child;
				}
			}
		} else {
			hasData = ref->HasField(msg, field);
		}
		
		if (!hasData)
		{
			item->setText(Column_Value, QString());
		}
	}
	
	BOOST_FOREACH(const FieldDescriptor *field, fields)
	{
		// Get the item for this field if the field has been seen before
		//FIXME - This looks up in the map twice
		QTreeWidgetItem *item;
		if (fieldMap.contains(field->number()))
		{
			// Field is already in parent
			item = fieldMap[field->number()];
		} else {
			// New field
			item = new QTreeWidgetItem(parent);
			fieldMap[field->number()] = item;
			
			parent->setData(Column_Field, FieldMapRole, QVariant::fromValue<FieldMap>(fieldMap));
			item->setData(Column_Tag, Qt::DisplayRole, field->number());
			item->setText(Column_Field, QString::fromStdString(field->name()));
			
			if (field->type() == FieldDescriptor::TYPE_MESSAGE && !field->is_repeated())
			{
				// Singular messages are expanded by default
				expandItem(item);
			}
			
			newFields = true;
		}
		
		if (field->is_repeated())
		{
			// Repeated field
			int n = ref->FieldSize(msg, field);
			
			// Show the number of elements as the value for the field itself
			item->setData(Column_Value, Qt::DisplayRole, n);
			
			// Make sure we have enough children
			int children = item->childCount();
			if (children < n)
			{
				// Add children
				for (int i = children; i < n; ++i)
				{
					QTreeWidgetItem *child = new QTreeWidgetItem(item);
					child->setText(Column_Field, QString("[%1]").arg(i));
					
					// For repeated items, the tag column holds the index in the field
					child->setData(Column_Tag, Qt::DisplayRole, i);
					
					// A FieldMap is not used here because the items don't actually have tags.
					// The item's position in its parent is its position in the repeated field.
				}
				
				newFields = true;
			} else if (children > n)
			{
				// Remove excess children
				// Internally, QTreeWidgetItem stores a QList of children.
				// Hopefully this is efficient.
				QList<QTreeWidgetItem *> kids = item->takeChildren();
				for (int i = 0; i < (children - n); ++i)
				{
					delete kids.back();
					kids.pop_back();
				}
				item->addChildren(kids);
			}
			
			// Set data for children
			for (int i = 0; i < n; ++i)
			{
				QTreeWidgetItem *child = item->child(i);
				
				switch (field->type())
				{
					case FieldDescriptor::TYPE_INT32:
					case FieldDescriptor::TYPE_SINT32:
					case FieldDescriptor::TYPE_FIXED32:
					case FieldDescriptor::TYPE_SFIXED32:
						child->setData(Column_Value, Qt::DisplayRole, ref->GetRepeatedInt32(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_INT64:
					case FieldDescriptor::TYPE_SINT64:
					case FieldDescriptor::TYPE_FIXED64:
					case FieldDescriptor::TYPE_SFIXED64:
						child->setData(Column_Value, Qt::DisplayRole, (qlonglong)ref->GetRepeatedInt64(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_UINT32:
						child->setData(Column_Value, Qt::DisplayRole, ref->GetRepeatedUInt32(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_UINT64:
						child->setData(Column_Value, Qt::DisplayRole, (qulonglong)ref->GetRepeatedUInt64(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_FLOAT:
						child->setData(Column_Value, Qt::DisplayRole, ref->GetRepeatedFloat(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_DOUBLE:
						child->setData(Column_Value, Qt::DisplayRole, ref->GetRepeatedDouble(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_BOOL:
						child->setCheckState(Column_Value, ref->GetRepeatedBool(msg, field, i) ? Qt::Checked : Qt::Unchecked);
						break;
					
					case FieldDescriptor::TYPE_ENUM:
					{
						const EnumValueDescriptor *ev = ref->GetRepeatedEnum(msg, field, i);
						child->setText(Column_Value, QString::fromStdString(ev->name()));
						break;
					}
					
					case FieldDescriptor::TYPE_STRING:
						child->setText(Column_Value, QString::fromStdString(ref->GetRepeatedString(msg, field, i)));
						break;
					
					case FieldDescriptor::TYPE_MESSAGE:
						child->setData(Column_Tag, IsMessageRole, true);
						newFields |= addTreeData(child, ref->GetRepeatedMessage(msg, field, i));
						break;
					
					case FieldDescriptor::TYPE_BYTES:
						addBytes(child, ref->GetRepeatedString(msg, field, i));
						break;
					
					default:
						child->setText(Column_Value, QString("??? %1").arg(field->type()));
						break;
				}
			}
		} else switch (field->type())
		{
			case FieldDescriptor::TYPE_INT32:
			case FieldDescriptor::TYPE_SINT32:
			case FieldDescriptor::TYPE_FIXED32:
			case FieldDescriptor::TYPE_SFIXED32:
				item->setData(Column_Value, Qt::DisplayRole, ref->GetInt32(msg, field));
				break;
			
			case FieldDescriptor::TYPE_INT64:
			case FieldDescriptor::TYPE_SINT64:
			case FieldDescriptor::TYPE_FIXED64:
			case FieldDescriptor::TYPE_SFIXED64:
				item->setData(Column_Value, Qt::DisplayRole, (qlonglong)ref->GetInt64(msg, field));
				break;
			
			case FieldDescriptor::TYPE_UINT32:
				item->setData(Column_Value, Qt::DisplayRole, ref->GetUInt32(msg, field));
				break;
			
			case FieldDescriptor::TYPE_UINT64:
				item->setData(Column_Value, Qt::DisplayRole, (qulonglong)ref->GetUInt64(msg, field));
				break;
			
			case FieldDescriptor::TYPE_FLOAT:
				item->setData(Column_Value, Qt::DisplayRole, ref->GetFloat(msg, field));
				break;
			
			case FieldDescriptor::TYPE_DOUBLE:
				item->setData(Column_Value, Qt::DisplayRole, ref->GetDouble(msg, field));
				break;
			
			case FieldDescriptor::TYPE_BOOL:
				//FIXME - Disable user operation of the checkbox
				item->setCheckState(Column_Value, ref->GetBool(msg, field) ? Qt::Checked : Qt::Unchecked);
				break;
			
			case FieldDescriptor::TYPE_ENUM:
			{
				const EnumValueDescriptor *ev = ref->GetEnum(msg, field);
				item->setText(Column_Value, QString::fromStdString(ev->name()));
				break;
			}
			
			case FieldDescriptor::TYPE_STRING:
				item->setText(Column_Value, QString::fromStdString(ref->GetString(msg, field)));
				break;
			
			case FieldDescriptor::TYPE_MESSAGE:
				item->setData(Column_Tag, IsMessageRole, true);
				newFields |= addTreeData(item, ref->GetMessage(msg, field));
				break;
			
			case FieldDescriptor::TYPE_BYTES:
				addBytes(item, ref->GetString(msg, field));
				break;
			
			default:
				item->setText(Column_Value, QString("??? %1").arg(field->type()));
				break;
		}
	}
	
	return newFields;
}

void ProtobufTree::addBytes(QTreeWidgetItem* parent, const std::string& bytes)
{
	int n = bytes.size();
	parent->setText(Column_Value, QString("%1 bytes").arg(n));
	
	int children = parent->childCount();
	if (children < n)
	{
		// Add children
		for (int i = children; i < n; ++i)
		{
			QTreeWidgetItem *child = new QTreeWidgetItem(parent);
			child->setText(Column_Field, QString("[%1]").arg(i));
			
			// For bytes, the tag column holds the index in the field
			child->setData(Column_Tag, Qt::DisplayRole, i);
		}
	} else if (children > n)
	{
		// Remove children
		for (int i = n; i < children; ++i)
		{
			parent->removeChild(parent->child(i));
		}
	}
	
	// Set data
	for (int i = 0; i < n; ++i)
	{
		QTreeWidgetItem *item = parent->child(i);
		QString text;
		text.sprintf("0x%02x", bytes[i]);
		item->setText(Column_Value, text);
	}
}

void ProtobufTree::expandMessages(QTreeWidgetItem* item)
{
	if (!item)
	{
		item = invisibleRootItem();
	}
	
	expandItem(item);
	
	for (int i = 0; i < item->childCount(); ++i)
	{
		QTreeWidgetItem *child = item->child(i);
		
		if (child->data(Column_Tag, IsMessageRole).toBool())
		{
			expandMessages(child);
		}
	}
}

void ProtobufTree::expandSubtree(QTreeWidgetItem* item)
{
	expandItem(item);
	for (int i = 0; i < item->childCount(); ++i)
	{
		QTreeWidgetItem *child = item->child(i);
		expandSubtree(child);
	}
}

void ProtobufTree::collapseSubtree(QTreeWidgetItem* item)
{
	collapseItem(item);
	for (int i = 0; i < item->childCount(); ++i)
	{
		QTreeWidgetItem *child = item->child(i);
		collapseSubtree(child);
	}
}

void ProtobufTree::contextMenuEvent(QContextMenuEvent* e)
{
	QMenu menu;
	
	QAction *expandItemAction = 0, *collapseItemAction = 0;
	QTreeWidgetItem *item = itemAt(e->pos());
	if (item)
	{
		expandItemAction = menu.addAction("Expand");
		collapseItemAction = menu.addAction("Collapse");
		menu.addSeparator();
	}
	
	QAction *expandMessagesAction = menu.addAction("Expand Only Messages");
	menu.addSeparator();
	
	QAction *expandAction = menu.addAction("Expand All");
	QAction *collapseAction = menu.addAction("Collapse All");
	menu.addSeparator();
	
	QAction *showTags = menu.addAction("Show tags");
	showTags->setCheckable(true);
	showTags->setChecked(!isColumnHidden(Column_Tag));
	
	QAction *act = menu.exec(mapToGlobal(e->pos()));
	if (act == expandMessagesAction)
	{
		collapseAll();
		expandMessages();
	} else if (act == expandAction)
	{
		expandAll();
	} else if (act == collapseAction)
	{
		collapseAll();
	} else if (act == showTags)
	{
		setColumnHidden(Column_Tag, !showTags->isChecked());
	} else if (expandItemAction && act == expandItemAction)
	{
		expandSubtree(item);
	} else if (collapseItemAction && act == collapseItemAction)
	{
		collapseSubtree(item);
	}
}
