#include "Packet.hpp"

#include <QDomElement>
#include <QDomNode>

#include <stdexcept>

Packet::Packet()
{
	
}

Packet Packet::parse(QDomDocument doc)
{
	Packet p;
	
	//add built in types
	p._types.append(Type("int", "0"));
	p._types.append(Type("bool", "false"));
	p._types.append(Type("float", "0"));
	p._types.append(Type("char", "0"));
	
	//stdint types
	p._types.append(Type("int8_t", "0"));
	p._types.append(Type("int24_t", "0"));
	p._types.append(Type("int32_t", "0"));
	p._types.append(Type("int64_t", "0"));
	p._types.append(Type("uint8_t", "0"));
	p._types.append(Type("uint24_t", "0"));
	p._types.append(Type("uint32_t", "0"));
	p._types.append(Type("uint64_t", "0"));
	
	
	QDomElement e = doc.documentElement();
	
	//get attributes for the whole packet
	p._name = e.attribute("name");
	p._uid = e.attribute("uid");
	
	//the rest of the packet can be treated like the internals of a type/object
	Object o = p.parseObject(e);
	o.name = p._name;
	
	//the packet object needs an extra static member for the port
	Member type;
	type.name = "Type";
	type.type = Type("int", "0");
	type.value = p._uid;
	type.staticMem = true;
	
	o.members.append(type);
	
	p._objects.append(o);
	
	return p;
}

Packet::Type Packet::parseType(QDomElement& e)
{
	QString name = e.attribute("name");
	
	//this is an include type, not an object
	Type t;
	
	t.name = e.attribute("name");
	t.include = e.attribute("file");
	t.ns = e.attribute("namespace");
	t.defaultValue = e.attribute("default");
		
	printf("\tNew Type: %s\n", t.name.toStdString().c_str());
		
	return t;
}

Packet::Object Packet::parseObject(QDomElement& e, Object* parent)
{
	//if not an include type, then an internal object
	Object o;
	o.name = e.attribute("name");
	o.parent = parent;
	
	//add the object as an available type
	Type t;
	t.name = o.name;
	
	if (parent)
	{
		t.ns = parent->cType();
	}
	
	_types.append(t);
	
	QDomNode n = e.firstChild();
	while (!n.isNull())
	{
		if (n.isElement())
		{
			QDomElement elem = n.toElement();	
			
			if (elem.tagName() == "type")
			{
				//if this was not an include, then parse the insides
				if (elem.hasAttribute("file"))
				{
					_types.append(parseType(elem));
				}
				else
				{
					o.objects.append(parseObject(elem, &o));
				}
			}
			else if (elem.tagName() == "enum")
			{
				Enum en = parseEnum(elem);
				
				//add the enum as a type
				Type enumType;
				enumType.name = en.name;
				enumType.ns = o.name;
				_types.append(enumType);
				
				o.enums.append(en);
			}
			else
			{
				//otherwise assume its a member
				o.members.append(parseMember(elem));
			}
		}
			
		n = n.nextSibling();
	}
	
	return o;
}

Packet::Enum Packet::parseEnum(QDomElement& e)
{
	Enum en;
	en.name = e.attribute("name");
	
	QDomNode n = e.firstChild();
	while (!n.isNull())
	{
		if (n.isElement())
		{
			QDomElement elem = n.toElement();
			
			if (elem.tagName() != "item")
			{
				//TODO throw exception
			}
			
			en.items.append(elem.attribute("name"));
		}
		
		n = n.nextSibling();
	}
	
	printf("\tNew Enum '%s' : {", en.name.toStdString().c_str());
	
	Q_FOREACH(QString item, en.items)
	{
		printf(" %s ", item.toStdString().c_str());
	}
	
	printf("}\n");
	return en;
}

Packet::Member Packet::parseMember(QDomElement& e)
{
	QString type = e.tagName();
	
	Member m;
	m.name = e.attribute("name");
	m.qty = e.attribute("qty");
	m.value = e.attribute("value");
	
	Q_FOREACH(Type t, _types)
	{
		if (t.name == type)
		{
			m.type = t;
			
			printf("\tNew Member: '%s' of Type: '%s'\n", 
				   m.name.toStdString().c_str(), t.cType().toStdString().c_str());
			return m;
		}
	}
	
	QString error = "Error: Type '" + type + "' not found";
	throw std::runtime_error(error.toAscii().data());
}

QString Packet::Object::cType()
{
	QString tName;
					
	if (parent)
	{
		tName += parent->cType() + "::";
	}
					
	tName += name;
					
	return tName;
}

void Packet::Enum::print(StyleStream& out)
{
	out << "typedef enum\n";
	out <<"{\n";
					
	for (int i=0 ; i<items.size() ; ++i)
	{
		out << items[i];
						
		if (i < (items.size()-1))
		{
			out << ",\n";
		}
		else
		{
			out << "\n";
		}
	}
					
	out << "} " << name << ";\n\n";
}

void Packet::print(StyleStream& out)
{
	QStringList includes;
			
	//header guards
	QString guard = "_" + _name.toUpper() + "_HPP";
	out << "#ifndef " << guard << "\n";
	out << "#define " << guard << "\n\n";
	
	//always include vector and stdint
	out << "#include <vector>\n";
	out << "#include <stdint.h>\n";
	
	out << "\n";
	
	Q_FOREACH(Type t, _types)
	{
		QString inc = t.getInclude();
		if (t.hasInclude() && !includes.contains(inc))
		{
			out << "#include <" << t.getInclude() << ">\n";
			includes.append(inc);
		}
	}
	
	out << "\n";
	
	out << "namespace Packet\n";
	out << "{\n";
			
	Q_FOREACH(Object o, _objects)
	{
		o.print(out);
	}
	
	out << "}\n\n";
	
	out << "#endif /* " << guard << " */\n";
}

void Packet::Object::print(StyleStream& out)
{
	out << "class " << name << "\n";
	out << "{\n";
	out << "public:\n\n";
					
					//print internal enums
	Q_FOREACH(Enum e, enums)
	{
		e.print(out);
	}
					
					//print internal objects
	Q_FOREACH(Object o, objects)
	{
		o.print(out);
	}
					
	out << name << "()";
					
					//setup initializer list
	QStringList initialize;
	Q_FOREACH(Member m, members)
	{
		QString init = m.cInitializer();
		if (!init.isEmpty())
		{
			initialize.append(init);
		}
	}
					
	for (int i=0 ; i<initialize.size() ; ++i)
	{
		if (i == 0)
		{
			out << " :\n";
		}

		if (i > 0)
		{
			out << ", ";
		}
						
		out << initialize[i];
	}
					
	out << "{}\n\n";
					
					//TODO (de)serialization method
					
					//print the members
	Q_FOREACH(Member m, members)
	{
		out << m.cMember() << "\n\n";
	}
					
	out << "} __attribute__((__packed__));\n\n";
}

QString Packet::Member::cMember()
{
	QString cStyle;
					
	if (staticMem)
	{
		cStyle += "static const ";
	}
					
	if (qty == "*")
	{
		cStyle += "std::vector<" + type.cType() + "> ";
	}
	else
	{
		cStyle += type.cType() + " ";
	}
					
	cStyle += name;
					
	if (!qty.isEmpty() && qty != "*")
	{
		cStyle += "[" + qty + "]";
	}
					
	if (staticMem)
	{
		cStyle += " = " + value;
	}
					
	cStyle += ";";
					
	return cStyle;
}

QString Packet::Member::cInitializer()
{
	//if there is a qty, don't initialize
	//otherwise create initializer
	if (qty.isEmpty() && !staticMem)
	{
		QString initValue;
						
		if (value.isEmpty() && !type.defaultValue.isEmpty())
		{
			initValue = type.defaultValue;
		}
		else if (!value.isEmpty())
		{
			initValue = value;
		}
						
		if (!initValue.isEmpty())
		{
			return name + "(" + initValue + ")";
		}
	}
					
	return "";
}

QString Packet::Type::cType()
{
	QString cType;
					
	if (!ns.isEmpty())
	{
		cType += ns + "::";
	}
					
	cType += name;
					
	return cType;
}
