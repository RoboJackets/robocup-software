#ifndef _PACKET_HPP
#define _PACKET_HPP

#include <QDomDocument>
#include <QVector>
#include <QString>
#include <QStringList>
#include <QTextStream>

class StyleStream
{
	public:
		StyleStream(QString* string) :
			_stream(string), _tabs(0)
		{
		}
		
		StyleStream(QIODevice* device) :
			_stream(device), _tabs(0)
		{
		}
		
		StyleStream& operator<< (const char* in)
		{
			QString input(in);
			
			if (input.contains('{'))
			{
				_tabs++;
			}
			
			if (input.contains('}'))
			{
				_stream << "\n";
				_tabs--;
				for (unsigned int i=0 ; i<_tabs ; ++i)
				{
					_stream << "    ";
				}
			}
			
			_stream << input;
			
			if (input.contains('\n'))
			{
				for (unsigned int i=0 ; i<_tabs ; ++i)
				{
					_stream << "    ";
				}
			}
			
			return *this;
		}
		
		template <typename T>
		StyleStream& operator<< (T in)
		{
			_stream << in;
			return *this;
		}
		
		QTextStream _stream;
		
		unsigned int _tabs;
};

class Packet
{
	private:
		/** represents a c++ type */
		class Type
		{
			public:
				QString cType();
				
				bool hasInclude() const
				{
					return (!include.isEmpty());
				}
				
				QString getInclude() const
				{
					return include;
				}
				
			public:
				/** helper for creating built in types */
				Type(QString n = "", QString dVal = "") : 
					name(n), defaultValue(dVal) {}
				
				/** the name of the type (after the :: part of namespace)*/
				QString name;
				
				/** the namespace of the type (if there is one) */
				QString ns;
				
				/** the include file for the type */
				QString include;
				
				/** the default value for the type */
				QString defaultValue;
		};
		
		class Member
		{
			public:
				Member() : staticMem(false) {}
				
				QString cInitializer();
				
				QString cMember();
				
			public:
				/** the type of the member */
				Type type;
				
				/** member name */
				QString name;
				
				/** quantity qualifier */
				QString qty;
				
				/** the value for the member */
				QString value;
				
				/** if true the member is static */
				bool staticMem;
		};
		
		class Enum
		{
			public:
				void print(StyleStream& out);
			
			public:
				/** enum name */
				QString name;
				
				/** list of entries */
				QStringList items;
		};
		
		/** an object contains members, custom enums, and more objects */
		class Object
		{
			public:
				Object() : parent(0) {}
				
				/** returns the c style typename for the object */
				QString cType();
				
				void print(StyleStream& out);
			
			public:
				/** the name of the object */
				QString name;
				
				/** objects within this object */
				QVector<Object> objects;
				
				/** enums in this type */
				QVector<Enum> enums;
				
				/** members in this type */
				QVector<Member> members;
				
				Object* parent;
		};
	
	public:
		
		/** packets can only be created through parsing */
		static Packet parse(QDomDocument doc);
		
		void print(StyleStream& out);
		
	private:
		Packet();
		Packet& operator&=(Packet&);
		
		/** parses a new type into the packet */
		Type parseType(QDomElement& e);
		
		Object parseObject(QDomElement& e, Object* parent = 0);
		
		/** parses and creates a new enum */
		Enum parseEnum(QDomElement& e);
		
		/** process a member */
		Member parseMember(QDomElement& e);
		
	/// members ///
	private:
		/** name of the packet */
		QString _name;
		
		/** unique packet id (this becomes the port offset)*/
		QString _uid;
		
		/** types used in this packet */
		QVector<Type> _types;
		
		/** inside objects */
		QVector<Object> _objects;
};

#endif /* _PACKET_HPP */
