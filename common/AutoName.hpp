#pragma once

#include <Utils.hpp>
#include <typeinfo>

/// Inherit from this class to get a name() function that will return the name of the class.
class AutoName
{
	public:
		virtual ~AutoName()
		{
		}
		/**
		 * gets the name of the class
		 */
		const QString &name()
		{
			// We get the name automatically from type_info, but the object doesn't have the proper type when the constructor is called
			// because the subclass' vtable isn't in place.  Thus we have to find the name lazily.
			if (_name.isNull())
			{
				_name = className(typeid(*this));
			}
			
			return _name;
		}
		
	private:
		QString _name;
};
