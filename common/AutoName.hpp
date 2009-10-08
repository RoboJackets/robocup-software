#pragma once

#include <Utils.hpp>
#include <typeinfo>

class AutoName
{
	public:
		virtual ~AutoName()
		{
		}
		
		const std::string &name()
		{
			// We get the name automatically from type_info, but the object doesn't have the proper type when Play::Play() is called
			// because the subclass' vtable isn't in place.  Thus we have to find the name lazily.
			if (_name.empty())
			{
				_name = Utils::className(typeid(*this)).toStdString();
			}
			
			return _name;
		}
		
	private:
		std::string _name;
};
