#pragma once

#include <Utils.hpp>

#include <string>
#include <map>

namespace Gameplay
{
    template<class T>
    class Named_Object
    {
    public:
        Named_Object(const std::string &name)
        {
            _name = name;
            
            if (!_map)
            {
                _map = new Map();
            }

            (*_map)[name] = static_cast<T *>(this);
        }
        
        ~Named_Object()
        {
            // Have to give the template parameters explicitly, as gcc won't infer
            // the "const" on the pointer.
            Utils::map_remove<Map, T* const>(*_map, static_cast<T *>(this));
        }
        
        static T *find(const std::string &name)
        {
            if (_map)
            {
                return Utils::map_lookup(*_map, name);
            } else {
                return 0;
            }
        }

        const std::string &name() const { return _name; }
        
    protected:
        std::string _name;
        
        typedef std::map<std::string, T *> Map;
        static Map *_map;
    };

    template<class T> std::map<std::string, T *> *Named_Object<T>::_map;
}
