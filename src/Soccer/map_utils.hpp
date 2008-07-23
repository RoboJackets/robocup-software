#ifndef _MAP_UTILS_H_
#define _MAP_UTILS_H_

// Removes all entries in a std::map which associate to the given value.
template<class Map_Type, class Data_Type>
void map_remove(Map_Type &map, Data_Type &value)
{
    typename Map_Type::iterator i = map.begin();
    while (i != map.end())
    {
        typename Map_Type::iterator next = i;
        ++next;
        
        if (i->second == value)
        {
            map.erase(i);
        }
        
        i = next;
    }
}

// If <key> exists in <map>, returns map[key].
// If not, returns 0.
template<typename Map>
typename Map::mapped_type map_lookup(const Map &map, typename Map::key_type key)
{
    typename Map::const_iterator i = map.find(key);
    if (i != map.end())
    {
        return i->second;
    } else {
        return 0;
    }
}

#endif // _MAP_UTILS_H_
