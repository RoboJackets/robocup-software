#ifndef _RECORD_HPP_
#define _RECORD_HPP_

#include <list>

#include <Geometry/Point2d.hpp>

class Record
{
public:
    typedef struct
    {
        Geometry::Point2d pos[5];
    } Entry;
    
    Record();

    void write(FILE *fp);
    
    void clear() { records.clear(); }
    unsigned int size() const { return records.size(); }
    
    bool valid[5];
    std::list<Entry> records;
};

#endif // _RECORD_HPP_
