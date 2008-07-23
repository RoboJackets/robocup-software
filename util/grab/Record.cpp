#include "Record.hpp"

#include <boost/foreach.hpp>

Record::Record()
{
    for (int i = 0; i < 5; ++i)
    {
        valid[i] = false;
    }
}

void Record::write(FILE *fp)
{
    fprintf(fp, "NAME \"recorded\"\n\n");
    for (int i = 0; i < 5; ++i)
    {
        if (valid[i])
        {
            fprintf(fp, "ROLE @%d\n", i);
            BOOST_FOREACH(const Entry &entry, records)
            {
                fprintf(fp, "    move pos=(%g, %g)\n", entry.pos[i].x, entry.pos[i].y);
            }
        }
    }
}
