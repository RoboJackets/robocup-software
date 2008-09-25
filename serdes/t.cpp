#include <stdio.h>

class Ser
{
public:
    void operator&(int i)
    {
        printf("int %d\n", i);
    }
};

class Thing
{
public:
    class Inner
    {
    public:
    };
    
    Inner i;
};

void serdes(Ser &buf, Thing::Inner &i)
{
    printf("Thing::Inner\n");
}

void serdes(Ser &buf, Thing &t)
{
    printf("Thing\n");
    serdes(buf, t.i);
}

int main()
{
    Ser buf;
    
    Thing t;
    serdes(buf, t);
    
    return 0;
}
