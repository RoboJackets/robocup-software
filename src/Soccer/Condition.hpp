#ifndef _CONDITION_H_
#define _CONDITION_H_

#include <list>

class Predicate;

// A condition evaluates a set of predicates.
class Condition
{
public:
    void add(Predicate *predicate, bool inverted = false);
    
    // Returns true if all predicates are true (AND).
    bool all() const;
    
    // Returns true if any predicates are true (OR).
    bool any() const;

protected:
    class Predicate_Ref
    {
    public:
        Predicate_Ref(Predicate *pred, bool inv)
        {
            predicate = pred;
            inverted = inv;
        }
        
        bool evaluate() const;
        
        Predicate *predicate;
        bool inverted;
    };
    
    std::list<Predicate_Ref> _predicates;
};

#endif // _CONDITION_H_
