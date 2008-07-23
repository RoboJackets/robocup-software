#include "Condition.hpp"
#include "Predicate.hpp"

#include <boost/foreach.hpp>

bool Condition::Predicate_Ref::evaluate() const
{
    if (inverted)
    {
        return !predicate->evaluate();
    } else {
        return predicate->evaluate();
    }
}

void Condition::add(Predicate *predicate, bool inverted)
{
    _predicates.push_back(Predicate_Ref(predicate, inverted));
}

bool Condition::all() const
{
    BOOST_FOREACH(const Predicate_Ref &ref, _predicates)
    {
        if (!ref.evaluate())
        {
            return false;
        }
    }
    
    return true;
}

bool Condition::any() const
{
    BOOST_FOREACH(const Predicate_Ref &ref, _predicates)
    {
        if (ref.evaluate())
        {
            return true;
        }
    }
    
    return false;
}
