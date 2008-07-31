/// The first include should be the primary header
#include "Coding.hpp"

/// Other includes should come next
#include <Other1>
#include "Other2.hpp"

/// namespace usage
using namespace std;

/// static members
char Coding::_privateStatic = 0;

/// ! Openning braces are always on their own line
/// ! Closing braces should be on their own line
///   - two exceptions: else(if), do...while

/// Sample Constructor
/// optinal initializer list should be on next line, colon on prototype line
Coding::Coding() :
        BaseClass(), ...
{
    /// Comments inside implementation DO NOT need to be in doxygen format
    
    for (... ; ... ; ...)
    {
    
    }
    
    do
    {
    
    }
    while (...);
    
    ///otionally the while can be on the same line as the }
    /// } while (...);
    
    while (...)
    {
    
    }
}

Coding::~Coding()
{

}

int privateMember() const
{
    return _privateMember;
}

void privateMember (int val)
{
    /// sample if..elseif...else statement
    /// optinally, the elseif/else can be on same line as closing brace
    if (...)
    {
    }
    else if (...)
    {
    }
    else
    {
    
    }
    
    /// Alternative if/else
    /*
    if (...)
    {
    } else if (...)
    {
    } else
    {
    }
    */
    
    ///switch statement example
    switch (...)
    {
        case ... :
            break;
        /// put a blank line after each case.
        case ... :
            break;
        
        /// It is often useful to do this:
        case 3:
        {
            int i;  /// Need braces for the declaraction
            break;
        }
        
        /// Use default only when the compiler expects it.
        /// Don't do default: break; unless switching on an enum (compiler will
        /// warn if you don't do this).
    }
}
