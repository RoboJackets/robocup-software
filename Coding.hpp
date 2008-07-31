// Explain /// instead of //

/// This file describles the coding style and conventions used throughout the
/// project. Style and convention are a strong recommendation and the tips are
/// a suggestion.

/// ! Always use header guards !
/// Format: NAMESPACE_FILE_HPP
/// If no namespace, then Format: _FILE_HPP
#ifndef _CODING_HPP
#define _CODING_HPP

/// Includes should come right after the header guards
#include <GlobalInclude>
#include "LocalInclude.hpp"

/// !!! DO NOT make typedef with pointer implied !!!
//typedef SomeType* NewName;

/// !!! Instead use
typedef SomeType NewName;

/// This is a global constant (recommended)
const float GlobalConstant = 1.0f;
/// Alternative (optional)
#define GlobalConstant ((float)(1.0f))

/// All header comments should be done using the standard Doxygen format.
/// A doxygen comment starts with /** and ends with */
/// Multiline comments should have a leading * on each line

/// The classname should match the filename
/// General naming is camel case style

/// Header files should try to contain only interface declarations and not
/// implementations. Exceptions can include const getters

/// Brackets means an increase in indentation level ... ALWAYS
/// Indentation is 4 spaces, no tab characters

/// TIPS ///
/// * Prefer references to pointers.
/// * Use const where applicable

/** This is a sample doxygen comment for the class file
 *  This is the second line
 *  This is the last line */
class Coding
{
        /// Within the class, there are 3 types of entities
        /// types, methods, and members (in that order)
        /// Each section should have its own visibility lavel(s)
        /// If a visibility level is empty it can be omitted

        /// types ///
    public:
        /** comment for the enum */
        typedef enum
        {
            EnumMember,
            AnotherEnumMember
        } EnumType;

    protected:
        /** comment for the struct */
        typedef struct
        {
            int structMember;
        } AStruct;

    private:
        class PrivateClass
        {
                ...
        }

        /// methods ///
    public:
        Coding();
        ~Coding();

        /** getter for _privateMember (const is suggested)
         *  If getPrivateMember makes more sense in context, use that */
        int privateMember() const;
        /** setter for _privateMember
         *  If setPrivateMember makes more sense in context, use that */
        void privateMember (const int val);

        /// members ///
    public:
        /// public members (classes and structs) don't use leading '_'
        float publicMember;

    private:
        int _privateMember;

        /// references (&) and pointers (*) should follow the typename
        /// they are a modification of the type

        /// Only declare one variable per line.

        /** comment for _anotherPrivateMember */
        AStruct* _anotherPrivateMember;
        EnumType& _referenceExample;

        static char _privateStatic;
}

#endif /* _CODING_HPP */