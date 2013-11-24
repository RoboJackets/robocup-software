// Explain /// instead of //

/// This file describles the coding style and conventions used throughout the
/// project. Style and convention are a strong recommendation and the tips are
/// a suggestion.

/// ! Always use pragma once to ensure that the file cant be over-included !
#pragma once

/// Includes should come right after the header guards
#include <GlobalInclude>
#include "LocalInclude.hpp"

/// !!! DO NOT make typedef with pointer implied !!!
//typedef SomeType* NewName;

/// !!! Instead use
typedef SomeType NewName;

/// This is a global constant (recommended)
const float GlobalConstant = 1.0f;
/// Alternative (always use parenthesis)
#define GlobalConstant ((float)(1.0f))

/// All header comments should be done using the standard Doxygen format.
/// A doxygen comment starts with /** and ends with */
/// Multiline comments should have a leading * on each line

/// The classname should match the filename
/// General naming is camel case style

/// Header files should try to contain only interface declarations and not
/// implementations. Exceptions can include const getters

/// Brackets means an increase in indentation level ... ALWAYS
/// Leading Indentation is always a tab character
/// Any indentatio after that should be spaces

/// TIPS ///
/// * Prefer references to pointers.
/// * Use const where applicable

/** This is a sample doxygen comment for the class file
 *  This is the second line
 *  This is the last line */
class CodingStyle
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
		CodingStyle();
		~CodingStyle();
		
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
