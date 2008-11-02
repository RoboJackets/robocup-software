#ifndef IDENTIFIER_HPP_
#define IDENTIFIER_HPP_

#include <vector>

#include "../Colors.h"

namespace Vision
{
	class Process;
	
	/** Generic base for a pattern identification scheme */
	class Identifier
	{
		public:
			class Robot
			{
				public:
					/** shell id of the robot */
					int id;
					
					/** x and y global field position */
					float x,y;
					
					/** global angle (+-180 deg) */
					float angle;
			};
		
		public:
			unsigned int numRobots() const { return _robots.size(); }
			const std::vector<Identifier::Robot>& robots() const { return _robots; } 
			
			/** load an identifier from a file, return the created identifier 
			 *  Calling process must destroy the identifier */
			static Identifier* load(const char* filename, Process* process, Color center);
			
			virtual void run() = 0;
			
		protected:
			Identifier() {};
			
			/** load the identifier for the element */
			//void load(QDomElement element);
			
		/// members ///
		protected:
			/** list of identified robots */
			std::vector<Identifier::Robot> _robots;
	};
}

#endif /* IDENTIFIER_HPP_ */
