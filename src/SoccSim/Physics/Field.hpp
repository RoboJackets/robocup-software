#ifndef FIELD_HPP_
#define FIELD_HPP_

#include "Entity.hpp"

#include <QVector>

namespace Physics
{
	class Field : public Entity
	{
		public:
			Field(dWorldID world, dSpaceID space);
			~Field();

			void paint();

		private:
			static void wall(dSpaceID space, float w, float l, float x, float y);
	};
}

#endif /*FIELD_HPP_*/
