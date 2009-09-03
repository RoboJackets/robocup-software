#include "Identifier.hpp"

#include <QFile>
#include <QDomElement>
#include <QDomDocument>
#include <boost/format.hpp>
#include <stdexcept>

#include "Vector_ID.h"

using namespace Vision;
using namespace boost;
using namespace std;

Identifier* Identifier::load(const char* filename, Process* process, Color center)
{
	Identifier* ident = 0;
	
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		throw runtime_error(str(format("Can't read from %s") % filename));
	}
	
	QDomDocument document;
	
	if (!document.setContent(&file))
	{
		file.close();
		
		//throw excpetion
	}
	
	file.close();
	
	QDomElement element = document.documentElement();
	if (element.tagName() != "ident")
	{
		//expected ident tag
	}
	else
	{
		//TODO, loop through list of identifiers and select right one
		if (element.attribute("type") == "VectorId")
		{
			ident = Vector_ID::load(element, process, center);
		}
	}
	
	return ident;
}
