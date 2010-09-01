#include "StripChart.hpp"

#include <QPainter>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <protobuf/LogFrame.pb.h>
#include <Geometry2d/Point.hpp>

#include <google/protobuf/descriptor.h>

using namespace std;
using namespace Packet;
using namespace boost;
using namespace google::protobuf;

StripChart::StripChart(QWidget* parent)
{
	_history = 0;
	_minValue = 0;
	_maxValue = 1;
	_function = 0;
	autoRange = true;
	_color = Qt::yellow;
	
	QPalette p = palette();
	p.setColor(QPalette::Window, Qt::black);
	setPalette(p);
	setAutoFillBackground(true);
	
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	setMinimumSize(100, 100);
}

StripChart::~StripChart()
{
	function(0);
}

void StripChart::function(Chart::Function* function)
{
	if (_function)
	{
		delete _function;
	}
	
	_function = function;
}

QPointF StripChart::dataPoint(int i, float value)
{
	float x = i * width() / _history->size();
	int h = height();
	float y = h - (value - _minValue) * h / (_maxValue - _minValue);
	return QPointF(x, y);
}

void StripChart::paintEvent(QPaintEvent* e)
{
	if (!_history || _history->empty() || !_function)
	{
		return;
	}
	
	QPainter p(this);
	
	// X-axis
	p.setPen(Qt::gray);
	QPointF x = dataPoint(0, 0);
	p.drawLine(x, QPointF(width(), x.y()));
	
	p.setPen(_color);
	QPointF last;
	bool haveLast = false;
	float newMin = _minValue;
	float newMax = _maxValue;
	for (unsigned int i = 0; i < _history->size(); ++i)
	{
		float v = 0;
		if (_history->at(i) && _function->value(*_history->at(i).get(), v))
		{
			if (autoRange)
			{
				newMin = min(newMin, v);
				newMax = max(newMax, v);
			}
			
			QPointF pt = dataPoint(i, v);
			if (haveLast)
			{
				p.drawLine(last, pt);
			}
			last = pt;
			haveLast = true;
		} else {
			haveLast = false;
		}
	}
	
	_minValue = newMin;
	_maxValue = newMax;
}

////////

bool Chart::PointMagnitude::value(const Packet::LogFrame& frame, float& v) const
{
	const Message *msg = &frame;
	for (int i = 0; i < path.size(); ++i)
	{
		const Reflection *ref = msg->GetReflection();
		const Descriptor *desc = msg->GetDescriptor();
		
		int tag = path[i];
		const FieldDescriptor *fd = desc->FindFieldByNumber(tag);
		if (fd->type() != FieldDescriptor::TYPE_MESSAGE)
		{
			fprintf(stderr, "PointMagnitude: expected a message field\n");
			return false;
		}
		
		if (fd->is_repeated())
		{
			++i;
			if (i >= path.size())
			{
				fprintf(stderr, "PointMagnitude: ends after tag for repeated field without giving index\n");
				return false;
			}
			int j = path[i];
			if (ref->FieldSize(*msg, fd) <= j)
			{
				// Not enough items
				return false;
			}
			msg = &ref->GetRepeatedMessage(*msg, fd, j);
		} else {
			if (!ref->HasField(*msg, fd))
			{
				// Missing field
				return false;
			}
			msg = &ref->GetMessage(*msg, fd);
		}
	}
	
	if (msg->GetDescriptor()->name() != "Point")
	{
		fprintf(stderr, "PointMagnitude: path ended in a message other than Point\n");
		return false;
	}
	
	v = Geometry2d::Point(*static_cast<const Packet::Point *>(msg)).mag();
	return true;
}

bool Chart::NumericField::value(const Packet::LogFrame& frame, float& v) const
{
	const Message *msg = &frame;
	for (int i = 0; i < path.size(); ++i)
	{
		const Reflection *ref = msg->GetReflection();
		const Descriptor *desc = msg->GetDescriptor();
		
		int tag = path[i];
		const FieldDescriptor *fd = desc->FindFieldByNumber(tag);
		if (fd->type() != FieldDescriptor::TYPE_MESSAGE)
		{
			if ((i == path.size() - 1 && !fd->is_repeated()) || (i == path.size() - 2 && fd->is_repeated()))
			{
				// End of the path
				if (fd->is_repeated())
				{
					++i;
					if (i >= path.size())
					{
						fprintf(stderr, "NumericField: ends after tag for repeated field without giving index\n");
						return false;
					}
					int j = path[i];
					if (ref->FieldSize(*msg, fd) <= j)
					{
						// Not enough items
						return false;
					}
					
					switch (fd->type())
					{
						case FieldDescriptor::TYPE_FLOAT:
							v = ref->GetRepeatedFloat(*msg, fd, j);
							break;
						
						case FieldDescriptor::TYPE_DOUBLE:
							v = ref->GetRepeatedDouble(*msg, fd, j);
							break;
						
						default:
							fprintf(stderr, "NumericField: unsupported repeated field type %d\n", fd->type());
							return false;
					}
				} else {
					switch (fd->type())
					{
						case FieldDescriptor::TYPE_FLOAT:
							v = ref->GetFloat(*msg, fd);
							break;
						
						case FieldDescriptor::TYPE_DOUBLE:
							v = ref->GetDouble(*msg, fd);
							break;
						
						default:
							fprintf(stderr, "NumericField: unsupported field type %d\n", fd->type());
							return false;
					}
				}
				return true;
			} else {
				// Non-message field in the middle of a path
				fprintf(stderr, "NumericField: expected a message field\n");
				return false;
			}
		}
		
		if (fd->is_repeated())
		{
			++i;
			if (i >= path.size())
			{
				fprintf(stderr, "NumericField: ends after tag for repeated field without giving index\n");
				return false;
			}
			int j = path[i];
			if (ref->FieldSize(*msg, fd) <= j)
			{
				// Not enough items
				return false;
			}
			msg = &ref->GetRepeatedMessage(*msg, fd, j);
		} else {
			if (!ref->HasField(*msg, fd))
			{
				// Missing field
				return false;
			}
			msg = &ref->GetMessage(*msg, fd);
		}
	}
	
	// Not reached
	return false;
}
