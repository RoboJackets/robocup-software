#pragma once

#include "FieldBackgroundView.hpp"

#include <Geometry2d/Point.hpp>
#include <protobuf/LogFrame.pb.h>

#include <set>
#include <memory>

class Logger;

/** class that performs drawing of log data onto the field */
class FieldView : public FieldBackgroundView
{
	public:
		FieldView(QWidget* parent = nullptr);

        /// Visibility of debug layers
		void layerVisible(int i, bool value);
		bool layerVisible(int i) const;

		void history(const std::vector<std::shared_ptr<Packet::LogFrame> > *value)
		{
			_history = value;
		}

		// True if this control is showing live (vs. historical) data.
		// If false, it will draw a red border.
		bool live;

		bool showRawRobots;
		bool showRawBalls;
		bool showCoords;
		bool showDotPatterns;
		bool showTeamNames;

	protected:
		virtual void paintEvent(QPaintEvent* e);

		virtual void drawWorldSpace(QPainter &p);
		virtual void drawTeamSpace(QPainter &p);

		void drawText(QPainter& p, QPointF pos, QString text, bool center, bool worldSpace);
		void drawRobot(QPainter& p, bool blueRobot, int ID, QPointF pos, float theta, bool hasBall = false, bool faulty = false);
		void drawCoords(QPainter& p, bool worldSpace);

	protected:
		// Returns a pointer to the most recent frame, or null if none is available.
		std::shared_ptr<Packet::LogFrame> currentFrame();

		const std::vector<std::shared_ptr<Packet::LogFrame> > *_history;

		QVector<bool> _layerVisible;
};
