
#pragma once

#include <vector>
#include "WindowEvaluator.hpp"
#include "ReceivePointEvaluator.hpp"
#include <Geometry2d/Segment.hpp>

#include <Utils.hpp>



namespace Gameplay {


	class PointEvaluator {
	public:
		virtual ~PointEvaluator();
		virtual float evaluatePoint(Geometry2d::Point &pt) = 0;
	};



	//======================================================================



	class ShotVantagePointEvaluator : public PointEvaluator {
	public:
		ShotVantagePointEvaluator(SystemState *state) : _windowEvaluator(state) {

		}

		ShotVantagePointEvaluator(ShotVantagePointEvaluator &other) : _windowEvaluator(other._windowEvaluator.state()) {

		}


		virtual float evaluatePoint(Geometry2d::Point &pt) {
			Geometry2d::Segment goalSegment(Geometry2d::Point(-Field_GoalWidth/2.0,6.05), Geometry2d::Point(Field_GoalWidth/2.0,6.05));

			_windowEvaluator.clear();
			_windowEvaluator.exclude.clear();
			_windowEvaluator.exclude.push_back(pt);
			_windowEvaluator.run(pt, goalSegment);


			float score = 0;
			if ( _windowEvaluator.windows.size() > 0 ) {
				score = _windowEvaluator.best()->segment.length();
			} else {
				score = 0;
			}

			return score;
		}

	private:
		WindowEvaluator _windowEvaluator;
	};



	//======================================================================



	class PassReceivePointEvaluator : public PointEvaluator {
	public:
		PassReceivePointEvaluator(SystemState *state) : _windowEvaluator(state) {}

		PassReceivePointEvaluator(PassReceivePointEvaluator &other) : _windowEvaluator(other._windowEvaluator.state()) {}


		virtual float evaluatePoint(Geometry2d::Point &pt) {


			//	FIXME: implement


			return 0;
		}

		//	FIXME: ?
		WindowEvaluator _windowEvaluator;
	};


	
	//======================================================================	



	class NotInGoalieBoxPointEvaluator : public PointEvaluator {
	public:
		virtual float evaluatePoint(Geometry2d::Point &pt) {
			return ballIsInTheirGoalieBox(pt) ? -INFINITY : 0;
		}
	};


	//======================================================================



	// typedef float (*PointEvaluationFunction)(Geometry2d::Point &pt);

	// template<PointEvaluationFunction func>
	// class PointEvaluatorImpl : public PointEvaluator {
	// public:

	// };



	//======================================================================



	class FieldEvaluator {
	public:
		FieldEvaluator(SystemState *state, bool includeDefaultEvaluators = true) {

			_systemState = state;

			if ( includeDefaultEvaluators ) {
				//	vantage point on the goal
				ShotVantagePointEvaluator *shot = new ShotVantagePointEvaluator(state);
				addPointEvaluator(shot);

				//	invalidates the point if it's in the goalie box
				NotInGoalieBoxPointEvaluator *notInGoalieBox = new NotInGoalieBoxPointEvaluator();
				addPointEvaluator(notInGoalieBox);

				PassReceivePointEvaluator *passEval = new PassReceivePointEvaluator(state);
				addPointEvaluator(passEval);
			}

			setPointEvaluationSpacing(.1);	//	evaluate every 10cm

			visualize = false;
		}


		~FieldEvaluator() {
			removeAllPointEvaluators();
		}


		void removeAllPointEvaluators() {
			for (PointEvaluator *ptEval :  _pointEvaluators) {
				delete ptEval;
			}
			_pointEvaluators.clear();
		}



		bool visualize;


		//	note: takes ownership of ptEval
		void addPointEvaluator(PointEvaluator *ptEval, float weight = 1) {
			_pointEvaluators.push_back(ptEval);
			_pointEvaluatorWeights.push_back(weight);
		}



		float evaluatePoint(Geometry2d::Point &pt) {
			float score = 0;
			for ( unsigned int i = 0; i < _pointEvaluators.size(); i++ ) {
				PointEvaluator *evaluator = _pointEvaluators[i];
				float weight = _pointEvaluatorWeights[i];
				score += evaluator->evaluatePoint(pt) * weight;
			}

			return score;
		}



		void setPointEvaluationSpacing(float spacing) {
			spacing = std::max(0.0f, spacing);
			spacing = std::min(0.5f, spacing);
			_evaluationSpacing = spacing;
		}

		float pointEvaluationSpacing() const {
			return _evaluationSpacing;
		}


		Geometry2d::Point bestPointInRect(Geometry2d::Rect &rect, float *scoreOut = NULL) {
			float maxScore = -1000;
			Geometry2d::Point bestPoint(-1, -1);
			float dx = _evaluationSpacing, dy = _evaluationSpacing;
			for ( float x = rect.minx(); x < rect.maxx(); x += dx ) {
				for ( float y = rect.miny(); y < rect.maxy(); y += dy ) {
					Geometry2d::Point pt(x, y);
					float score = evaluatePoint(pt);

					if ( score > maxScore ) {
						maxScore = score;
						bestPoint = pt;
					}
				}
			}


			if ( visualize ) {
				for ( float x = rect.minx(); x < rect.maxx(); x += dx ) {
					for ( float y = rect.miny(); y < rect.maxy(); y += dy ) {
						Geometry2d::Point pt(x, y);
						float score = evaluatePoint(pt);

						float normalizedScore = score / maxScore;
						normalizedScore = std::max(score, 0.0f);


						QColor color(normalizedScore * 255, 0, (1 - normalizedScore)*255);
						_systemState->drawCircle(pt, .02, color, "FieldEvaluator");
					}
				}
			}


			if ( scoreOut ) *scoreOut = maxScore;
			return bestPoint;
		}


	private:
		SystemState *_systemState;
		std::vector<PointEvaluator *> _pointEvaluators;
		std::vector<float> _pointEvaluatorWeights;
		float _evaluationSpacing;
	};

}

